#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_peripherals.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "a2dp_stream.h"
#include "board.h"
#include "audio_hal.h"

static const char *TAG = "SMART_CAR_SPEAKER";

#define COMMAND_UART_PORT_NUM  UART_NUM_0
#define BAUD_RATE              115200
#define BUF_SIZE               1024
#define RMS_THRESHOLD          300.0f
#define MIC_SAMPLE_RATE        16000

audio_board_handle_t board_handle;

// 파이프라인/엘리먼트 핸들 전역 선언
static audio_pipeline_handle_t g_play_pipeline     = NULL;
static audio_pipeline_handle_t g_recorder_pipeline = NULL;
static audio_element_handle_t  g_bt_stream_reader  = NULL;
static audio_element_handle_t  g_i2s_write         = NULL;
static audio_element_handle_t  g_i2s_read          = NULL;
static audio_element_handle_t  g_raw_writer        = NULL;

// ==========================================
// UART 명령 수신 태스크
// ==========================================
static void cmd_control_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(COMMAND_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            char *msg = (char *)data;
            if (strstr(msg, "DUCKING") != NULL) {
                ESP_LOGW(TAG, "🚨 볼륨 감소 (Ducking)");
                audio_hal_set_volume(board_handle->audio_hal, 10);
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:MUTED\n", 10);
            } else if (strstr(msg, "NORMAL") != NULL) {
                ESP_LOGI(TAG, "🟢 볼륨 복구");
                audio_hal_set_volume(board_handle->audio_hal, 60);
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:NORMAL\n", 11);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
    vTaskDelete(NULL);
}

// ==========================================
// 마이크 RMS 감지 태스크
// ==========================================
static void audio_read_task(void *arg) {
    audio_element_handle_t raw_writer_el = (audio_element_handle_t)arg;
    int16_t *mic_buf = (int16_t *)calloc(1, BUF_SIZE);

    TickType_t last_log_time     = 0;
    TickType_t last_trigger_time = 0;
    float      max_rms_per_sec   = 0;
    int16_t    max_peak_per_sec  = 0;
    float      max_delta_rms     = 0;
    float      prev_rms_db       = -96.0f;

    while (1) {
        int bytes_read = raw_stream_read(raw_writer_el, (char *)mic_buf, BUF_SIZE);
        if (bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int16_t);
            int64_t sum_squares  = 0;
            int16_t current_peak = 0;

            for (int i = 0; i < total_samples; i++) {
                int16_t val = mic_buf[i];
                if (abs(val) > current_peak) current_peak = abs(val);
                sum_squares += (int64_t)val * val;
            }

            float current_rms    = sqrtf((float)sum_squares / total_samples);
            float current_rms_db = (current_rms > 0.0f)
                                 ? 20.0f * log10f(current_rms / 32768.0f)
                                 : -96.0f;
            float delta_rms_db   = current_rms_db - prev_rms_db;
            prev_rms_db          = current_rms_db;

            if (current_rms  > max_rms_per_sec) max_rms_per_sec  = current_rms;
            if (current_peak > max_peak_per_sec) max_peak_per_sec = current_peak;
            if (delta_rms_db > max_delta_rms)    max_delta_rms    = delta_rms_db;

            TickType_t now = xTaskGetTickCount();
            if (now - last_log_time > pdMS_TO_TICKS(1000)) {
                ESP_LOGI(TAG, "최대 RMS: %.2f | dBFS: %.2f | ΔdBFS: %.2f | Peak: %d",
                         max_rms_per_sec, current_rms_db, max_delta_rms, max_peak_per_sec);
                last_log_time   = now;
                max_rms_per_sec = 0; max_peak_per_sec = 0; max_delta_rms = 0;
            }

            if (current_rms > RMS_THRESHOLD) {
                if (now - last_trigger_time > pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "외부 소리 감지! (RMS: %.2f)", current_rms);
                    uart_write_bytes(COMMAND_UART_PORT_NUM, "TRIGGER\n", 8);
                    last_trigger_time = now;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    free(mic_buf);
    vTaskDelete(NULL);
}

// ==========================================
// 파이프라인 이벤트 리스너 태스크
// ==========================================
static void pipeline_event_task(void *arg) {
    audio_event_iface_handle_t evt = (audio_event_iface_handle_t)arg;

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
            && msg.source == (void *)g_bt_stream_reader
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {

            audio_element_info_t music_info = {0};
            audio_element_getinfo(g_bt_stream_reader, &music_info);
            ESP_LOGI(TAG, "🎵 BT 음악 정보: sample_rate=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_set_music_info(g_i2s_write,
                                         music_info.sample_rates,
                                         music_info.channels,
                                         music_info.bits);
            i2s_stream_set_clk(g_i2s_write,
                               music_info.sample_rates,
                               music_info.bits,
                               music_info.channels);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    // 💡 [디버깅 마법의 2초]
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "🚀 스마트 스피커 부팅 시작...");
    ESP_LOGI(TAG, "===========================================");

    // ==========================================
    // 1. NVS 초기화
    // ==========================================
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // ==========================================
    // 2. 블루투스 스택 초기화 (충돌 방지 최적화 버전)
    // ==========================================
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ 블루투스 컨트롤러 초기화 실패: %s", esp_err_to_name(ret));
        return; 
    }
    
    // 💡 [핵심 해결] 강제로 특정 모드를 요구하지 않고, 시스템 기본 설정(bt_cfg.mode)을 그대로 따름
    ret = esp_bt_controller_enable(bt_cfg.mode); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ 블루투스 활성화 실패: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Bluedroid 초기화 실패: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Bluedroid 활성화 실패: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "✅ 블루투스 스택 초기화 완료!");

    // ==========================================
    // 3. 오디오 보드 초기화
    // ==========================================
    board_handle = audio_board_init();
    if (board_handle == NULL) {
        ESP_LOGE(TAG, "❌ 오디오 보드 초기화 실패! 하드웨어 핀이나 연결을 확인하세요.");
        return;
    }
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 60);
    ESP_LOGI(TAG, "✅ 오디오 보드 초기화 완료!");

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    // ==========================================
    // [A] 녹음 파이프라인 
    // ==========================================
    g_recorder_pipeline = audio_pipeline_init(&pipeline_cfg);
    i2s_stream_cfg_t i2s_read_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(I2S_NUM_1, MIC_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
    g_i2s_read = i2s_stream_init(&i2s_read_cfg);
    audio_element_info_t i2s_read_info = { .sample_rates = MIC_SAMPLE_RATE, .channels = 1, .bits = 16 };
    audio_element_setinfo(g_i2s_read, &i2s_read_info);
    i2s_stream_set_clk(g_i2s_read, MIC_SAMPLE_RATE, 16, 1);

    raw_stream_cfg_t raw_write_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_write_cfg.type = AUDIO_STREAM_WRITER;
    g_raw_writer = raw_stream_init(&raw_write_cfg);

    audio_pipeline_register(g_recorder_pipeline, g_i2s_read,  "i2s_read");
    audio_pipeline_register(g_recorder_pipeline, g_raw_writer, "raw_write");
    const char *recorder_link[] = {"i2s_read", "raw_write"};
    audio_pipeline_link(g_recorder_pipeline, recorder_link, 2);

    // ==========================================
    // [B] 재생 파이프라인 (BT A2DP)
    // ==========================================
    g_play_pipeline = audio_pipeline_init(&pipeline_cfg);

    a2dp_stream_config_t a2dp_config = {
        .type          = AUDIO_STREAM_READER,
        .user_callback = {0},
        .audio_hal     = board_handle->audio_hal,
    };
    g_bt_stream_reader = a2dp_stream_init(&a2dp_config);

    i2s_stream_cfg_t i2s_write_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(I2S_NUM_0, 44100, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_WRITER);
    i2s_write_cfg.uninstall_drv = false;
    g_i2s_write = i2s_stream_init(&i2s_write_cfg);
    audio_element_info_t i2s_write_info = { .sample_rates = 44100, .channels = 2, .bits = 16 };
    audio_element_setinfo(g_i2s_write, &i2s_write_info);

    audio_pipeline_register(g_play_pipeline, g_bt_stream_reader, "bt");
    audio_pipeline_register(g_play_pipeline, g_i2s_write,        "i2s_write");
    const char *play_link[] = {"bt", "i2s_write"};
    audio_pipeline_link(g_play_pipeline, play_link, 2);

    // ==========================================
    // 이벤트 리스너 설정
    // ==========================================
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt  = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(g_play_pipeline,     evt);
    audio_pipeline_set_listener(g_recorder_pipeline, evt);

    // ==========================================
    // 4. UART 초기화
    // ==========================================
    uart_config_t uart_cfg = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(COMMAND_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(COMMAND_UART_PORT_NUM, &uart_cfg);

    esp_bt_dev_set_device_name("ESP_SMART_SPEAKER");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ESP_LOGI(TAG, "모든 파이프라인 가동! 휴대폰에서 블루투스를 연결하세요!");

    audio_pipeline_run(g_recorder_pipeline);
    audio_pipeline_run(g_play_pipeline);

    xTaskCreate(cmd_control_task,    "cmd_task",    4096, NULL,         10, NULL);
    xTaskCreate(audio_read_task,     "audio_read",  4096, g_raw_writer, 10, NULL);
    xTaskCreate(pipeline_event_task, "event_task",  4096, evt,          9, NULL);
}
