#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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

static const char *TAG = "SMART_CAR";

// ================================================================
// ✅ 까만색 UART 소켓 (UART0) 전용 통신 설정
// (따로 핀 번호를 지정하지 않아도 까만 소켓으로 자동 연결됩니다)
// ================================================================
#define PI_UART_PORT    UART_NUM_0
#define BAUD_RATE       115200
#define BUF_SIZE        1024
#define MIC_SAMPLE_RATE 16000

audio_board_handle_t board_handle;

static audio_pipeline_handle_t g_play_pipeline     = NULL;
static audio_pipeline_handle_t g_recorder_pipeline = NULL;
static audio_element_handle_t  g_bt_stream_reader  = NULL;
static audio_element_handle_t  g_i2s_write         = NULL;
static audio_element_handle_t  g_i2s_read          = NULL;
static audio_element_handle_t  g_raw_writer        = NULL;

// ==========================================
// Pi 명령 수신 태스크
// ==========================================
static void cmd_control_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(PI_UART_PORT, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            char *msg = (char *)data;

            if (strstr(msg, "DUCKING:4") != NULL) {
                ESP_LOGI(TAG, "DUCKING 4 → MUTE");
                audio_hal_set_volume(board_handle->audio_hal, 0);
            } else if (strstr(msg, "DUCKING:3") != NULL) {
                ESP_LOGI(TAG, "DUCKING 3 → 볼륨 15");
                audio_hal_set_volume(board_handle->audio_hal, 15);
            } else if (strstr(msg, "DUCKING:2") != NULL) {
                ESP_LOGI(TAG, "DUCKING 2 → 볼륨 30");
                audio_hal_set_volume(board_handle->audio_hal, 30);
            } else if (strstr(msg, "DUCKING:1") != NULL) {
                ESP_LOGI(TAG, "DUCKING 1 → 볼륨 45");
                audio_hal_set_volume(board_handle->audio_hal, 45);
            } else if (strstr(msg, "NORMAL") != NULL) {
                ESP_LOGI(TAG, "NORMAL → 볼륨 60 복구");
                audio_hal_set_volume(board_handle->audio_hal, 60);
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

    TickType_t last_send_time        = 0;
    float      max_rms_per_interval  = 0;
    int16_t    max_peak_per_interval = 0;
    float      max_delta_rms         = 0;
    float      prev_rms_db           = -96.0f;

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

            if (current_rms  > max_rms_per_interval) max_rms_per_interval  = current_rms;
            if (current_peak > max_peak_per_interval) max_peak_per_interval = current_peak;
            if (delta_rms_db > max_delta_rms)         max_delta_rms         = delta_rms_db;

            TickType_t now = xTaskGetTickCount();
            if (now - last_send_time > pdMS_TO_TICKS(250)) {

                // ✅ UART0 (까만 소켓)을 통해 파이로 데이터 전송
                char uart_buf[64];
                int ulen = snprintf(uart_buf, sizeof(uart_buf),
                                    "RMS_DB=%.2f,DRMS_DB=%.2f,PEAK=%d\n",
                                    current_rms_db, delta_rms_db, max_peak_per_interval);
                uart_write_bytes(PI_UART_PORT, uart_buf, ulen);

                last_send_time        = now;
                max_rms_per_interval  = 0;
                max_peak_per_interval = 0;
                max_delta_rms         = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    free(mic_buf);
    vTaskDelete(NULL);
}

// ==========================================
// 파이프라인 이벤트 태스크
// ==========================================
static void pipeline_event_task(void *arg) {
    audio_event_iface_handle_t evt = (audio_event_iface_handle_t)arg;
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) continue;

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
            && msg.source == (void *)g_bt_stream_reader
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {

            audio_element_info_t music_info = {0};
            audio_element_getinfo(g_bt_stream_reader, &music_info);
            ESP_LOGI(TAG, "🎵 BT: %dHz %dch %dbit",
                     music_info.sample_rates, music_info.channels, music_info.bits);

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
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ✅ 불필요한 시스템 로그를 줄여서 파이 통신 부하 감소
    esp_log_level_set("*",   ESP_LOG_WARN);
    esp_log_level_set(TAG,   ESP_LOG_INFO);

    ESP_LOGI(TAG, "스마트 스피커 부팅 시작");

    // 1. NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 2. 블루투스 초기화 
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);
    ret = esp_bt_controller_enable(bt_cfg.mode); 
    ESP_ERROR_CHECK(ret);
    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);
    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ 블루투스 초기화 완료");

    // 3. 오디오 보드
    board_handle = audio_board_init();
    if (!board_handle) { ESP_LOGE(TAG, "❌ 오디오 보드 초기화 실패"); return; }
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 60);
    ESP_LOGI(TAG, "✅ 오디오 보드 초기화 완료");

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    // [A] 녹음
    g_recorder_pipeline = audio_pipeline_init(&pipeline_cfg);
    i2s_stream_cfg_t i2s_read_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_1, MIC_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
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

    // [B] 재생
    g_play_pipeline = audio_pipeline_init(&pipeline_cfg);
    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER, .user_callback = {0},
        .audio_hal = board_handle->audio_hal,
    };
    g_bt_stream_reader = a2dp_stream_init(&a2dp_config);

    i2s_stream_cfg_t i2s_write_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0, 44100, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_WRITER);
    i2s_write_cfg.uninstall_drv = false;
    g_i2s_write = i2s_stream_init(&i2s_write_cfg);
    audio_element_info_t i2s_write_info = { .sample_rates = 44100, .channels = 2, .bits = 16 };
    audio_element_setinfo(g_i2s_write, &i2s_write_info);

    audio_pipeline_register(g_play_pipeline, g_bt_stream_reader, "bt");
    audio_pipeline_register(g_play_pipeline, g_i2s_write,        "i2s_write");
    const char *play_link[] = {"bt", "i2s_write"};
    audio_pipeline_link(g_play_pipeline, play_link, 2);

    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt  = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(g_play_pipeline,     evt);
    audio_pipeline_set_listener(g_recorder_pipeline, evt);

    // ✅ 4. UART0 (까만 소켓) 드라이버 설치 (읽기/쓰기 활성화)
    uart_driver_install(PI_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

    esp_bt_dev_set_device_name("ESP_SMART_SPEAKER");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ESP_LOGI(TAG, "🚀 시스템 가동! 블루투스를 연결하세요.");

    audio_pipeline_run(g_recorder_pipeline);
    audio_pipeline_run(g_play_pipeline);

    xTaskCreate(cmd_control_task,    "cmd_task",   4096, NULL,          10, NULL);
    xTaskCreate(audio_read_task,     "audio_read", 4096, g_raw_writer,  10, NULL);
    xTaskCreate(pipeline_event_task, "event_task", 4096, evt,           9, NULL);
    }
