#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "board.h"
#include "audio_hal.h"

static const char *TAG = "SMART_CAR_SPEAKER";

#define COMMAND_UART_PORT_NUM  UART_NUM_0
#define BAUD_RATE              115200
#define BUF_SIZE               1024
#define RMS_THRESHOLD          300.0
#define SAMPLE_RATE            16000

audio_board_handle_t board_handle;
audio_element_handle_t raw_play_el;

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
                ESP_LOGW(TAG, "[라즈베리파이 명령] 음소거 실행");
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:MUTED\n", 10);
            } else if (strstr(msg, "NORMAL") != NULL) {
                ESP_LOGI(TAG, "상황 종료. 볼륨 복구");
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:NORMAL\n", 11);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
    vTaskDelete(NULL);
}

// ==========================================
// 440Hz 테스트 톤 재생 태스크
// ==========================================
static void audio_play_task(void *arg) {
    int16_t *play_buf = (int16_t *)malloc(BUF_SIZE);
    float frequency = 440.0f;
    int sample_count = 0;
    int samples = BUF_SIZE / sizeof(int16_t);

    ESP_LOGI(TAG, "440Hz 테스트 톤 재생 시작");

    while (1) {
        for (int i = 0; i < samples; i++) {
            play_buf[i] = (int16_t)(10000.0f * sinf(2.0f * M_PI * frequency * sample_count / SAMPLE_RATE));
            sample_count++;
        }
        // raw_stream_write는 내부적으로 블로킹 → vTaskDelay 불필요
        raw_stream_write(raw_play_el, (char *)play_buf, BUF_SIZE);
    }
    free(play_buf);
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
    float   max_rms_per_sec  = 0;
    int16_t max_peak_per_sec = 0;

    while (1) {
        int bytes_read = raw_stream_read(raw_writer_el, (char *)mic_buf, BUF_SIZE);

        if (bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int16_t);
            int64_t sum_squares = 0;
            int16_t current_peak = 0;

            for (int i = 0; i < total_samples; i++) {
                int16_t val = mic_buf[i];
                if (abs(val) > current_peak) current_peak = abs(val);
                sum_squares += (int64_t)val * val;
            }

            float current_rms = sqrtf((float)sum_squares / total_samples);
            if (current_rms  > max_rms_per_sec)  max_rms_per_sec  = current_rms;
            if (current_peak > max_peak_per_sec)  max_peak_per_sec = current_peak;

            TickType_t now = xTaskGetTickCount();

            if (now - last_log_time > pdMS_TO_TICKS(1000)) {
                ESP_LOGI(TAG, "최대 RMS: %.2f | 최고 Peak: %d", max_rms_per_sec, max_peak_per_sec);
                last_log_time    = now;
                max_rms_per_sec  = 0;
                max_peak_per_sec = 0;
            }

            if (current_rms > RMS_THRESHOLD) {
                if (now - last_trigger_time > pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "외부 소리 감지 (RMS: %.2f)", current_rms);
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

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 60);

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    // ==========================================
    // [A] 녹음 파이프라인 (ES7243 → I2S 포트 1)
    // ==========================================
    audio_pipeline_handle_t recorder_pipeline = audio_pipeline_init(&pipeline_cfg);

    // 수정: IDF5에서 포트는 chan_cfg.id 로 지정
    i2s_stream_cfg_t i2s_read_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_1,                   // ES7243(마이크)가 연결된 I2S 포트 1
        SAMPLE_RATE,
        I2S_DATA_BIT_WIDTH_16BIT,
        AUDIO_STREAM_READER
    );
    audio_element_handle_t i2s_read = i2s_stream_init(&i2s_read_cfg);

    audio_element_info_t i2s_read_info = {
        .sample_rates = SAMPLE_RATE,
        .channels     = 1,   // ES7243: 모노
        .bits         = 16
    };
    audio_element_setinfo(i2s_read, &i2s_read_info);

    raw_stream_cfg_t raw_write_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_write_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t raw_writer = raw_stream_init(&raw_write_cfg);

    audio_pipeline_register(recorder_pipeline, i2s_read,  "i2s_read");
    audio_pipeline_register(recorder_pipeline, raw_writer, "raw_write");
    const char *recorder_link[] = {"i2s_read", "raw_write"};
    audio_pipeline_link(recorder_pipeline, recorder_link, 2);

    // ==========================================
    // [B] 재생 파이프라인 (ES8311 → I2S 포트 0)
    // ==========================================
    audio_pipeline_handle_t play_pipeline = audio_pipeline_init(&pipeline_cfg);

    raw_stream_cfg_t raw_play_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_play_cfg.type = AUDIO_STREAM_READER;
    raw_play_el = raw_stream_init(&raw_play_cfg);

    // 재생은 포트 0 (ES8311), uninstall_drv = false 로 드라이버 공유
    i2s_stream_cfg_t i2s_write_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0,                   // ES8311(스피커)가 연결된 I2S 포트 0
        SAMPLE_RATE,
        I2S_DATA_BIT_WIDTH_16BIT,
        AUDIO_STREAM_WRITER
    );
    i2s_write_cfg.uninstall_drv = false;
    audio_element_handle_t i2s_write = i2s_stream_init(&i2s_write_cfg);

    audio_element_info_t i2s_write_info = {
        .sample_rates = SAMPLE_RATE,
        .channels     = 1,   // ES8311: 모노
        .bits         = 16
    };
    audio_element_setinfo(i2s_write, &i2s_write_info);

    audio_pipeline_register(play_pipeline, raw_play_el, "raw_play");
    audio_pipeline_register(play_pipeline, i2s_write,   "i2s_write");
    const char *play_link[] = {"raw_play", "i2s_write"};
    audio_pipeline_link(play_pipeline, play_link, 2);

    // ==========================================
    // I2S 클럭 설정 (포트별 각각, 모노 1ch)
    // ==========================================
    i2s_stream_set_clk(i2s_read,  SAMPLE_RATE, 16, 1);
    i2s_stream_set_clk(i2s_write, SAMPLE_RATE, 16, 1);

    // ==========================================
    // UART 초기화
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

    ESP_LOGI(TAG, "Full-Duplex 테스트 가동 (LyraT-Mini v1.2 IDF5 수정 버전)");

    audio_pipeline_run(recorder_pipeline);
    audio_pipeline_run(play_pipeline);

    xTaskCreate(cmd_control_task, "cmd_task",        4096, NULL,       10, NULL);
    xTaskCreate(audio_play_task,  "audio_play_task", 4096, NULL,       10, NULL);
    xTaskCreate(audio_read_task,  "audio_read_task", 4096, raw_writer, 10, NULL);
}
