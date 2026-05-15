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
// 💡 이제 진짜 소리가 들어오므로 임계값을 1000으로 높입니다.
#define RMS_THRESHOLD          1000.0 

audio_board_handle_t board_handle;

// 1. 통신 및 제어 태스크
static void cmd_control_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    
    while (1) {
        int len = uart_read_bytes(COMMAND_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            char *msg = (char *)data;
            ESP_LOGI(TAG, "명령 수신: %s", msg);

            if (strstr(msg, "DUCKING") != NULL) {
                audio_hal_set_volume(board_handle->audio_hal, 0);
                ESP_LOGW(TAG, "사이렌 발생! 스피커 음소거 실행");
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:MUTED\n", 10);
            } 
            else if (strstr(msg, "NORMAL") != NULL) {
                audio_hal_set_volume(board_handle->audio_hal, 70);
                ESP_LOGI(TAG, "상황 종료. 볼륨 복구");
                uart_write_bytes(COMMAND_UART_PORT_NUM, "ACK:NORMAL\n", 11);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
    vTaskDelete(NULL);
}

// 2. 1차 필터 태스크 (비트 자동 추적 로직 탑재)
static void audio_read_task(void *arg) {
    audio_element_handle_t raw_el = (audio_element_handle_t)arg;
    uint8_t *audio_buf = (uint8_t *)calloc(1, BUF_SIZE);
    TickType_t last_log_time = 0;
    TickType_t last_trigger_time = 0;

    ESP_LOGI(TAG, "마이크 녹음 및 비트 추적 감시 시작...");

    while (1) {
        int bytes_read = raw_stream_read(raw_el, (char *)audio_buf, BUF_SIZE);
        
        if (bytes_read > 0) {
            int16_t *buf_16 = (int16_t *)audio_buf;
            int32_t *buf_32 = (int32_t *)audio_buf;

            int16_t peak_16 = 0;
            int16_t peak_32 = 0;

            // 1. 하단 16비트(찌꺼기)의 최대값 확인
            for (int i = 0; i < bytes_read / 2; i++) {
                if (abs(buf_16[i]) > peak_16) peak_16 = abs(buf_16[i]);
            }

            // 2. 상단 16비트(진짜 소리가 숨은 곳)의 최대값 확인
            for (int i = 0; i < bytes_read / 4; i++) {
                int16_t upper_16 = (int16_t)(buf_32[i] >> 16);
                if (abs(upper_16) > peak_32) peak_32 = abs(upper_16);
            }

            // 💡 소리가 32비트 상단에 숨어있는지 자동 판별!
            int is_32bit_hidden = (peak_32 > peak_16 * 10); 
            int num_samples = is_32bit_hidden ? (bytes_read / 4) : (bytes_read / 2);
            
            int64_t sum_squares = 0;
            int16_t final_peak = 0;

            // 판별된 진짜 데이터만 뽑아서 RMS를 계산합니다. (억지 증폭 100배 삭제!)
            for (int i = 0; i < num_samples; i++) {
                int16_t raw_val = is_32bit_hidden ? (int16_t)(buf_32[i] >> 16) : buf_16[i];

                if (abs(raw_val) > final_peak) final_peak = abs(raw_val);
                sum_squares += (int64_t)raw_val * raw_val;
            }
            
            float rms = sqrt((float)sum_squares / num_samples);
            TickType_t now = xTaskGetTickCount();

            // 1초마다 출력 (어디서 데이터를 뽑았는지 표시)
            if (now - last_log_time > pdMS_TO_TICKS(1000)) {
                if (is_32bit_hidden) {
                    ESP_LOGI(TAG, "🎯 [32비트 숨김 발견!] RMS: %.2f | ⚡ 진짜 Peak: %d", rms, final_peak);
                } else {
                    ESP_LOGI(TAG, "🎤 [일반 16비트] RMS: %.2f | ⚡ Peak: %d", rms, final_peak);
                }
                last_log_time = now;
            }

            if (rms > RMS_THRESHOLD) {
                if (now - last_trigger_time > pdMS_TO_TICKS(1000)) { 
                    ESP_LOGW(TAG, "🚨 Fast Trigger 발동! (RMS: %.2f)", rms);
                    uart_write_bytes(COMMAND_UART_PORT_NUM, "TRIGGER\n", 8);
                    last_trigger_time = now;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    free(audio_buf);
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    board_handle = audio_board_init();
    
    // 마이크와 스피커 모두 정상 작동하도록 BOTH로 기동하고, ADC/DAC 볼륨을 최대로 확보
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 100);

    audio_pipeline_handle_t pipeline;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    audio_element_handle_t i2s_read = i2s_stream_init(&i2s_cfg);
    
    audio_element_info_t i2s_info = {0};
    audio_element_getinfo(i2s_read, &i2s_info);
    i2s_info.sample_rates = 16000;
    i2s_info.channels = 1; 
    i2s_info.bits = 16;
    audio_element_setinfo(i2s_read, &i2s_info);

    // 하드웨어 클럭 강제 동기화 (필수)
    i2s_stream_set_clk(i2s_read, 16000, 16, 1);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER; 
    audio_element_handle_t raw_read = raw_stream_init(&raw_cfg);

    audio_pipeline_register(pipeline, i2s_read, "i2s");
    audio_pipeline_register(pipeline, raw_read, "raw");
    const char *link_tag[] = {"i2s", "raw"};
    audio_pipeline_link(pipeline, link_tag, 2);

    uart_config_t uart_cfg = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(COMMAND_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(COMMAND_UART_PORT_NUM, &uart_cfg);

    ESP_LOGI(TAG, "통합 시스템 가동 (UART Control Mode)");
    
    audio_pipeline_run(pipeline);
    xTaskCreate(cmd_control_task, "cmd_task", 4096, NULL, 10, NULL);
    xTaskCreate(audio_read_task, "audio_read_task", 4096, raw_read, 10, NULL);
}
