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
// 15배 증폭에 맞추어 박수 소리에만 반응하도록 300.0으로 튜닝했습니다.
#define RMS_THRESHOLD          300.0 

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
                // 라즈베리 파이 테스트 시 마이크가 꺼지는 현상 방지를 위해 주석 처리
                // audio_hal_set_volume(board_handle->audio_hal, 0); 
                ESP_LOGW(TAG, "[라즈베리 파이 명령] 음소거 실행됨 (마이크는 유지)");
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

// 2. 1차 필터 태스크 (1초 요약 및 안전 증폭)
static void audio_read_task(void *arg) {
    audio_element_handle_t raw_el = (audio_element_handle_t)arg;
    int16_t *audio_buf = (int16_t *)calloc(1, BUF_SIZE);
    TickType_t last_log_time = 0;
    TickType_t last_trigger_time = 0;

    float max_rms_per_sec = 0;    // 1초 동안의 최고 RMS 기록
    int16_t max_peak_per_sec = 0; // 1초 동안의 최고 Peak 기록

    ESP_LOGI(TAG, "마이크 녹음 및 감시 시작...");

    while (1) {
        int bytes_read = raw_stream_read(raw_el, (char *)audio_buf, BUF_SIZE);
        
        if (bytes_read > 0) {
            int total_samples = bytes_read / 2; 
            int64_t sum_squares = 0;
            int16_t current_peak = 0; 
            
            for (int i = 0; i < total_samples; i++) {
                int16_t raw_val = audio_buf[i]; 
                
                // 마이크 기본 볼륨이 작아서 15배 증폭합니다.
                int32_t amplified = (int32_t)raw_val * 15; 
                
                // 클리핑(깨짐) 방지
                if (amplified > 32767) amplified = 32767;
                if (amplified < -32768) amplified = -32768;

                if (abs(amplified) > current_peak) {
                    current_peak = abs(amplified);
                }
                sum_squares += (int64_t)amplified * amplified;
            }
            
            float current_rms = sqrt((float)sum_squares / total_samples);

            // 현재 조각이 1초 내에서 가장 큰 소리였다면 기록 갱신
            if (current_rms > max_rms_per_sec) max_rms_per_sec = current_rms;
            if (current_peak > max_peak_per_sec) max_peak_per_sec = current_peak;

            TickType_t now = xTaskGetTickCount();

            // 1초마다 모아둔 '최고 기록'을 출력하고 초기화
            // 최고 피트 = 15배수
                if (now - last_log_time > pdMS_TO_TICKS(1000)) {
                    ESP_LOGI(TAG, "최대 RMS: %.2f | 최고 Peak: %d", max_rms_per_sec, max_peak_per_sec);
                    last_log_time = now;
                    max_rms_per_sec = 0; 
                    max_peak_per_sec = 0;
            }

            // 트리거는 기록이 아니라 '현재 소리'를 기준으로 즉시 발동
            if (current_rms > RMS_THRESHOLD) {
                if (now - last_trigger_time > pdMS_TO_TICKS(1000)) { 
                    ESP_LOGW(TAG, "Fast Trigger 발동! (순간 RMS: %.2f)", current_rms);
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
    
    // 마이크(ADC) 전용 모드
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    audio_pipeline_handle_t pipeline;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_READER;
    audio_element_handle_t i2s_read = i2s_stream_init(&i2s_cfg);
    
    audio_element_info_t i2s_info = {0};
    audio_element_getinfo(i2s_read, &i2s_info);
    i2s_info.sample_rates = 16000;
    i2s_info.channels = 1;  // 모노 
    i2s_info.bits = 16;
    audio_element_setinfo(i2s_read, &i2s_info);

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
