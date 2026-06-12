#include "config.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "audio_element_iface.h"
#include "raw_stream.h"

void audio_read_task(void *arg) {
    audio_element_handle_t raw_writer_el = (audio_element_handle_t)arg;
    int16_t *mic_buf = (int16_t *)calloc(1, BUF_SIZE);
    TickType_t last_send_time = 0;
    float prev_rms_db = -96.0f;

    while (1) {
        int bytes_read = raw_stream_read(raw_writer_el, (char *)mic_buf, BUF_SIZE);
        if (bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int16_t);
            int64_t sum_squares = 0;
            for (int i = 0; i < total_samples; i++) sum_squares += (int64_t)mic_buf[i] * mic_buf[i];
            
            float current_rms = sqrtf((float)sum_squares / total_samples);
            float current_rms_db = (current_rms > 0.0f) ? 20.0f * log10f(current_rms / 32768.0f) : -96.0f;
            float delta_rms_db = current_rms_db - prev_rms_db;
            prev_rms_db = current_rms_db;

            TickType_t now = xTaskGetTickCount();
            if (now - last_send_time > pdMS_TO_TICKS(250)) {
                char uart_buf[64];
                int ulen = snprintf(uart_buf, sizeof(uart_buf), "RMS_DB=%.2f,DRMS_DB=%.2f\n", current_rms_db, delta_rms_db);
                uart_write_bytes(PI_UART_PORT, uart_buf, ulen);
                last_send_time = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    free(mic_buf);
    vTaskDelete(NULL);
}
