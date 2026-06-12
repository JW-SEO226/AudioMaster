#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "board.h"
#include "audio_hal.h"

extern audio_board_handle_t board_handle; // main.c에서 선언된 변수 참조

void uart_comm_init(void) {
    uart_driver_install(PI_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void cmd_control_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(PI_UART_PORT, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            char *msg = (char *)data;
            if (strstr(msg, "DUCKING:4") != NULL) audio_hal_set_volume(board_handle->audio_hal, 0);
            else if (strstr(msg, "DUCKING:3") != NULL) audio_hal_set_volume(board_handle->audio_hal, 15);
            else if (strstr(msg, "DUCKING:2") != NULL) audio_hal_set_volume(board_handle->audio_hal, 30);
            else if (strstr(msg, "DUCKING:1") != NULL) audio_hal_set_volume(board_handle->audio_hal, 45);
            else if (strstr(msg, "NORMAL") != NULL) audio_hal_set_volume(board_handle->audio_hal, 60);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
    vTaskDelete(NULL);
}
