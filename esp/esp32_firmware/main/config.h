#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define PI_UART_PORT    UART_NUM_0
#define BAUD_RATE       115200
#define BUF_SIZE        1024
#define MIC_SAMPLE_RATE 16000

// 함수 선언
void uart_comm_init(void);
void cmd_control_task(void *arg);
void audio_read_task(void *arg);

#endif // CONFIG_H
