#include "config.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "board.h"
#include "audio_hal.h"

audio_board_handle_t board_handle;

void app_main(void) {
    // 1. 기본 시스템 및 블루투스 초기화
    nvs_flash_init();
    esp_bt_controller_init(&(esp_bt_controller_config_t)BT_CONTROLLER_INIT_CONFIG_DEFAULT());
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    // 2. 오디오 보드 초기화
    board_handle = audio_board_init();
    audio_hal_set_volume(board_handle->audio_hal, 60);

    // 3. 오디오 파이프라인 구축 (I2S -> RAW)
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t g_recorder_pipeline = audio_pipeline_init(&pipeline_cfg);
    
    i2s_stream_cfg_t i2s_read_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(I2S_NUM_1, MIC_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
    audio_element_handle_t g_i2s_read = i2s_stream_init(&i2s_read_cfg);
    
    raw_stream_cfg_t raw_write_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_write_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t g_raw_writer = raw_stream_init(&raw_write_cfg);
    
    audio_pipeline_register(g_recorder_pipeline, g_i2s_read, "i2s_read");
    audio_pipeline_register(g_recorder_pipeline, g_raw_writer, "raw_write");
    audio_pipeline_link(g_recorder_pipeline, (const char *[]){"i2s_read", "raw_write"}, 2);
    
    // 4. UART 초기화 및 파이프라인 가동
    uart_comm_init();
    audio_pipeline_run(g_recorder_pipeline);
    
    // 5. 각각의 기능을 담당하는 태스크(스레드) 실행
    xTaskCreate(cmd_control_task, "cmd_task", 4096, NULL, 10, NULL);
    xTaskCreate(audio_read_task, "audio_read", 4096, g_raw_writer, 10, NULL);
}
