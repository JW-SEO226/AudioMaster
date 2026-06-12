idf_component_register(
    SRCS "SmartSpeakerSystem.c"
    INCLUDE_DIRS "."
    REQUIRES
        audio_pipeline
        audio_stream
        audio_hal
        audio_board
        bluetooth_service
        esp_peripherals   
        driver
        nvs_flash
        bt
)
