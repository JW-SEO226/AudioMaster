idf_component_register(
    SRCS "SmartSpeakerSystem.c"
    INCLUDE_DIRS "."
    REQUIRES
        audio_pipeline
        audio_stream
        audio_hal
        audio_board
        bluetooth_service
        esp_peripherals    # ← 반드시 추가
        driver
        nvs_flash
        bt
)