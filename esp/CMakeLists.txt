cmake_minimum_required(VERSION 3.16)

# 1. ADF 경로 설정
set(ADF_PATH "C:/esp/esp-adf")

# 2. ADF 부품 창고 지정
set(EXTRA_COMPONENT_DIRS "${ADF_PATH}/components")

# 3. IDF 설정 로드
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(SmartSpeaker_UART)

# 옛날 명칭 강제 치환 및 컴파일러 에러 무시
add_compile_definitions(
    xSemaphoreHandle=SemaphoreHandle_t
    portTICK_RATE_MS=portTICK_PERIOD_MS
)

add_compile_options(
    "-Wno-error=return-type"
    "-Wno-return-type"
    "-Wno-error=int-conversion"
    "-Wno-error=absolute-value" 
)