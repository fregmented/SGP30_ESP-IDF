cmake_minimum_required(VERSION 3.2)

set(ENV{IDF_PATH} "$ENV{HOME}/esp/esp-idf/")
set(ENV{PATH} "/usr/local/opt/sqlite/bin:/usr/local/opt/openssl/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:/Applications/Wireshark.app/Contents/MacOS:/usr/local/opt/go/libexec/bin:$ENV{HOME}/esp/xtensa-esp32-elf/bin:$ENV{IDF_PATH}/tools")

SET(CMAKE_C_COMPILER "$ENV{HOME}/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc")
SET(CMAKE_CXX_COMPILER "$ENV{HOME}/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++")
SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-fno-rtti -fno-exceptions -std=gnu++11 -Wno-error=deprecated-declarations -Wno-error=unused-function -Wno-unused-parameter -Wno-sign-compare -fstack-protector -Os -g3 -nostdlib -Wpointer-arith -Wno-error=unused-but-set-variable -Wno-error=unused-variable -mlongcalls -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -DARDUINO=10805 -DARDUINO_ARCH_ESP32 -DARDUINO_VARIANT=&quot;esp32thing&quot; -DARDUINO_BOARD=&quot;SparkFun\\ ESP32\\ Thing&quot; -DESP32 -DESP_PLATFORM -DF_CPU=240000000L -DHAVE_CONFIG_H -DMBEDTLS_CONFIG_FILE=&quot;mbedtls/esp_config.h&quot; -DPLATFORMIO=30502 -DARDUINO_ESP32_THING")
SET(CMAKE_C_FLAGS_DISTRIBUTION "-Wno-old-style-declaration -std=gnu99 -Wno-error=deprecated-declarations -Wno-error=unused-function -Wno-unused-parameter -Wno-sign-compare -fstack-protector -Os -g3 -nostdlib -Wpointer-arith -Wno-error=unused-but-set-variable -Wno-error=unused-variable -mlongcalls -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -DARDUINO=10805 -DARDUINO_ARCH_ESP32 -DARDUINO_VARIANT=&quot;esp32thing&quot; -DARDUINO_BOARD=&quot;SparkFun\\ ESP32\\ Thing&quot; -DESP32 -DESP_PLATFORM -DF_CPU=240000000L -DHAVE_CONFIG_H -DMBEDTLS_CONFIG_FILE=&quot;mbedtls/esp_config.h&quot; -DPLATFORMIO=30502 -DARDUINO_ESP32_THING")
SET(CMAKE_CXX_STANDARD 11)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(SGP30_Example
        VERSION 1.0.0
        LANGUAGES C)