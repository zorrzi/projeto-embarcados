# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD pico_w)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(mqtt/pico_emb)

# initialize the Pico SDK
pico_sdk_init()

add_compile_options(
  -Wall -Wno-format # int != int32_t as far as the compiler is concerned because
                    # gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
)   

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-maybe-uninitialized)
endif()

add_subdirectory(freertos)
add_subdirectory(mqtt)

