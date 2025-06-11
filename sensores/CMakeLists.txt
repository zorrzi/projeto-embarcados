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
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Inicialização padrão Pico SDK
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

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

# Nome do projeto alterado aqui
project(PicoDock_TFT_Resistive_Example C CXX ASM)

pico_sdk_init()
add_subdirectory(freertos)


# === Arquivos do Projeto ===
add_executable(PicoDock_TFT_Resistive_Example 
    main.c

    # GFX / ILI9341 / TOUCH
    tft_lcd_ili9341/gfx/gfx.c
    tft_lcd_ili9341/ili9341/ili9341.c
    tft_lcd_ili9341/touch_resistive/touch_resistive.c
)

# Nome e versão do programa
pico_set_program_name(PicoDock_TFT_Resistive_Example "PicoDock_TFT_Resistive_Example")
pico_set_program_version(PicoDock_TFT_Resistive_Example "0.1")

# Saída padrão via UART
pico_enable_stdio_uart(PicoDock_TFT_Resistive_Example 1)
pico_enable_stdio_usb(PicoDock_TFT_Resistive_Example 0)

# Vincula as libs padrão + DMA + ADC
target_link_libraries(PicoDock_TFT_Resistive_Example
    pico_stdlib
    hardware_spi
    hardware_dma     # <- DMA
    hardware_adc     # <- ADC (novo)
    freertos
)

# Diretórios com headers
target_include_directories(PicoDock_TFT_Resistive_Example PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}

    # ILI9341 + Touch + GFX
    ${CMAKE_CURRENT_LIST_DIR}/gfx
    ${CMAKE_CURRENT_LIST_DIR}/ili9341
    ${CMAKE_CURRENT_LIST_DIR}/touch_resistive
)

# Gera os arquivos UF2 e binários
pico_add_extra_outputs(PicoDock_TFT_Resistive_Example)
