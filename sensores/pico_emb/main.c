#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#include "tft_lcd_ili9341/ili9341/ili9341.h"
#include "tft_lcd_ili9341/gfx/gfx.h"

#define SCREEN_WIDTH 240
#define BACKLIGHT_PIN 15
#define LDR_GPIO 26
#define LDR_ADC_CHANNEL 0
#define R_LOAD 10000.0f // 10k Ohms

// Variáveis globais
float global_voltage = 0;
float global_current = 0;
float global_resistance = 0;
uint16_t global_adc_raw = 0;

void task_sensor(void *params) {
    adc_init();
    adc_gpio_init(LDR_GPIO);

    while (true) {
        adc_select_input(LDR_ADC_CHANNEL);

        uint16_t raw = adc_read();
        float voltage = raw * 3.3f / 4095.0f;
        float current = voltage / R_LOAD;
        float resistance = voltage > 0.0f ? (3.3f - voltage) * R_LOAD / voltage : 1e6; // Proteção contra divisão por zero

        global_adc_raw = raw;
        global_voltage = voltage;
        global_current = current;
        global_resistance = resistance;

        printf("LDR | ADC: %d | V=%.2f V | R=%.0f Ohm | I=%.2f mA\n",
               raw, voltage, resistance, current * 1000.0f);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_display(void *params) {
    LCD_initDisplay();
    LCD_setRotation(0);
    GFX_createFramebuf();

    gpio_init(BACKLIGHT_PIN);
    gpio_set_dir(BACKLIGHT_PIN, GPIO_OUT);
    gpio_put(BACKLIGHT_PIN, 1);

    while (true) {
        GFX_clearScreen();
        GFX_setCursor(10, 10);
        GFX_printf("LDR - Sensor de Luz\n");
        GFX_printf("ADC: %4d\n", global_adc_raw);
        GFX_printf("Tensao: %.2f V\n", global_voltage);
        GFX_printf("Resistencia: %.0f Ohm\n", global_resistance);
        GFX_printf("Corrente: %.2f mA\n", global_current * 1000.0f);

        GFX_flush();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void) {
    stdio_init_all();

    xTaskCreate(task_sensor, "Sensor", 512, NULL, 1, NULL);
    xTaskCreate(task_display, "Display", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}
