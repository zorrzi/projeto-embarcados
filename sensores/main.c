#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

#include "tft_lcd_ili9341/ili9341/ili9341.h"
#include "tft_lcd_ili9341/gfx/gfx.h"

// Pinos do display
#define BACKLIGHT_PIN     15

// Pinos do SoftPot (ADC0 = GPIO26)
#define TOUCH_WIPER_PIN   26
#define TOUCH_ADC_CH      0

// Pinos do motor (4-wire stepper bipolar)
#define MOTOR_PIN1        2    // Coil A1
#define MOTOR_PIN2        3    // Coil A2
#define MOTOR_PIN3        4    // Coil B1
#define MOTOR_PIN4        5    // Coil B2

// Parâmetros do motor
#define STEPS_PER_REV     200   // 200 passos = 1 volta completa (full-step)
#define STEP_DELAY_MS       5   // Intervalo entre cada passo (em ms)

// Parâmetros do SoftPot / Touch
#define MA_LEN              20   // aumentado para maior filtragem
#define CALIB_SAMPLES      100   // mais amostras para calibração precisa
#define FAULT_MAX_STUCK    300   // erro só se ficar muito tempo sem variar (~15s)
#define HIGH_FAULT_LIMIT    50   // ciclos com leitura = 4095 (circuito aberto)
#define LOW_FAULT_LIMIT     50   // ciclos com leitura = 0 (curto/desconexão)
#define TOUCH_OFFSET       120   // sensibilidade do toque
#define NOISE_BAND         30    // ruído admissível
#define DEBOUNCE_COUNT     3     // amostras para debounce

// Parâmetros para economia de energia
#define IDLE_THRESHOLD     100   // ciclos vazios (≈100 × 50 ms = 5 s)
#define NORMAL_DELAY_MS    50    // atraso padrão entre leituras
#define LOW_POWER_DELAY_MS 500   // atraso em modo de economia

// Mantemos PID apenas para exibição; não influencia acionamento
#define Kp  1.0f
#define Ki  0.1f
#define Kd  0.05f

//--- Variáveis globais de calibração e detecção de falhas ---//
static uint16_t buffer_ma[MA_LEN];
static uint32_t sum_ma = 0;
static int idx_ma = 0;

// Valores de calibração
static uint16_t baseline_raw = 0;       // valor médio de “não toque”
static uint16_t touch_threshold = 0;    // limiar para “dor” (baseline_raw – TOUCH_OFFSET)

// Contadores de falha
static uint16_t last_raw_reading = 0;
static int identical_readings = 0;
static int high_fault_count = 0;
static int low_fault_count = 0;
static bool sensor_fault = false;

// Estado de “dor”
static bool showing_pain = false;
// Aguarda liberação do toque antes de novo disparo
static bool await_release = false;

// Debounce
static int touch_count = 0;
static int release_count = 0;

// Filtro passa-baixa extra
static uint16_t filtered_value = 0;
static float filter_alpha = 0.2f; // resposta mais rápida

// Contagem de ciclos sem atividade para economia de energia
static int idle_count = 0;
static bool power_saving = false;

// PID (apenas debug)
static float integral = 0.0f;
static float last_error = 0.0f;
static TickType_t last_time = 0;

// Textos para display
const char *texto_topo   = "Casper: O Fantasma Malvado";
const char *texto_padrao = "NAO MEXA! BONECO AMALDICOADO!";
const char *texto_ateu   = "EI! ISSO DOEU! REGENERANDO!";
const char *texto_falha  = "Erro no sensor!";

//------------------- Rotação do motor -------------------//
void rotate_head_once() {
    int i = 0;
    while (i < 512) {
        gpio_put(MOTOR_PIN1, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_put(MOTOR_PIN1, 0);

        gpio_put(MOTOR_PIN2, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_put(MOTOR_PIN2, 0);

        gpio_put(MOTOR_PIN3, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_put(MOTOR_PIN3, 0);

        gpio_put(MOTOR_PIN4, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_put(MOTOR_PIN4, 0);

        i++;
    }
}

//----------------- Calibração do SoftPot -----------------//
static void calibrate_touch_sensor(void) {
    uint32_t sum = 0;
    uint16_t sample;
    uint16_t max_sample = 0;
    uint16_t min_sample = 4095;

    printf("Iniciando calibracao do sensor...\n");
    printf("NAO TOQUE NO SENSOR DURANTE A CALIBRACAO!\n");

    // Descartar primeiras leituras para estabilização
    for (int i = 0; i < 20; i++) {
        adc_select_input(TOUCH_ADC_CH);
        adc_read();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Coleta CALIB_SAMPLES amostras para definir baseline
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        adc_select_input(TOUCH_ADC_CH);
        sample = adc_read();
        sum += sample;
        if (sample > max_sample) max_sample = sample;
        if (sample < min_sample) min_sample = sample;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    baseline_raw = sum / CALIB_SAMPLES;

    // Ruído observado
    uint16_t noise_range = max_sample - min_sample;
    printf("Ruido observado: %d (min: %d, max: %d)\n", noise_range, min_sample, max_sample);

    // Define limiar abaixo do baseline
    if (baseline_raw > (TOUCH_OFFSET + noise_range * 2)) {
        touch_threshold = baseline_raw - TOUCH_OFFSET;
    } else {
        touch_threshold = 100; // mínimo de segurança
    }
    printf("Calibracao completa - Baseline: %d, Threshold: %d\n", baseline_raw, touch_threshold);

    // Inicializa média móvel com o baseline
    for (int i = 0; i < MA_LEN; i++) {
        buffer_ma[i] = baseline_raw;
    }
    sum_ma = (uint32_t)baseline_raw * MA_LEN;
    idx_ma = 0;

    // Valor filtrado inicial
    filtered_value = baseline_raw;

    // Zera contadores de falha
    identical_readings = 0;
    high_fault_count = 0;
    low_fault_count = 0;
    sensor_fault = false;
    idle_count = 0;
    power_saving = false;
}

//----------------- Tarefa de Leitura & Exibição -----------------//
void task_display(void *params) {
    // Inicializa display
    LCD_initDisplay();
    LCD_setRotation(0);
    GFX_createFramebuf();

    // Liga backlight
    gpio_init(BACKLIGHT_PIN);
    gpio_set_dir(BACKLIGHT_PIN, GPIO_OUT);
    gpio_put(BACKLIGHT_PIN, 1);

    // Configura pinos do motor como saída
    gpio_init(MOTOR_PIN1); gpio_set_dir(MOTOR_PIN1, GPIO_OUT); gpio_put(MOTOR_PIN1, 0);
    gpio_init(MOTOR_PIN2); gpio_set_dir(MOTOR_PIN2, GPIO_OUT); gpio_put(MOTOR_PIN2, 0);
    gpio_init(MOTOR_PIN3); gpio_set_dir(MOTOR_PIN3, GPIO_OUT); gpio_put(MOTOR_PIN3, 0);
    gpio_init(MOTOR_PIN4); gpio_set_dir(MOTOR_PIN4, GPIO_OUT); gpio_put(MOTOR_PIN4, 0);

    // Inicializa ADC
    adc_init();
    adc_gpio_init(TOUCH_WIPER_PIN);
    adc_select_input(TOUCH_ADC_CH);

    // Tempo para estabilizar ADC
    vTaskDelay(pdMS_TO_TICKS(100));

    // Calibração inicial
    calibrate_touch_sensor();

    // Inicia PID
    last_time = xTaskGetTickCount();
    integral = 0.0f;
    last_error = 0.0f;

    // Atraso dinâmico baseado em modo (normal ou economia)
    int current_delay_ms = NORMAL_DELAY_MS;
    TickType_t last_wakeup = xTaskGetTickCount();

    for (;;) {
        // Seleciona canal e lê ADC
        adc_select_input(TOUCH_ADC_CH);
        uint16_t raw = adc_read();

        // ---------- DETECÇÃO DE FALHAS ---------- //

        // 1) Leitura parada no mesmo valor
        if (raw == last_raw_reading) {
            identical_readings++;
            if (identical_readings > FAULT_MAX_STUCK) {
                sensor_fault = true;
                printf("ERRO: Sensor travado no valor %d\n", raw);
            }
        } else {
            identical_readings = 0;
        }
        last_raw_reading = raw;

        // 2) Leitura = 4095 contínua → possível circuito aberto
        if (raw >= 4095) {
            high_fault_count++;
            if (high_fault_count > HIGH_FAULT_LIMIT) {
                sensor_fault = true;
                printf("ERRO: Leitura MAXIMA continua (%d ciclos)\n", high_fault_count);
            }
        } else {
            high_fault_count = 0;
        }

        // 3) Leitura = 0 contínua → possível curto ou desconectado
        if (raw == 0) {
            low_fault_count++;
            if (low_fault_count > LOW_FAULT_LIMIT) {
                sensor_fault = true;
                printf("ERRO: Leitura MINIMA continua (%d ciclos)\n", low_fault_count);
            }
        } else {
            low_fault_count = 0;
        }

        // Se falha, exibe mensagem e volta ao laço após delay
        if (sensor_fault) {
            GFX_clearScreen();
            GFX_setCursor(40, 120);
            GFX_printf("%s\n", texto_falha);
            GFX_flush();
            vTaskDelay(pdMS_TO_TICKS(current_delay_ms));
            continue;
        }

        // ---------- FILTRAGEM ---------- //

        // Média móvel
        sum_ma -= buffer_ma[idx_ma];
        buffer_ma[idx_ma] = raw;
        sum_ma += buffer_ma[idx_ma];
        idx_ma = (idx_ma + 1) % MA_LEN;
        uint16_t avg_raw = sum_ma / MA_LEN;

        // Filtro passa-baixa extra
        filtered_value = (uint16_t)((filter_alpha * avg_raw) + ((1.0f - filter_alpha) * filtered_value));
        uint16_t stable_value = filtered_value;

        // ---------- PID (debug) ---------- //
        TickType_t now = xTaskGetTickCount();
        float delta_t = (now - last_time) * (1.0f / configTICK_RATE_HZ);
        last_time = now;
        float error = (float)baseline_raw - (float)stable_value;
        float P = Kp * error;
        integral += error * delta_t;
        float I = Ki * integral;
        float derivative = (delta_t > 0.0f) ? ((error - last_error) / delta_t) : 0.0f;
        float D = Kd * derivative;
        last_error = error;
        float output = P + I + D;
        if (output > 100.0f) output = 100.0f;
        if (output < -100.0f) output = -100.0f;

        // ---------- GESTÃO DO MODO DE ECONOMIA ---------- //
        if (!power_saving) {
            // Se estável dentro do ruído por muitos ciclos → entra em economia
            if (abs((int)stable_value - (int)baseline_raw) <= NOISE_BAND) {
                idle_count++;
                if (idle_count >= IDLE_THRESHOLD) {
                    power_saving = true;
                    current_delay_ms = LOW_POWER_DELAY_MS;
                    // Opcional: desligar backlight para poupar energia
                    gpio_put(BACKLIGHT_PIN, 0);
                    printf("Entrando em modo de economia de energia.\n");
                }
            } else {
                idle_count = 0;
            }
        } else {
            // Se em economia, aguarda toque para sair
            if (stable_value < touch_threshold) {
                power_saving = false;
                idle_count = 0;
                current_delay_ms = NORMAL_DELAY_MS;
                // Religa backlight
                gpio_put(BACKLIGHT_PIN, 1);
                printf("Saindo do modo de economia de energia.\n");
            }
        }

        // ---------- DETECÇÃO DE TOQUE (debounce) ---------- //
        bool touch_detected = false;

        // Só detecta toque se não estiver aguardando liberação
        if (!await_release) {
            if (stable_value < touch_threshold &&
                abs((int)stable_value - (int)baseline_raw) > NOISE_BAND) {
                touch_count++;
                release_count = 0;
                if (touch_count >= DEBOUNCE_COUNT) {
                    touch_detected = true;
                    touch_count = DEBOUNCE_COUNT;
                }
            } else {
                release_count++;
                if (release_count >= DEBOUNCE_COUNT) {
                    touch_count = 0;
                    release_count = DEBOUNCE_COUNT;
                }
            }

            if (touch_detected && !showing_pain) {
                printf("TOQUE CONFIRMADO! Valor: %d (thresh: %d, diff: %d)\n",
                       stable_value, touch_threshold, baseline_raw - stable_value);

                // Ativar “dor” e atualizar display
                showing_pain = true;
                GFX_clearScreen();
                GFX_setCursor(40, 10);
                GFX_printf("%s\n", texto_topo);
                GFX_setCursor(40, 120);
                GFX_printf("%s\n", texto_ateu);
                GFX_flush();
                vTaskDelay(pdMS_TO_TICKS(50));  // garante que texto apareça

                // Gira a cabeça
                rotate_head_once();
                printf("Fim do estado de dor.\n");

                // Volta ao estado normal
                showing_pain = false;
                touch_count = 0;
                release_count = 0;
                await_release = true;
            }
        } else {
            // Aguarda liberação do toque (valor acima do threshold)
            if (stable_value >= touch_threshold) {
                await_release = false;
                touch_count = 0;
                release_count = 0;
            }
        }

        // ---------- ATUALIZAÇÃO DO DISPLAY ---------- //
        if (!power_saving) {
            GFX_clearScreen();

            // Título
            GFX_setCursor(40, 10);
            GFX_printf("%s\n", texto_topo);

            // Texto central
            const char *texto_centro = showing_pain ? texto_ateu : texto_padrao;
            GFX_setCursor(40, 120);
            GFX_printf("%s\n", texto_centro);

            GFX_flush();
        }
        // Se estiver em power_saving, pula atualização para economizar energia

        // ---------- AGUARDA PRÓXIMA LEITURA ---------- //
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(current_delay_ms));
    }
}

//----------------- Função main -----------------//
int main(void) {
    stdio_init_all();

    // Espera inicialização do USB serial
    sleep_ms(2000);
    printf("Casper: O Boneco Malvado - Iniciando...\n");

    // Cria tarefa de leitura/exibição
    xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);

    // Inicia FreeRTOS
    vTaskStartScheduler();

    // Nunca chega aqui
    while (true) {
        tight_loop_contents();
    }
    return 0;
}
