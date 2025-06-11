#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "pico/stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm
#include "hardware/sync.h" // wait for interrupt
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"  // adc
#include "hardware/gpio.h"

#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/timeouts.h"   
#include "lwip/inet.h"

#define WIFI_SSID                 "iPhone do Mateus (2)"
#define WIFI_PASSWORD             "matinheiro"
#define MQTT_SERVER_IP            "172.20.10.7"
#define MQTT_BROKER_PORT          1883

#define AUDIO_IN_PIN 27

#define SAMPLE_RATE 8000
#define DATA_LENGTH SAMPLE_RATE*2 // WAV_DATA_LENGTH //16000
#define FREQ 8000

#define AUDIO_PUBLISH_INTERVAL_MS 1000

#define TOPIC_AUDIO_DATA          "/audio_data"

#define MQTT_SUBSCRIBE_QOS        1
#define MQTT_PUBLISH_QOS          1
#define MQTT_PUBLISH_RETAIN       0

char audio[DATA_LENGTH];

int wav_position = 0;

SemaphoreHandle_t xSemaphoreRecordDone;
SemaphoreHandle_t xSemaphoreAudioReady;

typedef struct {
    mqtt_client_t                     *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    bool                               stop_client;
} MQTT_CLIENT_DATA_T;

const char * full_topic(MQTT_CLIENT_DATA_T *state, const char *suffix) {
    return suffix;
}

void pub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) printf("Publish callback erro = %d\n", err);
}

void sub_request_cb(void *arg, err_t err) {
    printf("Subscribe callback err = %d\n", err);
}

void unsub_request_cb(void *arg, err_t err) {
    printf("Unsubscribe callback err = %d\n", err);
}

static MQTT_CLIENT_DATA_T state = {0};
static volatile bool       mqtt_connected = false;
static absolute_time_t     next_publish_time;

static void wifi_connect(void);
static void mqtt_setup_connection(void);
static void sub_unsub_topics(MQTT_CLIENT_DATA_T *s, bool sub);
static void publish_audio_data(MQTT_CLIENT_DATA_T *s);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

bool timer_0_callback(repeating_timer_t* rt) {
    if (wav_position < DATA_LENGTH) {
        audio[wav_position++] = adc_read() / 16;
        return true; // keep repeating
    }
    else {
        xSemaphoreGiveFromISR(xSemaphoreRecordDone, 0);
        return false; // stop repeating
    }
}

void mic_task() {
    adc_gpio_init(AUDIO_IN_PIN);
    adc_init();
    adc_select_input(AUDIO_IN_PIN - 26);

    repeating_timer_t timer_0;

    while (1) {
        // ======== DETECÇÃO DE FALA ========
        printf("Aguardando detecção de voz...\n");
        int detectado = 0;
        
        while (!detectado) {
            int sum = 0;
            int values[50];
        
            for (int i = 0; i < 50; i++) {
                values[i] = adc_read();
                sum += values[i];
                sleep_us(100);
            }
        
            int mean = sum / 50;
        
            int energy = 0;
            for (int i = 0; i < 50; i++) {
                energy += abs(values[i] - mean);
            }
        
            energy /= 50;
            //printf("Energia do sinal: %d\n", energy);
        
            if (energy > 1100) { // Limiar ajustável
                detectado = 1;
                printf("Voz detectada!\n");
            }
        }
        

        // ======== INÍCIO DA GRAVAÇÃO ========
        wav_position = 0;
        if (!add_repeating_timer_us(1000000 / SAMPLE_RATE,
                                    timer_0_callback,
                                    NULL,
                                    &timer_0)) {
            printf("Erro ao iniciar timer\n");
        }

        if (xSemaphoreTake(xSemaphoreRecordDone, portMAX_DELAY) == pdTRUE) {
            cancel_repeating_timer(&timer_0);
        }

        // ======== FILTRO PASSA-BAIXAS DIGITAL ========
        // Média móvel de 3 pontos
        for (int i = 2; i < DATA_LENGTH; i++) {
            audio[i] = (audio[i] + audio[i - 1] + audio[i - 2]) / 3;
        }

        // ======== SINALIZAR ÁUDIO PRONTO ========
        xSemaphoreGive(xSemaphoreAudioReady);
    }
}

static void wifi_connect(void) {
    if (cyw43_arch_init()) panic("Wi-Fi init failed");
    cyw43_arch_enable_sta_mode();
    while (cyw43_arch_wifi_connect_timeout_ms(
           WIFI_SSID, WIFI_PASSWORD,
           CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) {
        printf("Tentando Wi-Fi...\n");
        sleep_ms(1000);
    }
    printf("✓ Wi-Fi OK, IP %s\n",
           ipaddr_ntoa(&cyw43_state.netif[0].ip_addr));
}

static void mqtt_setup_connection(void) {
    if (mqtt_connected) return;

    ip_addr_t broker_ip;
    if (!ipaddr_aton(MQTT_SERVER_IP, &broker_ip)) {
        printf("✗ IP broker inválido: %s\n", MQTT_SERVER_IP);
        return;
    }

    state.mqtt_client_info.client_id   = "pico_client";
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
    state.mqtt_client_info.keep_alive  = 0;

    cyw43_arch_lwip_begin();
    if (!state.mqtt_client_inst) {
        state.mqtt_client_inst = mqtt_client_new();
        if (!state.mqtt_client_inst) panic("mqtt_client_new failed");
    }
    mqtt_client_connect(
        state.mqtt_client_inst,
        &broker_ip,
        MQTT_BROKER_PORT,
        mqtt_connection_cb,
        &state,
        &state.mqtt_client_info
    );
    cyw43_arch_lwip_end();

    printf("→ MQTT connect solicitado\n");
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    printf("MQTT status = %d\n", status);
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_connected = true;
        printf("✓ Conexão MQTT OK, subscrevendo tópicos…\n");
        sub_unsub_topics(&state, true);

        cyw43_arch_lwip_begin();
        mqtt_set_inpub_callback(
            state.mqtt_client_inst,
            NULL,
            mqtt_incoming_data_cb,
            &state
        );
        cyw43_arch_lwip_end();
    } else {
        mqtt_connected = false;
        printf("✗ MQTT refused, retry em 5s\n");
        sleep_ms(5000);
        mqtt_setup_connection();
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T *s, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    cyw43_arch_lwip_begin();
    mqtt_sub_unsub(s->mqtt_client_inst, full_topic(s, "/print"),  MQTT_SUBSCRIBE_QOS, cb, s, sub);
    mqtt_sub_unsub(s->mqtt_client_inst, full_topic(s, "/ping"),   MQTT_SUBSCRIBE_QOS, cb, s, sub);
    mqtt_sub_unsub(s->mqtt_client_inst, full_topic(s, "/exit"),   MQTT_SUBSCRIBE_QOS, cb, s, sub);
    cyw43_arch_lwip_end();
    printf("→ %s tópicos\n", sub ? "Subscribed" : "Unsubscribed");
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    const char *msg = (const char*)data;
    printf("Mensagem em %.*s\n", len, msg);
}

static void publish_audio_data(MQTT_CLIENT_DATA_T *s) {
    // Exemplo de mensagem curta: um array de 1 byte ou uma string
    const char sinalzinho[] = "A";  // string de 1 caractere
    const u8_t *dados = (const u8_t *)sinalzinho;
    u16_t tamanho = sizeof(sinalzinho) - 1; // sem contar o '\0'

    const char *key = full_topic(s, TOPIC_AUDIO_DATA);
    printf("Publicando sinal de teste → %s\n", key);
    printf("Tamanho dos dados: %d bytes\n", tamanho);

    cyw43_arch_lwip_begin();
    err_t result = mqtt_publish(
        s->mqtt_client_inst,
        key,
        dados,           // ponteiro para o sinal curto
        tamanho,         // tamanho de 1 byte (neste caso 1)
        MQTT_PUBLISH_QOS,
        MQTT_PUBLISH_RETAIN,
        pub_request_cb,
        s
    );
    cyw43_arch_lwip_end();

    if (result == ERR_OK) {
        printf("✓ mqtt_publish (teste) retornou OK\n");
    } else {
        printf("✗ mqtt_publish (teste) falhou: %d\n", result);
    }
}


void mqtt_task() {
    wifi_connect();
    mqtt_setup_connection();

    next_publish_time = make_timeout_time_ms(AUDIO_PUBLISH_INTERVAL_MS);

    while (!state.stop_client) {
        cyw43_arch_lwip_begin();
        sys_check_timeouts();
        cyw43_arch_lwip_end();

        // Aguarda áudio estar pronto para envio
        if (xSemaphoreTake(xSemaphoreAudioReady, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (mqtt_connected) {
                publish_audio_data(&state);
            }
        }
        
        sleep_ms(10);
    }

    printf("Saindo… unsubscribing e desconectando\n");
    sub_unsub_topics(&state, false);
    mqtt_disconnect(state.mqtt_client_inst);
    cyw43_arch_deinit();
}

int main() {
    stdio_init_all();
    printf("oi\n");

    xSemaphoreRecordDone = xSemaphoreCreateBinary();
    xSemaphoreAudioReady = xSemaphoreCreateBinary();

    xTaskCreate(mic_task, "Mic Task", 4095, NULL, 1, NULL);
    xTaskCreate(mqtt_task, "MQTT Task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}