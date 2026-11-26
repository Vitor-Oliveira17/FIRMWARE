/**
 * @file pwm_test.c
 * @brief Teste ISOLADO do PWM da ponte H BTS7960 no ESP32-C6.
 *
 * Nao usa IMU, encoder nem controlador: apenas GPIO + LEDC.
 *
 * Sequencia (repetida indefinidamente):
 *   Fase 1 - Niveis estaticos (para medir com multimetro):
 *       a) RPWM=0  LPWM=0  -> motor deve ficar PARADO
 *       b) RPWM=1  LPWM=0  -> gira sentido 1, maxima
 *       c) RPWM=0  LPWM=1  -> gira sentido 2, maxima
 *   Fase 2 - Rampa de PWM no RPWM (25/50/75/100%)
 *   Fase 3 - Rampa de PWM no LPWM (25/50/75/100%)
 *
 * Tensao media esperada no pino (multimetro DC): ~3.3 V * duty.
 *
 * ATENCAO: roda suspensa e area livre durante o teste.
 */

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PWM_TEST";

// ============================================================================
// CONFIGURACAO (ajuste aqui conforme a ligacao)
// ============================================================================
#define PIN_RPWM            23      // RPWM do BTS7960
#define PIN_LPWM            20      // LPWM do BTS7960
#define PIN_REN             5       // R_EN do BTS7960
#define PIN_LEN             7       // L_EN do BTS7960

#define PWM_FREQ_HZ         5000
#define PWM_RES             LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY        (1U << 8)   // 256 = nivel alto constante (100%)

#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_TIMER           LEDC_TIMER_0
#define CH_R                LEDC_CHANNEL_0
#define CH_L                LEDC_CHANNEL_1

#define STEP_MS             3000
#define PAUSE_MS            1500

// ============================================================================

static void pwm_set_percent(ledc_channel_t ch, uint32_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    const uint32_t duty = (percent * PWM_MAX_DUTY) / 100U;
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, ch));
}

static void bridge_set(uint32_t r_percent, uint32_t l_percent)
{
    // Sempre zera primeiro o lado que vai desligar, evitando os dois ativos.
    if (r_percent == 0) {
        pwm_set_percent(CH_R, 0);
        pwm_set_percent(CH_L, l_percent);
    } else {
        pwm_set_percent(CH_L, 0);
        pwm_set_percent(CH_R, r_percent);
    }
}

static void stop_and_wait(void)
{
    bridge_set(0, 0);
    ESP_LOGI(TAG, "   parado (RPWM=0, LPWM=0) por %d ms", PAUSE_MS);
    vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
}

static void hw_init(void)
{
    // Reseta os pinos e remove pull-ups/pull-downs internos.
    const int pwm_pins[] = {PIN_RPWM, PIN_LPWM};
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pwm_pins[i]);
        gpio_set_direction(pwm_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(pwm_pins[i], GPIO_FLOATING);
        gpio_set_level(pwm_pins[i], 0);
    }

    // Enables
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << PIN_REN) | (1ULL << PIN_LEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en_conf));
    gpio_set_level(PIN_REN, 0);
    gpio_set_level(PIN_LEN, 0);

    // Timer LEDC
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Canais LEDC
    ledc_channel_config_t ch_r = {
        .gpio_num = PIN_RPWM,
        .speed_mode = PWM_MODE,
        .channel = CH_R,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_r));

    ledc_channel_config_t ch_l = {
        .gpio_num = PIN_LPWM,
        .speed_mode = PWM_MODE,
        .channel = CH_L,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_l));

    // So habilita a ponte depois que os dois PWM estao garantidamente em 0.
    gpio_set_level(PIN_REN, 1);
    gpio_set_level(PIN_LEN, 1);
}

void app_main(void)
{
    ESP_LOGW(TAG, "=== TESTE ISOLADO DE PWM (BTS7960) ===");
    ESP_LOGW(TAG, "RPWM=GPIO%d  LPWM=GPIO%d  R_EN=GPIO%d  L_EN=GPIO%d  f=%d Hz",
             PIN_RPWM, PIN_LPWM, PIN_REN, PIN_LEN, PWM_FREQ_HZ);
    ESP_LOGW(TAG, "Mantenha a roda suspensa!");

    hw_init();
    vTaskDelay(pdMS_TO_TICKS(2000));

    static const uint32_t ramp[] = {25, 50, 75, 100};
    uint32_t cycle = 0;

    while (true) {
        ESP_LOGW(TAG, "----- Ciclo %lu -----", (unsigned long)++cycle);

        // ---------------- Fase 1: niveis estaticos ----------------
        ESP_LOGI(TAG, "[F1a] RPWM=0 LPWM=0 -> motor deve estar PARADO (%d ms)", STEP_MS);
        bridge_set(0, 0);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));

        ESP_LOGI(TAG, "[F1b] RPWM=1 LPWM=0 -> sentido 1, maximo (%d ms)", STEP_MS);
        bridge_set(100, 0);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        stop_and_wait();

        ESP_LOGI(TAG, "[F1c] RPWM=0 LPWM=1 -> sentido 2, maximo (%d ms)", STEP_MS);
        bridge_set(0, 100);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        stop_and_wait();

        // ---------------- Fase 2: rampa RPWM ----------------
        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "[F2] RPWM=%lu%%  (esperado ~%.2f V no GPIO%d)",
                     (unsigned long)ramp[i], 3.3f * ramp[i] / 100.0f, PIN_RPWM);
            bridge_set(ramp[i], 0);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        stop_and_wait();

        // ---------------- Fase 3: rampa LPWM ----------------
        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "[F3] LPWM=%lu%%  (esperado ~%.2f V no GPIO%d)",
                     (unsigned long)ramp[i], 3.3f * ramp[i] / 100.0f, PIN_LPWM);
            bridge_set(0, ramp[i]);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        stop_and_wait();
    }
}
