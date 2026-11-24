/**
 * @file motor.c
 * @brief Driver BTS7960 otimizado para ESP32-C6 (ESP-IDF v6.0)
 */

#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

// ============================================================================
// CONFIGURAÇÃO DOS PINOS
// ============================================================================
#define GPIO_RPWM        12               // PWM Frente
#define GPIO_LPWM        13               // PWM Trás
#define GPIO_REN          5               // Enable Direita (3.3V)
#define GPIO_LEN          7               // Enable Esquerda (3.3V)

#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R   LEDC_CHANNEL_0
#define LEDC_CHANNEL_L   LEDC_CHANNEL_1
#define LEDC_DUTY_RES    LEDC_TIMER_8_BIT // 0 a 255
#define LEDC_FREQUENCY   5000             // 5 kHz para garantir chaveamento estavel

void motor_init(void)
{
    ESP_LOGI(TAG, "Inicializando motor nos GPIOs: RPWM=%d, LPWM=%d, REN=%d, LEN=%d", 
             GPIO_RPWM, GPIO_LPWM, GPIO_REN, GPIO_LEN);

    // 1. GPIO12/13 sao USB D-/D+ no ESP32-C6: retira-os da funcao USB,
    //    remove pull-ups internos e forca nivel baixo antes de qualquer coisa.
    const gpio_num_t pwm_pins[] = {GPIO_RPWM, GPIO_LPWM};
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pwm_pins[i]);
        gpio_set_direction(pwm_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(pwm_pins[i], GPIO_FLOATING);
        gpio_set_level(pwm_pins[i], 0);
    }

    // 2. Enables como saida, inicialmente DESLIGADOS (ponte em alta impedancia)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_REN) | (1ULL << GPIO_LEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GPIO_REN, 0);
    gpio_set_level(GPIO_LEN, 0);

    // 3. Configura o Timer do LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 4. Configura Canal RPWM
    ledc_channel_config_t chan_r = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_R,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = GPIO_RPWM,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_r));

    // 5. Configura Canal LPWM
    ledc_channel_config_t chan_l = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_L,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = GPIO_LPWM,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_l));

    // 6. So agora, com os dois PWM garantidamente em 0, habilita a ponte H.
    gpio_set_level(GPIO_REN, 1);
    gpio_set_level(GPIO_LEN, 1);

    ESP_LOGI(TAG, "Motor inicializado com sucesso.");
}

void motor_set_speed(int16_t speed)
{
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        // Sentido 1: RPWM recebe o Duty, LPWM fica zerado
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, (uint32_t)speed));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_L, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_L));
    } 
    else if (speed < 0) {
        // Sentido 2: LPWM recebe o Duty, RPWM fica zerado
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_L, (uint32_t)(-speed)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_L));
    } 
    else {
        // Freio: Ambos zerados
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_L, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_L));
    }
}