/**
 * @file motor.c
 * @brief Driver BTS7960 para ESP32 (ESP32-WROOM-32, ESP-IDF v6.0)
 *
 * Duas formas de gerar o PWM (escolha em MOTOR_USE_SOFT_PWM):
 *   0 -> PWM de hardware (periferico LEDC), 5 kHz, 8 bits.
 *   1 -> PWM VIRTUAL por software: um timer (GPTimer) gera interrupcoes em
 *        cada borda e liga/desliga os pinos direto nos registradores do GPIO.
 *        Nao usa o LEDC. Frequencia padrao 2 kHz.
 */

#include "motor.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

// ============================================================================
// CONFIGURAÇÃO DOS PINOS
// ============================================================================
#define GPIO_RPWM        25               // PWM Frente
#define GPIO_LPWM        26               // PWM Tras
#define GPIO_REN         32               // Enable Direita (3.3V)
#define GPIO_LEN         33               // Enable Esquerda (3.3V)

// ============================================================================
// ESCOLHA DO GERADOR DE PWM
// ============================================================================
// 1 -> PWM virtual (software + timer). 0 -> PWM de hardware (LEDC).
#define MOTOR_USE_SOFT_PWM   0

// ---------------- PWM de hardware (LEDC) ----------------
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R   LEDC_CHANNEL_0
#define LEDC_CHANNEL_L   LEDC_CHANNEL_1
#define LEDC_DUTY_RES    LEDC_TIMER_8_BIT // 0 a 255
#define LEDC_FREQUENCY   5000             // 5 kHz para garantir chaveamento estavel

// ---------------- PWM virtual (software) ----------------
#if MOTOR_USE_SOFT_PWM
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define SOFT_PWM_FREQ_HZ      2000                                  // frequencia do PWM
#define SOFT_PWM_RES_HZ       1000000                               // timer de 1 us
#define SOFT_PWM_PERIOD       (SOFT_PWM_RES_HZ / SOFT_PWM_FREQ_HZ)  // 500 ticks
// Pulsos (ou intervalos em nivel baixo) menores que isto nao sao gerados,
// porque a latencia da interrupcao os tornaria imprecisos.
#define SOFT_PWM_MIN_TICKS    8

#define MASK_R   (1UL << GPIO_RPWM)
#define MASK_L   (1UL << GPIO_LPWM)

static gptimer_handle_t pwm_timer = NULL;

// Comando empacotado em 32 bits para ser lido de forma atomica pela ISR:
// bits 0..15 = ticks em nivel alto; bit 16 = sentido (0 = RPWM, 1 = LPWM).
static volatile uint32_t pwm_cmd = 0;

static uint64_t period_start = 0;
static uint32_t active_mask = 0;
static bool pulse_high = false;

static bool IRAM_ATTR soft_pwm_isr(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx)
{
    (void)user_ctx;
    gptimer_alarm_config_t next = {
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };

    if (pulse_high) {
        // Fim do pulso: desce o pino ativo e espera o fim do periodo.
        REG_WRITE(GPIO_OUT_W1TC_REG, active_mask);
        pulse_high = false;
        next.alarm_count = period_start + SOFT_PWM_PERIOD;
    } else {
        // Inicio de um novo periodo: le o comando mais recente.
        period_start = edata->alarm_value;
        const uint32_t cmd = pwm_cmd;
        const uint32_t high = cmd & 0xFFFF;
        const uint32_t mask = (cmd & 0x10000) ? MASK_L : MASK_R;

        if (high < SOFT_PWM_MIN_TICKS) {
            // 0%: os dois pinos em nivel baixo (freio).
            REG_WRITE(GPIO_OUT_W1TC_REG, MASK_R | MASK_L);
            active_mask = 0;
            next.alarm_count = period_start + SOFT_PWM_PERIOD;
        } else if (high > SOFT_PWM_PERIOD - SOFT_PWM_MIN_TICKS) {
            // ~100%: pino ativo fica alto o periodo inteiro (sem pulso baixo).
            REG_WRITE(GPIO_OUT_W1TC_REG, (MASK_R | MASK_L) & ~mask);
            REG_WRITE(GPIO_OUT_W1TS_REG, mask);
            active_mask = mask;
            next.alarm_count = period_start + SOFT_PWM_PERIOD;
        } else {
            // Pulso normal: sobe agora e desce apos 'high' ticks.
            REG_WRITE(GPIO_OUT_W1TC_REG, (MASK_R | MASK_L) & ~mask);
            REG_WRITE(GPIO_OUT_W1TS_REG, mask);
            active_mask = mask;
            pulse_high = true;
            next.alarm_count = period_start + high;
        }
    }

    gptimer_set_alarm_action(timer, &next);
    return false;
}

static void soft_pwm_init(void)
{
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = SOFT_PWM_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &pwm_timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = soft_pwm_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(pwm_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(pwm_timer));

    gptimer_alarm_config_t first = {
        .alarm_count = 100,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(pwm_timer, &first));
    ESP_ERROR_CHECK(gptimer_start(pwm_timer));

    ESP_LOGI(TAG, "PWM VIRTUAL (software) ativo: %d Hz, periodo %d us.",
             SOFT_PWM_FREQ_HZ, SOFT_PWM_PERIOD);
}
#endif // MOTOR_USE_SOFT_PWM

void motor_init(void)
{
    ESP_LOGI(TAG, "Inicializando motor nos GPIOs: RPWM=%d, LPWM=%d, REN=%d, LEN=%d", 
             GPIO_RPWM, GPIO_LPWM, GPIO_REN, GPIO_LEN);

    // 1. Reseta os pinos de PWM, remove pull-ups/pull-downs internos e
    //    forca nivel baixo antes de qualquer coisa.
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

#if MOTOR_USE_SOFT_PWM
    // 3. PWM virtual: os pinos ficam como GPIO comum e o timer gera as bordas.
    soft_pwm_init();
#else
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
#endif

    // 6. So agora, com os dois PWM garantidamente em 0, habilita a ponte H.
    gpio_set_level(GPIO_REN, 1);
    gpio_set_level(GPIO_LEN, 1);

    ESP_LOGI(TAG, "Motor inicializado com sucesso.");
}

void motor_set_speed(int16_t speed)
{
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

#if MOTOR_USE_SOFT_PWM
    // Converte o duty (0..255) em ticks do periodo e publica para a ISR.
    const uint32_t mag = (uint32_t)(speed >= 0 ? speed : -speed);
    const uint32_t high = (mag * SOFT_PWM_PERIOD) / 255U;
    const uint32_t dir = (speed < 0) ? 0x10000U : 0U;
    pwm_cmd = dir | (high & 0xFFFFU);
#else
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
#endif
}
