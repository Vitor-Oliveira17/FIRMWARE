/**
 * @file state_controller.c
 * @brief Implementação do Controlador em Espaço de Estados Discreto
 */

#include "state_controller.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "STATE_CTRL";

static state_controller_config_t ctrl_cfg;
static bool is_initialized = false;

static inline float saturate(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

void state_controller_init(void)
{
    state_controller_config_t default_cfg = {
        .kd = { STATE_CTRL_KD1_DEFAULT, STATE_CTRL_KD2_DEFAULT, STATE_CTRL_KD3_DEFAULT },
        .r = 0.0f,
        .v_nom = STATE_CTRL_VNOM_DEFAULT,
        .v_max = STATE_CTRL_VMAX_DEFAULT,
        .v_min = STATE_CTRL_VMIN_DEFAULT,
        .ts = STATE_CTRL_TS_DEFAULT,
    };

    state_controller_init_custom(&default_cfg);
}

void state_controller_init_custom(const state_controller_config_t *cfg)
{
    if (cfg == NULL) {
        state_controller_init();
        return;
    }

    ctrl_cfg = *cfg;
    is_initialized = true;

    ESP_LOGI(TAG, "Controlador Inicializado:");
    ESP_LOGI(TAG, "  Ts: %.4f s | Vnom: %.1f V | Limites: [%.1f V, %.1f V]",
             ctrl_cfg.ts, ctrl_cfg.v_nom, ctrl_cfg.v_min, ctrl_cfg.v_max);
    ESP_LOGI(TAG, "  Kd = [%.4f, %.4f, %.4f]",
             ctrl_cfg.kd[0], ctrl_cfg.kd[1], ctrl_cfg.kd[2]);
    ESP_LOGI(TAG, "  Referencia r: %.4f rad", ctrl_cfg.r);
}

void state_controller_set_gains(float kd1, float kd2, float kd3)
{
    ctrl_cfg.kd[0] = kd1;
    ctrl_cfg.kd[1] = kd2;
    ctrl_cfg.kd[2] = kd3;
    ESP_LOGI(TAG, "Novos Ganhos Kd = [%.4f, %.4f, %.4f]", kd1, kd2, kd3);
}

void state_controller_set_reference(float reference_rad)
{
    ctrl_cfg.r = reference_rad;
}

void state_controller_get_config(state_controller_config_t *out_cfg)
{
    if (out_cfg) {
        *out_cfg = ctrl_cfg;
    }
}

int16_t state_controller_update(const state_vector_t *x, state_controller_output_t *out)
{
    if (!is_initialized) {
        state_controller_init();
    }

    if (x == NULL) {
        return 0;
    }

    // 1. Termo de realimentação de estados: Kd * x[k]
    // Conforme o diagrama do Simulink:
    // u_raw = r[k] - (Kd1 * x1 + Kd2 * x2 + Kd3 * x3)
    float feedback = (ctrl_cfg.kd[0] * x->theta) +
                     (ctrl_cfg.kd[1] * x->theta_dot) +
                     (ctrl_cfg.kd[2] * x->omega_r);

    float u_raw = ctrl_cfg.r - feedback;

    // 2. Bloco de Saturação de Tensão (Tensão máxima suportada pelo motor / bateria)
    float u_sat = saturate(u_raw, ctrl_cfg.v_min, ctrl_cfg.v_max);

    // 3. Mapeamento de Tensão (V) para Duty Cycle (-255 a +255 do BTS7960)
    float duty_f = 0.0f;
    if (ctrl_cfg.v_nom > 0.0f) {
        duty_f = (u_sat / ctrl_cfg.v_nom) * 255.0f;
    }

    int16_t duty = (int16_t)roundf(saturate(duty_f, -255.0f, 255.0f));

    // 4. Preenche as saídas caso o ponteiro tenha sido fornecido
    if (out != NULL) {
        out->u_raw = u_raw;
        out->voltage = u_sat;
        out->duty = duty;
    }

    return duty;
}

