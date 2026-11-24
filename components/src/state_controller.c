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

// Discretização ZOH do modelo em paramEncomendadoc.m para Ts = 1 ms.
static const state_space_model_t discrete_model = {
    .ad = {
        { 1.0000412089f,  0.0009998727f,  0.0000002317f },
        { 0.0824081770f,  0.9997591726f,  0.0004631878f },
        {-0.0822831421f,  0.0002404412f,  0.9965048795f },
    },
    .bd = {-0.0000010024f, -0.0020034734f, 0.0151178009f},
    .cd = {1.0f, 0.0f, 0.0f},
    .dd = 0.0f,
};

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
    if (!is_initialized) {
        state_controller_init();
    }

    ctrl_cfg.kd[0] = kd1;
    ctrl_cfg.kd[1] = kd2;
    ctrl_cfg.kd[2] = kd3;
    ESP_LOGI(TAG, "Novos Ganhos Kd = [%.4f, %.4f, %.4f]", kd1, kd2, kd3);
}

void state_controller_set_reference(float reference_rad)
{
    if (!is_initialized) {
        state_controller_init();
    }

    ctrl_cfg.r = reference_rad;
}

void state_controller_get_config(state_controller_config_t *out_cfg)
{
    if (out_cfg) {
        *out_cfg = ctrl_cfg;
    }
}

void state_controller_get_model(state_space_model_t *out_model)
{
    if (out_model != NULL) {
        *out_model = discrete_model;
    }
}

void state_controller_predict(const state_vector_t *x, float voltage,
                              state_vector_t *next_state, float *output)
{
    if (x == NULL) {
        return;
    }

    const float state[3] = {x->theta, x->theta_dot, x->omega_r};
    float next[3] = {0.0f, 0.0f, 0.0f};
    float y = discrete_model.dd * voltage;

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            next[row] += discrete_model.ad[row][col] * state[col];
        }
        next[row] += discrete_model.bd[row] * voltage;
        y += discrete_model.cd[row] * state[row];
    }

    if (next_state != NULL) {
        next_state->theta = next[0];
        next_state->theta_dot = next[1];
        next_state->omega_r = next[2];
    }
    if (output != NULL) {
        *output = y;
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
        state_controller_predict(x, u_sat, &out->predicted_state,
                                 &out->predicted_output);
    }

    return duty;
}

