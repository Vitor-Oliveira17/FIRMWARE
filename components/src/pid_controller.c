/**
 * @file pid_controller.c
 * @brief Implementação Discreta do PID em Paralelo com Realimentação de Velocidade
 */

#include "pid_controller.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "PID_CTRL";

static pid_controller_config_t ctrl_cfg;
static bool is_initialized = false;

// Estados internos equivalentes às variáveis "persistent" do MATLAB
static float ek_1  = 0.0f;
static float uik_1 = 0.0f;
static float udk_1 = 0.0f;

static inline float saturate(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

void pid_controller_reset(void)
{
    ek_1  = 0.0f;
    uik_1 = 0.0f;
    udk_1 = 0.0f;
}

void pid_controller_init(void)
{
    pid_controller_config_t default_cfg = {
        .kp       = PID_CTRL_KP_DEFAULT,
        .ki       = PID_CTRL_KI_DEFAULT,
        .kd       = PID_CTRL_KD_DEFAULT,
        .kw       = PID_CTRL_KW_DEFAULT,
        .n_filter = PID_CTRL_N_DEFAULT,
        .r        = 0.0f,
        .v_fonte  = PID_CTRL_VFONTE_DEFAULT,
        .v_max    = PID_CTRL_VMAX_DEFAULT,
        .v_min    = PID_CTRL_VMIN_DEFAULT,
        .ts       = PID_CTRL_TS_DEFAULT,
    };

    pid_controller_init_custom(&default_cfg);
}

void pid_controller_init_custom(const pid_controller_config_t *cfg)
{
    if (cfg == NULL) {
        pid_controller_init();
        return;
    }

    ctrl_cfg = *cfg;
    pid_controller_reset();
    is_initialized = true;

    ESP_LOGI(TAG, "PID Inicializado:");
    ESP_LOGI(TAG, "  Ts: %.4f s | Vfonte: %.1f V | Limites: [%.1f V, %.1f V]",
             ctrl_cfg.ts, ctrl_cfg.v_fonte, ctrl_cfg.v_min, ctrl_cfg.v_max);
    ESP_LOGI(TAG, " Kp=%.2f Ki=%.2f Kd=%.2f Kw=%.3f N=%.1f",
            ctrl_cfg.kp, ctrl_cfg.ki, ctrl_cfg.kd, ctrl_cfg.kw, ctrl_cfg.n_filter);
}

void pid_controller_set_gains(float kp, float ki, float kd, float kw)
{
    if (!is_initialized) {
        pid_controller_init();
    }
    ctrl_cfg.kp = kp;
    ctrl_cfg.ki = ki;
    ctrl_cfg.kd = kd;
    ctrl_cfg.kw = kw;
    pid_controller_reset();
}

int16_t pid_controller_update(float theta, float omega_r, pid_controller_output_t *out)
{
    if (!is_initialized) {
        pid_controller_init();
    }

    const float ts = ctrl_cfg.ts;
    const float n  = ctrl_cfg.n_filter;

    // 1. Erro angular: e[k] = r[k] - theta[k]
    float ek = ctrl_cfg.r - theta;

    // 2. Termo Proporcional
    float upk = ctrl_cfg.kp * ek;

    // 3. Termo Integral (Tustin/Trapezoidal: Ki * Ts / 2)
    float uik = uik_1 + (ctrl_cfg.ki * ts * 0.5f) * (ek + ek_1);

    // 4. Termo Derivativo com Filtro Passa-Baixas N (Tustin/Trapezoidal)
    // Quando N -> inf, reduz-se exatamente a: (2*Kd/Ts)*(ek - ek_1) - udk_1
    float denom = 2.0f + (n * ts);
    float alpha_d = (2.0f - (n * ts)) / denom;
    float beta_d  = (2.0f * ctrl_cfg.kd * n) / denom;
    float udk = (alpha_d * udk_1) + (beta_d * (ek - ek_1));

    // 5. Soma do PID + Realimentação paralela da velocidade da roda (somador "+-")
    float u_pid = upk + uik + udk;
    float u_raw = u_pid - (ctrl_cfg.kw * omega_r);

    // 6. Saturação de tensão (+-12V)
    float u_sat = saturate(u_raw, ctrl_cfg.v_min, ctrl_cfg.v_max);

    // Anti-windup (Clamping): congela o integrador se houver saturação e Ki != 0
    if (u_raw != u_sat && ctrl_cfg.ki != 0.0f) {
        uik = uik_1;
    }

    // 7. Atualiza histórico para a próxima amostra (k -> k+1)
    ek_1  = ek;
    uik_1 = uik;
    udk_1 = udk;

    // 8. Conversão de Tensão (V) para Duty Cycle (-255 a +255 do BTS7960)
    float duty_f = 0.0f;
    if (ctrl_cfg.v_fonte > 0.0f) {
        duty_f = (u_sat / ctrl_cfg.v_fonte) * 255.0f;
    }
    int16_t duty = (int16_t)roundf(saturate(duty_f, -255.0f, 255.0f));

    if (out != NULL) {
        out->u_raw   = u_raw;
        out->voltage = u_sat;
        out->duty    = duty;
    }

    return duty;
}