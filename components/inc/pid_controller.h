/**
 * @file pid_controller.h
 * @brief Controlador PID Discreto (Tustin com Filtro) + Realimentação de Velocidade da Roda
 *
 * Lei de Controle (mesma convenção do diagrama Simulink):
 *   e[k]     = r[k] - theta[k]
 *   u_pid[k] = u_p[k] + u_i[k] + u_d[k]
 *   u_raw[k] = u_pid[k] - (Kw * omega_r[k])
 *   u[k]     = saturate(u_raw[k], V_min, V_max)
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// PARÂMETROS PADRÃO (SINCRONIZADOS COM O SCRIPT MATLAB / SIMULINK)
// ============================================================================
#define PID_CTRL_TS_DEFAULT         0.001f    // Ts = 1 ms (1000 Hz)
#define PID_CTRL_VNOM_DEFAULT       12.0f     // Tensão nominal do motor (V)
#define PID_CTRL_VMAX_DEFAULT       12.0f     // Saturação superior (V)
#define PID_CTRL_VMIN_DEFAULT      -12.0f     // Saturação inferior (V)

// Ganhos sintonizados no MATLAB
#define PID_CTRL_KP_DEFAULT       -109.0f
#define PID_CTRL_KI_DEFAULT          0.0f
#define PID_CTRL_KD_DEFAULT        -12.0f
#define PID_CTRL_KW_DEFAULT         -0.47f
#define PID_CTRL_N_DEFAULT          100.0f     // Coeficiente do filtro derivativo

typedef struct {
    float kp;
    float ki;
    float kd;
    float kw;         /**< Ganho da realimentação paralela de velocidade da roda */
    float n_filter;   /**< Coeficiente N do filtro passa-baixas da derivada */
    float r;          /**< Referência de ângulo (radianos, padrão 0.0) */
    float v_nom;      /**< Tensão nominal para conversão PWM (Volts) */
    float v_max;      /**< Saturação máxima (Volts) */
    float v_min;      /**< Saturação mínima (Volts) */
    float ts;         /**< Período de amostragem (s) */
} pid_controller_config_t;

typedef struct {
    float u_raw;      /**< Tensão calculada antes da saturação (Volts) */
    float voltage;    /**< Tensão final aplicada após saturação (Volts) */
    int16_t duty;     /**< Duty cycle para o driver BTS7960 (-255 a +255) */
} pid_controller_output_t;

void pid_controller_init(void);
void pid_controller_init_custom(const pid_controller_config_t *cfg);
void pid_controller_reset(void);
void pid_controller_set_gains(float kp, float ki, float kd, float kw);

/**
 * @brief Executa um passo de amostragem do controlador PID + Kw.
 *
 * @param theta   Ângulo atual do pêndulo (rad)
 * @param omega_r Velocidade angular relativa da roda (rad/s)
 * @param out     Ponteiro opcional para diagnóstico (u_raw, voltage, duty)
 * @return int16_t Duty cycle (-255 a +255)
 */
int16_t pid_controller_update(float theta, float omega_r, pid_controller_output_t *out);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H_