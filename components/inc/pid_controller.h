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
// PARÂMETROS PADRÃO (SINCRONIZADOS COM STATE_CONTROLLER / MATLAB / SIMULINK)
// ============================================================================
#define PID_CTRL_TS_DEFAULT         0.001f    // Ts = 1 ms (1000 Hz)
#define PID_CTRL_VNOM_DEFAULT       12.0f     // Tensão nominal do motor do modelo (V)
#define PID_CTRL_VMAX_DEFAULT       12.0f     // Saturação superior (V)
#define PID_CTRL_VMIN_DEFAULT      -12.0f     // Saturação inferior (V)

// Ganhos sintonizados no MATLAB
#define PID_CTRL_KP_DEFAULT       -109.0f
#define PID_CTRL_KI_DEFAULT          0.0f
#define PID_CTRL_KD_DEFAULT        -12.0f
#define PID_CTRL_KW_DEFAULT         -0.47f
#define PID_CTRL_N_DEFAULT         100.0f     // Coeficiente do filtro derivativo

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

/**
 * @brief Configurações do controlador PID + realimentação de velocidade
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float kw;         /**< Ganho da realimentação paralela de velocidade da roda */
    float n_filter;   /**< Coeficiente N do filtro passa-baixas da derivada */
    float r;          /**< Referência de ângulo do pêndulo r[k] (radianos, padrão 0.0) */
    float v_nom;      /**< Tensão nominal para conversão PWM (Volts) */
    float v_max;      /**< Saturação máxima de tensão (Volts) */
    float v_min;      /**< Saturação mínima de tensão (Volts) */
    float ts;         /**< Período de amostragem em segundos */
} pid_controller_config_t;

/**
 * @brief Dados de saída do controlador a cada período de amostragem
 */
typedef struct {
    float u_raw;      /**< Tensão calculada antes da saturação (Volts) */
    float voltage;    /**< Tensão final aplicada após saturação (Volts) */
    int16_t duty;     /**< Duty cycle correspondente para a ponte H (-255 a +255) */
    pid_state_vector_t predicted_state; /**< Estado previsto para k+1 pelo modelo discreto */
    float predicted_output; /**< Saída y[k] = Cd*x[k] + Dd*u[k] (ângulo theta) */
} pid_controller_output_t;

//Predição da velocidade angular
/**
 * @brief Vetor de estados do sistema x[k]
 */
typedef struct {
    float theta;      /**< x1: Ângulo de inclinação do pêndulo (radianos) */
    float theta_dot;  /**< x2: Velocidade angular do pêndulo (rad/s) */
    float omega_r;    /**< x3: Velocidade angular da roda de reação (rad/s) */
} pid_state_vector_t;

/**
 * @brief Modelo discreto x[k+1] = Ad*x[k] + Bd*u[k], y[k] = Cd*x[k] + Dd*u[k].
 */
typedef struct {
    float ad[3][3];
    float bd[3];
    float cd[3];
    float dd;
} pid_space_model_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o controlador com as configurações padrão do projeto.
 */
void pid_controller_init(void);

/**
 * @brief Inicializa o controlador com configurações customizadas.
 *
 * @param cfg Ponteiro para a estrutura com os parâmetros customizados.
 */
void pid_controller_init_custom(const pid_controller_config_t *cfg);

/**
 * @brief Zera os estados internos do integrador e do filtro derivativo.
 */
void pid_controller_reset(void);

/**
 * @brief Atualiza os ganhos do controlador em tempo de execução.
 */
void pid_controller_set_gains(float kp, float ki, float kd, float kw);

/**
 * @brief Define a referência de ângulo do pêndulo (em radianos).
 *
 * @param reference_rad Referência desejada (normalmente 0 rad para vertical).
 */
void pid_controller_set_reference(float reference_rad);

/**
 * @brief Obtém a configuração ativa no controlador.
 */
void pid_controller_get_config(pid_controller_config_t *out_cfg);

/**
 * @brief Obtém as matrizes discretas calculadas no script MATLAB.
 */
void pid_controller_get_model(pid_space_model_t *out_model);

/**
 * @brief Propaga a planta discreta uma amostra usando a tensão de entrada.
 */
void pid_controller_predict(const pid_state_vector_t *x, float voltage,
                            pid_state_vector_t *next_state, float *output);

/**
 * @brief Executa um passo de amostragem do controlador PID + Kw.
 *
 * @param theta   Ângulo atual do pêndulo (rad)
 * @param omega_r Velocidade angular relativa da roda (rad/s)
 * @param out     Ponteiro opcional para diagnóstico (u_raw, voltage, duty)
 * @return int16_t Duty cycle pronto para envio direto à função motor_set_speed().
 */
int16_t pid_controller_update(float theta, float omega_r, pid_controller_output_t *out);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H_