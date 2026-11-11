/**
 * @file state_controller.h
 * @brief Controlador em Espaço de Estados Discreto para o Pêndulo com Roda de Inércia (SEC)
 *
 * Modelo Discreto:
 *   x[k+1] = Ad * x[k] + Bd * u[k]
 *   y[k]   = Cd * x[k] + Dd * u[k]
 *
 * Lei de Controle com Realimentação de Estados e Referência:
 *   u_raw[k] = r[k] - Kd * x[k]
 *            = r[k] - (Kd1 * x1[k] + Kd2 * x2[k] + Kd3 * x3[k])
 *   u[k]     = saturate(u_raw[k], V_min, V_max)
 */

#ifndef STATE_CONTROLLER_H_
#define STATE_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// PARÂMETROS PADRÃO DO MODELO DISCRETO E GANHOS (MATLAB / SIMULINK)
// ============================================================================
#define STATE_CTRL_TS_DEFAULT           0.001f    // Ts = 1 ms (1000 Hz)
#define STATE_CTRL_VNOM_DEFAULT         24.0f     // Tensão nominal máxima da fonte/motor (V)
#define STATE_CTRL_VMAX_DEFAULT         24.0f     // Saturação superior (V)
#define STATE_CTRL_VMIN_DEFAULT        -24.0f     // Saturação inferior (V)

// Ganhos Kd obtidos a partir dos polos [-52.2083, -4.6407, -0.2471]
// u = r + (Kd1*x1 + Kd2*x2 + Kd3*x3) -> Sinais de Kd vem negativos do matlab
#define STATE_CTRL_KD1_DEFAULT         -52.2083f  // Ganho do estado x1 (Ângulo theta do pêndulo em rad)
#define STATE_CTRL_KD2_DEFAULT          -4.6407f  // Ganho do estado x2 (Velocidade angular theta_dot em rad/s)
#define STATE_CTRL_KD3_DEFAULT          -0.2471f  // Ganho do estado x3 (Velocidade angular da roda omega_r em rad/s)

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

/**
 * @brief Vetor de estados do sistema x[k]
 */
typedef struct {
    float theta;      /**< x1: Ângulo de inclinação do pêndulo (radianos) */
    float theta_dot;  /**< x2: Velocidade angular do pêndulo (rad/s) */
    float omega_r;    /**< x3: Velocidade angular da roda de reação (rad/s) */
} state_vector_t;

/**
 * @brief Configurações do controlador em espaço de estados
 */
typedef struct {
    float kd[3];      /**< Ganhos de realimentação [Kd1, Kd2, Kd3] */
    float r;          /**< Referência de ângulo do pêndulo r[k] (radianos, padrão 0.0) */
    float v_nom;      /**< Tensão nominal para conversão PWM (Volts) */
    float v_max;      /**< Saturação máxima de tensão (Volts) */
    float v_min;      /**< Saturação mínima de tensão (Volts) */
    float ts;         /**< Período de amostragem em segundos */
} state_controller_config_t;

/**
 * @brief Dados de saída do controlador a cada período de amostragem
 */
typedef struct {
    float u_raw;      /**< Tensão calculada antes da saturação (Volts) */
    float voltage;    /**< Tensão final aplicada após saturação (Volts) */
    int16_t duty;     /**< Duty cycle correspondente para a ponte H (-255 a +255) */
} state_controller_output_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o controlador com as configurações padrão do projeto.
 */
void state_controller_init(void);

/**
 * @brief Inicializa o controlador com configurações customizadas.
 * 
 * @param cfg Ponteiro para a estrutura com os parâmetros customizados.
 */
void state_controller_init_custom(const state_controller_config_t *cfg);

/**
 * @brief Atualiza os ganhos Kd em tempo de execução.
 * 
 * @param kd1 Ganho para theta
 * @param kd2 Ganho para theta_dot
 * @param kd3 Ganho para omega_r
 */
void state_controller_set_gains(float kd1, float kd2, float kd3);

/**
 * @brief Define a referência de ângulo do pêndulo (em radianos).
 * 
 * @param reference_rad Referência desejada (normalmente 0 rad para vertical).
 */
void state_controller_set_reference(float reference_rad);

/**
 * @brief Executa o cálculo da lei de controle discreto (u[k] = sat(r - Kd*x[k])).
 * 
 * @param[in]  x   Vetor de estados no instante atual x[k].
 * @param[out] out Ponteiro para estrutura que receberá u_raw, voltage e duty.
 * @return int16_t Duty cycle pronto para envio direto à função motor_set_speed().
 */
int16_t state_controller_update(const state_vector_t *x, state_controller_output_t *out);

/**
 * @brief Obtém a configuração ativa no controlador.
 */
void state_controller_get_config(state_controller_config_t *out_cfg);

#ifdef __cplusplus
}
#endif

#endif // STATE_CONTROLLER_H_

