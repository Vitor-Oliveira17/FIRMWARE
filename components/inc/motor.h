/**
 * @file motor.h
 * @brief Driver de controle de Motor DC usando ponte H (BTS7960) e PWM (LEDC)
 * 
 * Este módulo gerencia os sinais de controle (RPWM, LPWM, R_EN, L_EN)
 * para definir a velocidade e o sentido de rotação do motor DC.
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa os pinos de controle e os periféricos PWM (LEDC) do motor.
 * 
 * Configura os GPIOs de Enable e inicializa os canais de PWM para controle
 * bidirecional da ponte H BTS7960.
 */
void motor_init(void);

/**
 * @brief Define a velocidade e o sentido de rotação do motor.
 * 
 * @param speed Valor da velocidade desejada. 
 *              Varia de -255 (velocidade máxima para trás) 
 *              a +255 (velocidade máxima para frente).
 *              O valor 0 aciona o freio (motor parado).
 */
void motor_set_speed(int16_t speed);

#endif // MOTOR_H_