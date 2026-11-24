/**
 * @file motor_test.h
 * @brief Teste em malha aberta da ponte BTS7960 e do encoder AS5600.
 */

#ifndef MOTOR_TEST_H_
#define MOTOR_TEST_H_

#include "esp_err.h"
#include "driver/i2c_master.h"

// O modo de operacao (equilibrio / sensores / teste do motor) agora e
// escolhido por SEC_MODE em main/main.c.

/**
 * @brief Inicia uma única sequência de PWM: +25%, +50%, +75%, +100% e reverso.
 *
 * A malha de equilíbrio deve permanecer desativada enquanto este teste executa.
 */
esp_err_t motor_test_start(i2c_master_bus_handle_t bus_handle);

#endif // MOTOR_TEST_H_
