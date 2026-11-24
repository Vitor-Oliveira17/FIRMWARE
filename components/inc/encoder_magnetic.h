/**
 * @file encoder_magnetic.h
 * @brief Driver para Encoder Magnético I2C (AS5600)
 */

#ifndef ENCODER_MAGNETIC_H_
#define ENCODER_MAGNETIC_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define AS5600_ADDR         0x36
#define AS5600_RAW_ANGLE    0x0C
#define AS5600_ANGLE        0x0E

float encoder_read_angle(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Calcula e retorna o RPM baseado na variação do ângulo ao longo do tempo.
 * 
 * @param bus_handle Handle do barramento I2C
 * @return float Rotações por Minuto (RPM). Positivo (Frente) ou Negativo (Trás).
 */
float encoder_read_rpm(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Ultimo angulo lido com sucesso por encoder_read_rpm() (graus, -1 se nenhum).
 *        Nao acessa o barramento I2C.
 */
float encoder_get_last_angle(void);

/**
 * @brief Numero acumulado de falhas de leitura do AS5600.
 */
uint32_t encoder_get_error_count(void);

/**
 * @brief Ultimo codigo de erro I2C do AS5600 (ESP_OK se nunca falhou).
 */
esp_err_t encoder_get_last_error(void);

/**
 * @brief Diagnostico na inicializacao: verifica se o AS5600 responde no I2C,
 *        le STATUS/AGC/MAGNITUDE (deteccao do ima) e imprime 10 angulos.
 */
esp_err_t encoder_diagnose(i2c_master_bus_handle_t bus_handle);

#endif // ENCODER_MAGNETIC_H_