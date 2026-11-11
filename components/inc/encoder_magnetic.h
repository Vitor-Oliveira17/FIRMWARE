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

#endif // ENCODER_MAGNETIC_H_