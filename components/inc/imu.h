/**
 * @file imu.h
 * @brief Driver do Giroscópio MPU-9250/6500 via I2C para o ESP32-C6
 * 
 * Fornece leitura do eixo Z do giroscópio para o controlador PD de direcção.
 * O sensor comunica via I2C (não CAN). Pino ADO deve estar ligado ao GND (endereço 0x68).
 */

#ifndef IMU_H_
#define IMU_H_

#include "esp_err.h"
#include "driver/i2c_master.h"

// ============================================================================
// CONFIGURAÇÃO DO HARDWARE I2C
// ============================================================================
#define IMU_I2C_SDA_PIN     21             // GPIO 21
#define IMU_I2C_SCL_PIN     22           // GPIO 22
#define IMU_I2C_PORT        0           // Porta I2C 0 do ESP32-C6
#define IMU_I2C_FREQ_HZ     400000      // 400kHz (I2C Fast Mode)
#define MPU_ADDR            0x68        // Endereço I2C quando ADO = GND

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o barramento I2C e acorda o MPU9250
 * @return ESP_OK se o sensor respondeu, ESP_FAIL se não foi detectado
 */
esp_err_t imu_init(void);

/**
 * @brief Calibra o giroscópio com 500 amostras (robô deve estar PARADO)
 * @return ESP_OK se a calibração correu bem
 */
esp_err_t imu_calibrate(void);

/**
 * @brief Lê a velocidade angular do eixo Z
 * @return Velocidade angular em graus/segundo (já com offset subtraído)
 */
float imu_read_gyro_z(void);

void i2c_scanner(void);

i2c_master_bus_handle_t imu_get_i2c_bus_handle(void);

#endif
