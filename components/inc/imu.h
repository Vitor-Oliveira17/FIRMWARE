/**
 * @file imu.h
 * @brief Driver do Giroscópio/Acelerômetro MPU-6050 via I2C
 * 
 * Fornece leitura do eixo Z do giroscópio para o controlador em espaço de estados.
 * O sensor comunica via I2C. Pino AD0 deve estar ligado ao GND (endereço 0x68).
 */

#ifndef IMU_H_
#define IMU_H_

#include "esp_err.h"
#include "driver/i2c_master.h"

// ============================================================================
// CONFIGURAÇÃO DO HARDWARE I2C
// ============================================================================
#define IMU_I2C_SDA_PIN     21           // GPIO 21 (Pino SDA I2C)
#define IMU_I2C_SCL_PIN     22           // GPIO 22 (Pino SCL I2C)
#define IMU_I2C_PORT        0            // Porta I2C 0 do ESP32
#define IMU_I2C_FREQ_HZ     400000       // 400kHz (I2C Fast Mode)
#define MPU_ADDR            0x68         // Endereço I2C quando AD0 = GND

// ============================================================================
// FILTRO PASSA-BAIXAS DIGITAL INTERNO DO MPU-6050 (DLPF, registrador 0x1A)
// ============================================================================
// DLPF_CFG | Accel: banda / atraso | Gyro: banda / atraso
//    0     |   260 Hz / 0.0 ms     |   256 Hz / 0.98 ms  (desligado, 8 kHz)
//    1     |   184 Hz / 2.0 ms     |   188 Hz / 1.9 ms
//    2     |    94 Hz / 3.0 ms     |    98 Hz / 2.8 ms
//    3     |    44 Hz / 4.9 ms     |    42 Hz / 4.8 ms   <- padrao
//    4     |    21 Hz / 8.5 ms     |    20 Hz / 8.3 ms
//    5     |    10 Hz / 13.8 ms    |    10 Hz / 13.4 ms
// O filtro age ANTES da amostragem: remove a vibracao do motor/roda sem
// aliasing, o que um filtro em software apos a leitura a 1 kHz nao consegue.
#define IMU_DLPF_CFG        3
// Com DLPF ligado o giroscopio amostra a 1 kHz: taxa = 1 kHz / (1 + DIV).
#define IMU_SMPLRT_DIV      0            // 1 kHz, igual a malha de controle

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o barramento I2C e acorda o MPU-6050
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
 * @return Velocidade angular em graus/segundo (já com offset subtraído e filtrada)
 */
float imu_read_gyro_z(void);

/**
 * @brief Lê a aceleração nos três eixos (X, Y, Z) em 'g' (1g = 9.81 m/s^2)
 */
esp_err_t imu_read_accel(float *ax, float *ay, float *az);

/**
 * @brief Estima o ângulo de inclinação theta (em radianos) usando filtro complementar (Giroscópio + Acelerômetro)
 * @param dt Intervalo de amostragem em segundos
 * @return Ângulo estimado em radianos, relativo à postura mantida durante a calibração.
 */
float imu_read_angle_complementary(float dt);

/**
 * @brief Lê acelerômetro e giroscópio numa única transação I2C e atualiza o filtro.
 *
 * Esta é a função indicada para a malha de controle de 1 kHz.
 */
esp_err_t imu_read_state(float dt, float *angle_rad, float *gyro_z_deg_s,
                         float *ax_g, float *ay_g, float *az_g);

/**
 * @brief Varre o barramento I2C e imprime os endereços encontrados.
 *        Função de diagnóstico; não é chamada durante a inicialização normal.
 */
void i2c_scanner(void);

/**
 * @brief Retorna o handle do barramento I2C para ser compartilhado com outros sensores (como o encoder AS5600)
 */
i2c_master_bus_handle_t imu_get_i2c_bus_handle(void);

#endif // IMU_H_
