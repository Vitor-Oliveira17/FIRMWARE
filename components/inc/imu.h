/**
 * @file imu.h
 * @brief Driver do Giroscópio/Acelerômetro MPU-6500 via I2C
 * 
 * Fornece leitura do eixo Z do giroscópio para o controlador em espaço de estados.
 * O sensor comunica via I2C. Pino AD0/SDO deve estar ligado ao GND (endereço 0x68)
 * e o pino NCS ligado ao VCC (3.3 V) para manter o chip no modo I2C.
 */

#ifndef IMU_H_
#define IMU_H_

#include "esp_err.h"
#include "driver/i2c_master.h"

// ============================================================================
// CONFIGURAÇÃO DO HARDWARE I2C
// ============================================================================
#define IMU_I2C_SDA_PIN     21           // GPIO 21 (SDA padrao do ESP32)
#define IMU_I2C_SCL_PIN     22           // GPIO 22 (SCL padrao do ESP32)
#define IMU_I2C_PORT        0            // Porta I2C 0 do ESP32
#define IMU_I2C_FREQ_HZ     400000       // 400kHz (I2C Fast Mode)
#define MPU_ADDR            0x68         // Endereço I2C quando AD0/SDO = GND

// ----------------------------------------------------------------------------
// I2C POR SOFTWARE (bit-bang)
// ----------------------------------------------------------------------------
// 1 -> usa um barramento I2C virtual nos pinos SDA/SCL acima, controlados
//      diretamente por software (nao usa o controlador I2C do chip).
// 0 -> usa o controlador I2C de hardware do ESP32 (IMU_I2C_FREQ_HZ).
// No modo software o encoder AS5600 nao e suportado (use USE_ENCODER 0).
#define IMU_USE_SOFT_I2C            1
// Meio periodo do clock do I2C virtual, em nanossegundos.
// 1250 ns ~ 400 kHz: a leitura completa da IMU leva ~0.45 ms e cabe no ciclo
// de 1 ms da malha. Se houver falhas de leitura (errIMU subindo), aumente
// para 1600 (~300 kHz). Acima de ~2000 a leitura nao cabe mais em 1 ms.
#define IMU_SOFT_I2C_HALF_PERIOD_NS 1250

// WHO_AM_I (registrador 0x75) esperado para o MPU-6500.
#define IMU_WHO_AM_I_MPU6500    0x70

// ============================================================================
// FILTROS PASSA-BAIXAS DIGITAIS INTERNOS DO MPU-6500
// ============================================================================
// No MPU-6500 o giroscopio e o acelerometro tem filtros SEPARADOS.
//
// Giroscopio: DLPF_CFG em CONFIG (0x1A), com FCHOICE_B = 00 em 0x1B.
// DLPF_CFG | Gyro: banda / atraso
//    0     |   250 Hz /  0.97 ms
//    1     |   184 Hz /  2.9 ms
//    2     |    92 Hz /  3.9 ms
//    3     |    41 Hz /  5.9 ms   <- padrao
//    4     |    20 Hz /  9.9 ms
//    5     |    10 Hz / 17.9 ms
//    6     |     5 Hz / 33.5 ms
#define IMU_DLPF_CFG        3
//
// Acelerometro: A_DLPF_CFG em ACCEL_CONFIG2 (0x1D), com ACCEL_FCHOICE_B = 0.
// A_DLPF_CFG | Accel: banda / atraso
//    1       |  218 Hz /  1.9 ms
//    2       |   99 Hz /  2.9 ms
//    3       |   45 Hz /  4.9 ms   <- padrao
//    4       |   21 Hz /  8.9 ms
//    5       |   10 Hz / 16.8 ms
//    6       |    5 Hz / 32.5 ms
#define IMU_ACCEL_DLPF_CFG  3
//
// O filtro age ANTES da amostragem: remove a vibracao do motor/roda sem
// aliasing, o que um filtro em software apos a leitura a 1 kHz nao consegue.
// Com DLPF ligado a taxa interna e 1 kHz: taxa = 1 kHz / (1 + DIV).
#define IMU_SMPLRT_DIV      0            // 1 kHz, igual a malha de controle

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o barramento I2C e acorda o MPU-6500
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
