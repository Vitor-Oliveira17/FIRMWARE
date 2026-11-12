/**
 * @file imu.c
 * @brief Implementação do Driver MPU-9250/6500 via I2C
 * 
 * Baseado no exemplo oficial da Espressif:
 * esp-idf/examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 * 
 * Registros do MPU9250 usados:
 *   0x6B = Power Management (0x00 = acordar)
 *   0x1B = Gyro Config (0x00 = escala ±250°/s, sensibilidade 131 LSB/(°/s))
 *   0x47 = Gyro Z High byte
 *   0x48 = Gyro Z Low byte
 *   0x75 = WHO_AM_I (deve retornar 0x71 para MPU9250 ou 0x70 para MPU6500)
 */

#include "imu.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "IMU";

#define I2C_MASTER_TIMEOUT_MS   1000

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

// Offset de fábrica do giroscópio (calculado na calibração)
static float gyroZ_offset = 0.0f;

/*
// Filtro de Média Móvel — suaviza o ruído do giroscópio
Mudança de média móvel -> leitura direta filtrada por hardware

#define GYRO_SAMPLES 10
static float gyro_buffer[GYRO_SAMPLES] = {0};
static int gyro_index = 0;
static float gyroZ_filtered = 0.0f;

static float gyro_moving_average(float new_sample) {
    gyro_buffer[gyro_index] = new_sample;
    gyro_index = (gyro_index + 1) % GYRO_SAMPLES;

    float soma = 0;
    for (int i = 0; i < GYRO_SAMPLES; i++) soma += gyro_buffer[i];
    return soma / GYRO_SAMPLES;
}
*/  

static float gyroZ_filtered;

// ============================================================================
// FUNÇÕES INTERNAS
// ============================================================================

/**
 * @brief Escreve 1 byte num registrador do MPU9250
 * 
 * Equivalente Arduino: Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(data); Wire.endTransmission();
 * Equivalente API legada: i2c_cmd_link_create → start → write → write → stop → execute → delete (7 linhas!)
 */
static esp_err_t mpu_write_byte(uint8_t reg_addr, uint8_t data) {          
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Lê N bytes a partir de um registrador do MPU9250
 * 
 * Equivalente Arduino: Wire.write(reg); Wire.endTransmission(false); Wire.requestFrom(0x68, len);
 * Equivalente API legada: i2c_cmd_link_create → start → write addr → write reg → repeated start → read → stop → execute → delete (9 linhas!)
 * API nova: UMA única linha! O "Repeated Start" é feito automaticamente pelo driver.
 */
static esp_err_t mpu_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

esp_err_t imu_init(void) {
    ESP_LOGI(TAG, "Inicializando I2C e MPU9250...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_PIN,
        .scl_io_num = IMU_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,                   
        .flags.enable_internal_pullup = true,      // Pull-ups externos definidos com o Lukas
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,    // Endereço de 7 bits
        .device_address = MPU_ADDR,                // 0x68 (ADO no GND)
        .scl_speed_hz = IMU_I2C_FREQ_HZ,          // 400kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    uint8_t who_am_i = 0;
    esp_err_t err = mpu_read_bytes(0x75, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 NAO respondeu no I2C!");
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (esperado: 0x71 para MPU9250)", who_am_i);

    //Acorda a MPU
    mpu_write_byte(0x6B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    //Configura a escala do giroscopio
    mpu_write_byte(0x1B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    //Configura o DLPF (filtro de hardware da leitura do giroscopio)
    // 0x01 = Bandwidth 184 Hz (Atraso de apenas 2.9 ms)
    mpu_write_byte(0x1A, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "MPU9250 inicializado com sucesso!");
    return ESP_OK;
}

esp_err_t imu_calibrate(void) {
    ESP_LOGW(TAG, " CALIBRANDO GIROSCOPIO... NAO MEXA NO ROBO!");

    int32_t soma = 0;
    int amostras_ok = 0;
    uint8_t buf[2];

    for (int i = 0; i < 500; i++) {
        if (mpu_read_bytes(0x47, buf, 2) == ESP_OK) {
            int16_t gz_raw = (int16_t)((buf[0] << 8) | buf[1]);
            soma += gz_raw;
            amostras_ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(3));  
    }

    if (amostras_ok == 0) {
        ESP_LOGE(TAG, "Nenhuma amostra lida!");
        return ESP_FAIL;
    }

    gyroZ_offset = (float)soma / (float)amostras_ok;
    ESP_LOGI(TAG, "Calibracao completa! %d amostras. Offset Z = %.2f", amostras_ok, gyroZ_offset);
    return ESP_OK;
}

float imu_read_gyro_z(void) {
    uint8_t buf[2];

    if (mpu_read_bytes(0x47, buf, 2) != ESP_OK) {
        return gyroZ_filtered;  // Se falhar a leitura, mantém o último valor filtrado
    }

    // Remonta os 2 bytes num inteiro de 16 bits com sinal
    int16_t gz_raw = (int16_t)((buf[0] << 8) | buf[1]);

    // Subtrai o offset de fábrica e converte para graus/segundo
    // Na escala ±250°/s, a sensibilidade é 131 LSB por cada °/s
    float gz_deg_s = (gz_raw - gyroZ_offset) / 131.0f;

    // Atualiza a memória e retorna o valor filtrado pelo DLPF de hardware
    gyroZ_filtered = gz_deg_s;

    return gyroZ_filtered;
}


void i2c_scanner(void) {
    if (bus_handle == NULL) {
        ESP_LOGE("SCANNER", "Barramento I2C nao foi inicializado!");
        return;
    }

    ESP_LOGI("SCANNER", "Iniciando varredura I2C...");
    int encontrados = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, 100);
        if (ret == ESP_OK) {
            ESP_LOGI("SCANNER", "Dispositivo encontrado no endereco: 0x%02X", addr);
            encontrados++;
        }
    }

    if (encontrados == 0) {
        ESP_LOGW("SCANNER", "Nenhum dispositivo respondeu no barramento I2C.");
    }
}


i2c_master_bus_handle_t imu_get_i2c_bus_handle(void)
{
    return bus_handle; // Substitua bus_handle pelo nome da sua variável i2c_master_bus_handle_t no imu.c
}

float imu_read_accel_angle(void) {
    uint8_t buf[4];
// Lê Acelerômetro X e Y (Registradores 0x3B até 0x3E no MPU9250)
    if (mpu_read_bytes(0x3B, buf, 4) != ESP_OK) {
        return 0.0f;
    }
    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);

// Calcula o ângulo em radianos em relação ao eixo Y (vertical).
    // Dependendo de qual face do MPU9250 está apontando para cima na sua placa, 
    // pode ser necessário inverter o sinal do ax_raw ou ay_raw abaixo (ex: -ax_raw).
    return atan2f((float)ax_raw, (float)ay_raw);
}