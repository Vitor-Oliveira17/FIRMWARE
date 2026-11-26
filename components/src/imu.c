/**
 * @file imu.c
 * @brief Implementação do Driver MPU-6500 via I2C
 */

#include "imu.h"
#include <math.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soft_i2c.h"

static const char *TAG = "IMU";

// A malha executa a cada 1 ms; uma transferência não pode bloqueá-la por 1 s.
#define I2C_MASTER_TIMEOUT_MS   2

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static bool soft_ready = false;   // I2C por software inicializado

static bool imu_ready(void)
{
#if IMU_USE_SOFT_I2C
    return soft_ready;
#else
    return dev_handle != NULL;
#endif
}

/** Libera o barramento apos falha na identificacao. */
static void imu_release(void)
{
#if IMU_USE_SOFT_I2C
    soft_ready = false;
#else
    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);
    dev_handle = NULL;
    bus_handle = NULL;
#endif
}

// Offset de fábrica do giroscópio (calculado na calibração)
static float gyroZ_offset = 0.0f;

// Filtro de Média Móvel — suaviza o ruído do giroscópio
#define GYRO_SAMPLES 10
static float gyro_buffer[GYRO_SAMPLES] = {0};
static int gyro_index = 0;
static float gyroZ_filtered = 0.0f;
static float comp_angle_rad = 0.0f;
static float angle_reference_rad = 0.0f;
static bool complementary_needs_reference = true;

#define PI_F 3.14159265358979323846f

static float gyro_moving_average(float new_sample) {
    gyro_buffer[gyro_index] = new_sample;
    gyro_index = (gyro_index + 1) % GYRO_SAMPLES;

    float soma = 0;
    for (int i = 0; i < GYRO_SAMPLES; i++) soma += gyro_buffer[i];
    return soma / GYRO_SAMPLES;
}

static esp_err_t mpu_write_byte(uint8_t reg_addr, uint8_t data) {
#if IMU_USE_SOFT_I2C
    if (!soft_ready) return ESP_ERR_INVALID_STATE;
    return soft_i2c_write_reg(MPU_ADDR, reg_addr, data);
#else
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
#endif
}

static esp_err_t mpu_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (!imu_ready() || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
#if IMU_USE_SOFT_I2C
    return soft_i2c_read_regs(MPU_ADDR, reg_addr, data, len);
#else
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
#endif
}

static float wrap_angle_rad(float angle)
{
    while (angle > PI_F) {
        angle -= 2.0f * PI_F;
    }
    while (angle < -PI_F) {
        angle += 2.0f * PI_F;
    }
    return angle;
}

esp_err_t imu_init(void) {
    ESP_LOGI(TAG, "Inicializando I2C e MPU6500...");

#if IMU_USE_SOFT_I2C
    if (soft_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = soft_i2c_init(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, IMU_SOFT_I2C_HALF_PERIOD_NS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar I2C por software: %s", esp_err_to_name(err));
        return err;
    }
    int sda_lvl = 0, scl_lvl = 0;
    soft_i2c_read_lines(&sda_lvl, &scl_lvl);
    ESP_LOGI(TAG, "I2C POR SOFTWARE em SDA=GPIO%d SCL=GPIO%d (meio periodo %d ns) | linhas SDA=%d SCL=%d",
             IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, IMU_SOFT_I2C_HALF_PERIOD_NS, sda_lvl, scl_lvl);
    if (!sda_lvl || !scl_lvl) {
        ESP_LOGE(TAG, "Linha I2C presa em 0: curto, IMU danificada ou pino do ESP danificado.");
    }
    soft_ready = true;
#else
    if (bus_handle != NULL || dev_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_PIN,
        .scl_io_num = IMU_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar barramento I2C: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar MPU6500: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return err;
    }

#endif

    uint8_t who_am_i = 0;
    err = mpu_read_bytes(0x75, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6500 nao respondeu no I2C (erro=%s). Confira SDA/SCL, VCC, GND "
                 "e NCS ligado ao VCC.", esp_err_to_name(err));
        imu_release();
        return err;
    }
    if (who_am_i == IMU_WHO_AM_I_MPU6500) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (MPU6500)", who_am_i);
    } else if (who_am_i == 0x68 || who_am_i == 0x71 || who_am_i == 0x73) {
        // MPU-6050 / MPU-9250 / MPU-9255: mesmo mapa de registradores usado aqui.
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X: nao e MPU6500, mas e compativel. Continuando.",
                 who_am_i);
    } else {
        ESP_LOGE(TAG, "Sensor nao identificado (WHO_AM_I=0x%02X, esperado 0x%02X para MPU6500)",
                 who_am_i, IMU_WHO_AM_I_MPU6500);
        imu_release();
        return ESP_ERR_NOT_FOUND;
    }

    // Reset completo do chip (PWR_MGMT_1, bit DEVICE_RESET) e espera estabilizar.
    err = mpu_write_byte(0x6B, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao resetar MPU6500: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Acorda e usa o PLL do giroscopio como clock (CLKSEL = 1, mais estavel).
    err = mpu_write_byte(0x6B, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao acordar MPU6500: %s", esp_err_to_name(err));
        return err;
    }
    // Liga todos os eixos do acelerometro e do giroscopio (PWR_MGMT_2).
    err = mpu_write_byte(0x6C, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao habilitar eixos: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Filtro do giroscopio (CONFIG, 0x1A) e taxa de amostragem (SMPLRT_DIV, 0x19).
    err = mpu_write_byte(0x1A, IMU_DLPF_CFG & 0x07);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar DLPF do giroscopio: %s", esp_err_to_name(err));
        return err;
    }
    err = mpu_write_byte(0x19, IMU_SMPLRT_DIV);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar taxa de amostragem: %s", esp_err_to_name(err));
        return err;
    }

    // Giroscopio +-250 deg/s (131 LSB/(deg/s)); FCHOICE_B = 00 -> usa o DLPF acima.
    err = mpu_write_byte(0x1B, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar giroscopio: %s", esp_err_to_name(err));
        return err;
    }
    // Acelerometro +-2 g (16384 LSB/g).
    err = mpu_write_byte(0x1C, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar acelerometro: %s", esp_err_to_name(err));
        return err;
    }
    // Filtro do acelerometro (ACCEL_CONFIG2, 0x1D) - exclusivo do MPU-6500.
    err = mpu_write_byte(0x1D, IMU_ACCEL_DLPF_CFG & 0x07);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar DLPF do acelerometro: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t gyro_dlpf = 0xFF, accel_dlpf = 0xFF;
    if (mpu_read_bytes(0x1A, &gyro_dlpf, 1) == ESP_OK &&
        mpu_read_bytes(0x1D, &accel_dlpf, 1) == ESP_OK) {
        ESP_LOGI(TAG, "DLPF giroscopio = %u (configurado %d) | DLPF acelerometro = %u (configurado %d)",
                 gyro_dlpf & 0x07, IMU_DLPF_CFG, accel_dlpf & 0x07, IMU_ACCEL_DLPF_CFG);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "MPU6500 inicializado com sucesso!");
    return ESP_OK;
}

esp_err_t imu_calibrate(void) {
    ESP_LOGW(TAG, " CALIBRANDO GIROSCOPIO E ZERO DO ANGULO... NAO MEXA NO ROBO!");

    int32_t soma_gz = 0;
    float soma_ax = 0.0f;
    float soma_ay = 0.0f;
    int amostras_ok = 0;
    uint8_t buf[14];

    // Le acelerometro + giroscopio juntos (0x3B..0x48), 500 amostras (~1.5 s).
    for (int i = 0; i < 500; i++) {
        if (mpu_read_bytes(0x3B, buf, sizeof(buf)) == ESP_OK) {
            const int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
            const int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
            const int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);
            soma_ax += (float)ax_raw / 16384.0f;
            soma_ay += (float)ay_raw / 16384.0f;
            soma_gz += gz_raw;
            amostras_ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    if (amostras_ok == 0) return ESP_FAIL;

    gyroZ_offset = (float)soma_gz / (float)amostras_ok;
    gyro_index = 0;
    gyroZ_filtered = 0.0f;
    for (int i = 0; i < GYRO_SAMPLES; i++) {
        gyro_buffer[i] = 0.0f;
    }

    // Zero do angulo = MEDIA do vetor gravidade durante a calibracao, e nao uma
    // unica amostra (uma leitura isolada com ruido/vibracao deslocava o zero em
    // ate ~1 grau e fazia o controle aplicar tensao com o robo parado).
    const float ax_mean = soma_ax / (float)amostras_ok;
    const float ay_mean = soma_ay / (float)amostras_ok;
    angle_reference_rad = atan2f(-ay_mean, ax_mean);
    comp_angle_rad = angle_reference_rad;
    complementary_needs_reference = false;

    ESP_LOGI(TAG, "Calibracao completa! %d amostras. Offset Z = %.2f | zero do angulo = %.2f deg",
             amostras_ok, gyroZ_offset, angle_reference_rad * 180.0f / PI_F);
    return ESP_OK;
}

float imu_read_gyro_z(void) {
    uint8_t buf[2];

    if (mpu_read_bytes(0x47, buf, 2) != ESP_OK) {
        return gyroZ_filtered;
    }

    int16_t gz_raw = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (gz_raw - gyroZ_offset) / 131.0f;
    gyroZ_filtered = gyro_moving_average(gz_deg_s);

    return gyroZ_filtered;
}

// ============================================================================
// ACELERÔMETRO E FILTRO COMPLEMENTAR
// ============================================================================

esp_err_t imu_read_accel(float *ax, float *ay, float *az) {
    uint8_t buf[6];

    if (ax == NULL || ay == NULL || az == NULL || !imu_ready()) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mpu_read_bytes(0x3B, buf, 6) != ESP_OK) {
        return ESP_FAIL;
    }

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // Fundo de escala padrão ±2g (16384 LSB/g)
    *ax = (float)ax_raw / 16384.0f;
    *ay = (float)ay_raw / 16384.0f;
    *az = (float)az_raw / 16384.0f;

    return ESP_OK;
}

static float update_complementary(float dt, float ax, float ay, float gyro_z_deg_s)
{
    if (dt <= 0.0f || !isfinite(dt)) {
        return wrap_angle_rad(comp_angle_rad - angle_reference_rad);
    }

    // Para uma rotacao +phi em torno de Z (mesmo sentido em que o giroscopio
    // mede gz > 0), o vetor gravidade gira -phi no referencial do sensor.
    // Por isso usa-se -ay: assim o angulo do acelerometro cresce no mesmo
    // sentido da integral do giroscopio. Com atan2f(ay, ax) os dois sinais
    // ficavam opostos e o filtro complementar divergia.
    const float accel_angle_rad = atan2f(-ay, ax);
    const float gyro_z_rad_s = gyro_z_deg_s * (PI_F / 180.0f);

    if (complementary_needs_reference) {
        comp_angle_rad = accel_angle_rad;
        angle_reference_rad = accel_angle_rad;
        complementary_needs_reference = false;
        return 0.0f;
    }

    // O erro é normalizado para evitar saltos ao cruzar -pi/pi.
    const float alpha = 0.998f;
    const float predicted_angle = comp_angle_rad + gyro_z_rad_s * dt;
    const float accel_error = wrap_angle_rad(accel_angle_rad - predicted_angle);
    comp_angle_rad = wrap_angle_rad(predicted_angle + (1.0f - alpha) * accel_error);

    return wrap_angle_rad(comp_angle_rad - angle_reference_rad);
}

float imu_read_angle_complementary(float dt) {
    float ax, ay, az;
    if (imu_read_accel(&ax, &ay, &az) != ESP_OK) {
        return wrap_angle_rad(comp_angle_rad - angle_reference_rad);
    }
    (void)az;
    return update_complementary(dt, ax, ay, imu_read_gyro_z());
}

esp_err_t imu_read_state(float dt, float *angle_rad, float *gyro_z_deg_s,
                         float *ax_g, float *ay_g, float *az_g)
{
    if (angle_rad == NULL || gyro_z_deg_s == NULL || ax_g == NULL ||
        ay_g == NULL || az_g == NULL || !imu_ready()) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[14];
    esp_err_t err = mpu_read_bytes(0x3B, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    const int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    const int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    const int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    *ax_g = (float)ax_raw / 16384.0f;
    *ay_g = (float)ay_raw / 16384.0f;
    *az_g = (float)az_raw / 16384.0f;
    *gyro_z_deg_s = gyro_moving_average((gz_raw - gyroZ_offset) / 131.0f);
    gyroZ_filtered = *gyro_z_deg_s;
    *angle_rad = update_complementary(dt, *ax_g, *ay_g, *gyro_z_deg_s);

    return ESP_OK;
}

void i2c_scanner(void) {
#if IMU_USE_SOFT_I2C
    if (!soft_ready) return;
#else
    if (bus_handle == NULL) return;
#endif

    ESP_LOGI("SCANNER", "Iniciando varredura I2C...");
    int encontrados = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
#if IMU_USE_SOFT_I2C
        const esp_err_t found = soft_i2c_probe(addr);
#else
        const esp_err_t found = i2c_master_probe(bus_handle, addr, 100);
#endif
        if (found == ESP_OK) {
            ESP_LOGI("SCANNER", "Dispositivo encontrado no endereco: 0x%02X", addr);
            encontrados++;
        }
    }

    if (encontrados == 0) ESP_LOGW("SCANNER", "Nenhum dispositivo respondeu.");
}

i2c_master_bus_handle_t imu_get_i2c_bus_handle(void) {
    return bus_handle;
}
