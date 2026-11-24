#include "encoder_magnetic.h"
#include "esp_log.h"
#include "esp_timer.h" // Necessário para pegar o tempo em microsegundos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ENCODER_MAG";
static i2c_master_dev_handle_t dev_handle = NULL;

#define AS5600_I2C_TIMEOUT_MS 2

// Variáveis de estado para o cálculo do RPM
static float last_angle = -1.0f;
static int64_t last_time_us = 0;
static float last_rpm = 0.0f;
static uint32_t read_errors = 0;
static esp_err_t last_error = ESP_OK;

static esp_err_t ensure_device(i2c_master_bus_handle_t bus_handle)
{
    if (dev_handle != NULL) {
        return ESP_OK;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 50);
}

float encoder_read_angle(i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == NULL) return -1.0f;

    esp_err_t err = ensure_device(bus_handle);
    if (err != ESP_OK) {
        last_error = err;
        return -1.0f;
    }

    uint8_t reg = AS5600_ANGLE;
    uint8_t data[2] = {0};

    err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2,
                                      AS5600_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        last_error = err;
        return -1.0f;
    }

    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    raw_angle &= 0x0FFF;

    return ((float)raw_angle * 360.0f) / 4096.0f;
}

float encoder_read_rpm(i2c_master_bus_handle_t bus_handle)
{
    float current_angle = encoder_read_angle(bus_handle);
    if (current_angle < 0.0f) {
        // Falha na leitura: mantem o ultimo valor para nao gerar degrau falso
        // de velocidade na malha de controle.
        read_errors++;
        return last_rpm;
    }

    int64_t current_time_us = esp_timer_get_time(); // Tempo atual do ESP32

    // Primeira vez que a função roda: inicializa as variáveis e retorna 0
    if (last_angle < 0.0f) {
        last_angle = current_angle;
        last_time_us = current_time_us;
        return 0.0f;
    }

    // Calcula o tempo que passou (em segundos)
    float delta_time_sec = (float)(current_time_us - last_time_us) / 1000000.0f;
    if (delta_time_sec <= 0.0f) return 0.0f; // Evita divisão por zero

    // Calcula a variação de ângulo
    float delta_angle = current_angle - last_angle;

    // Lógica para corrigir o cruzamento do zero (0 para 359 ou 359 para 0)
    // 180 graus é a tolerância. Se variou mais que meia volta de uma vez, é o wrap-around.
    if (delta_angle > 180.0f) {
        delta_angle -= 360.0f; // Ex: Estava em 10 e foi pra 350. Andou -20°. (340 - 360 = -20)
    } 
    else if (delta_angle < -180.0f) {
        delta_angle += 360.0f; // Ex: Estava em 350 e foi pra 10. Andou +20°. (-340 + 360 = +20)
    }

    // RPM = (Delta Ângulo em Graus / 360) / (Delta Tempo em segundos) * 60
    float rpm = (delta_angle / 360.0f) / delta_time_sec * 60.0f;

    // Atualiza o histórico para a próxima chamada
    last_angle = current_angle;
    last_time_us = current_time_us;
    last_rpm = rpm;

    return rpm;
}

float encoder_get_last_angle(void)
{
    return last_angle;
}

uint32_t encoder_get_error_count(void)
{
    return read_errors;
}

esp_err_t encoder_get_last_error(void)
{
    return last_error;
}

esp_err_t encoder_diagnose(i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGW(TAG, "===== DIAGNOSTICO AS5600 =====");
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "Barramento I2C nao inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    // 1. O chip responde no endereco 0x36?
    esp_err_t err = i2c_master_probe(bus_handle, AS5600_ADDR, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 NAO responde em 0x%02X (%s).", AS5600_ADDR, esp_err_to_name(err));
        ESP_LOGE(TAG, "-> Problema de LIGACAO: confira SDA(GPIO%d) SCL(GPIO%d), VCC 3.3V e GND do AS5600.",
                 21, 22);
        return err;
    }
    ESP_LOGI(TAG, "AS5600 respondeu em 0x%02X.", AS5600_ADDR);

    err = ensure_device(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao registrar o AS5600 no barramento: %s", esp_err_to_name(err));
        return err;
    }

    // 2. Estado do ima (STATUS 0x0B: bit5 MD, bit4 ML, bit3 MH), AGC e MAGNITUDE.
    uint8_t status = 0, agc = 0, mag[2] = {0};
    err = read_regs(0x0B, &status, 1);
    if (err == ESP_OK) err = read_regs(0x1A, &agc, 1);
    if (err == ESP_OK) err = read_regs(0x1B, mag, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler registradores de status: %s", esp_err_to_name(err));
        return err;
    }
    const bool md = status & 0x20, ml = status & 0x10, mh = status & 0x08;
    const uint16_t magnitude = (((uint16_t)mag[0] << 8) | mag[1]) & 0x0FFF;
    ESP_LOGI(TAG, "STATUS=0x%02X  MD(ima detectado)=%d  ML(fraco)=%d  MH(forte)=%d  AGC=%u  MAGNITUDE=%u",
             status, md, ml, mh, agc, magnitude);
    if (!md) {
        ESP_LOGE(TAG, "-> IMA NAO DETECTADO: use ima DIAMETRAL, centralizado sobre o chip.");
    } else if (ml) {
        ESP_LOGW(TAG, "-> Ima FRACO/longe demais: aproxime (ideal ~0.5 a 3 mm).");
    } else if (mh) {
        ESP_LOGW(TAG, "-> Ima FORTE/perto demais: afaste um pouco.");
    } else {
        ESP_LOGI(TAG, "-> Ima OK.");
    }

    // 3. Algumas leituras de angulo: gire a roda com a mao durante estes 2 s.
    ESP_LOGI(TAG, "Gire a roda com a mao: 10 leituras de angulo em 2 s...");
    for (int i = 0; i < 10; i++) {
        uint8_t raw[2] = {0};
        err = read_regs(AS5600_RAW_ANGLE, raw, 2);
        if (err == ESP_OK) {
            const uint16_t r = (((uint16_t)raw[0] << 8) | raw[1]) & 0x0FFF;
            ESP_LOGI(TAG, "  angulo bruto = %4u  (%.1f deg)", r, r * 360.0f / 4096.0f);
        } else {
            ESP_LOGE(TAG, "  leitura falhou: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGW(TAG, "===== FIM DO DIAGNOSTICO =====");
    return ESP_OK;
}
