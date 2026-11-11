#include "encoder_magnetic.h"
#include "esp_log.h"
#include "esp_timer.h" // Necessário para pegar o tempo em microsegundos

static const char *TAG = "ENCODER_MAG";
static i2c_master_dev_handle_t dev_handle = NULL;

// Variáveis de estado para o cálculo do RPM
static float last_angle = -1.0f;
static int64_t last_time_us = 0;

// 1. Memória do filtro passa-baixas
static float rpm_filtered = 0.0f;

// 2. Fator de suavização alpha(0.08 é ideal para Ts = 1 ms e AS5600)
#define RPM_LPF_ALPHA 0.08f

float encoder_read_angle(i2c_master_bus_handle_t bus_handle)
{
    // ... [Mantenha a sua função encoder_read_angle exatamente como está] ...
    if (bus_handle == NULL) return -1.0f;

    if (dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = AS5600_ADDR,
            .scl_speed_hz = 400000,
        };
        esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (err != ESP_OK) return -1.0f;
    }

    uint8_t reg = AS5600_ANGLE;
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, 50);
    if (err != ESP_OK) return -1.0f;

    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    raw_angle &= 0x0FFF;

    return ((float)raw_angle * 360.0f) / 4096.0f;
}

float encoder_read_rpm(i2c_master_bus_handle_t bus_handle)
{
    float current_angle = encoder_read_angle(bus_handle);
    if (current_angle < 0.0f) {
        return 0.0f; // Falha na leitura
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
    if (delta_time_sec <= 0.0f) return rpm_filtered; // Evita divisão por zero

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
    float rpm_raw = (delta_angle / 360.0f) / delta_time_sec * 60.0f;

    //Filtro passa-baixa
    rpm_filtered = (RPM_LPF_ALPHA * rpm_raw) + ((1.0f - RPM_LPF_ALPHA) * rpm_filtered); //Filtro passa baixa discreto

    // Atualiza o histórico para a próxima chamada
    last_angle = current_angle;
    last_time_us = current_time_us;

    return rpm_filtered;
}