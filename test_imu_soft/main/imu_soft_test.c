/**
 * @file imu_soft_test.c
 * @brief Teste da IMU (MPU-6050 ou MPU-6500) usando I2C POR SOFTWARE.
 *
 * Usa os GPIO 2 (SDA) e 3 (SCL) controlados diretamente, sem o periferico
 * I2C do ESP32-C6. Serve para separar as causas:
 *   - funciona aqui mas falha com o I2C de hardware -> periferico I2C danificado
 *   - falha aqui tambem -> problema nos pinos, na alimentacao ou na ligacao
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soft_i2c.h"

static const char *TAG = "IMU_SOFT";

#define PIN_SDA         2
#define PIN_SCL         3
#define PIN_BRIDGE_REN  5       // enables da ponte H forcados em 0 (motor desligado)
#define PIN_BRIDGE_LEN  7

static void bridge_off(void)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << PIN_BRIDGE_REN) | (1ULL << PIN_BRIDGE_LEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    gpio_set_level(PIN_BRIDGE_REN, 0);
    gpio_set_level(PIN_BRIDGE_LEN, 0);
}

static void verdict(const char *msg, bool ok)
{
    ESP_LOGW(TAG, "==================================================");
    if (ok) ESP_LOGI(TAG, "VEREDITO: %s", msg);
    else    ESP_LOGE(TAG, "VEREDITO: %s", msg);
    ESP_LOGW(TAG, "==================================================");
}

void app_main(void)
{
    bridge_off();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGW(TAG, "=== TESTE DA IMU COM I2C POR SOFTWARE (SDA=GPIO%d, SCL=GPIO%d, ~100 kHz) ===",
             PIN_SDA, PIN_SCL);

    // ---------------- 1. Linhas ----------------
    soft_i2c_init(PIN_SDA, PIN_SCL, 5000);   // ~100 kHz
    int sda = 0, scl = 0;
    soft_i2c_read_lines(&sda, &scl);
    ESP_LOGI(TAG, "[1] Linhas em repouso: SDA=%d SCL=%d (esperado 1 e 1)", sda, scl);
    if (!sda || !scl) {
        verdict("Linha I2C presa em 0 mesmo controlando o pino diretamente: curto para GND, "
                "IMU em curto ou pino do ESP danificado. Desconecte a IMU e rode de novo.", false);
        return;
    }

    // ---------------- 2. Varredura ----------------
    ESP_LOGI(TAG, "[2] Varredura I2C por software (0x08 a 0x77)...");
    int found = 0;
    uint8_t addr = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (soft_i2c_probe(a) == ESP_OK) {
            ESP_LOGI(TAG, "    dispositivo em 0x%02X", a);
            found++;
            if ((a == 0x68 || a == 0x69) && addr == 0) addr = a;
        }
    }
    if (found == 0 || addr == 0) {
        verdict("A IMU nao respondeu nem por software. O problema NAO e o periferico I2C: "
                "confira VCC 3.3V, GND, AD0 em GND, NCS em 3.3V (MPU-6500) e os fios. "
                "Se tudo estiver certo, os pinos ou a alimentacao da placa foram danificados.",
                false);
        return;
    }

    // ---------------- 3. WHO_AM_I ----------------
    uint8_t who = 0;
    esp_err_t err = soft_i2c_read_regs(addr, 0x75, &who, 1);
    ESP_LOGI(TAG, "[3] WHO_AM_I em 0x%02X = 0x%02X (%s)", addr, who, esp_err_to_name(err));
    if (err != ESP_OK) {
        verdict("A IMU responde ao endereco mas falha ao enviar dados.", false);
        return;
    }
    const bool is_6500 = (who != 0x68);
    ESP_LOGI(TAG, "    %s", who == 0x68 ? "MPU-6050" : who == 0x70 ? "MPU-6500" : "chip compativel/outro");

    // ---------------- 4. Configuracao e leituras ----------------
    soft_i2c_write_reg(addr, 0x6B, 0x80);          // reset
    vTaskDelay(pdMS_TO_TICKS(100));
    err = soft_i2c_write_reg(addr, 0x6B, 0x01);    // acorda, clock PLL
    if (err == ESP_OK) err = soft_i2c_write_reg(addr, 0x6C, 0x00);
    if (err == ESP_OK) err = soft_i2c_write_reg(addr, 0x1A, 0x03);
    if (err == ESP_OK) err = soft_i2c_write_reg(addr, 0x1B, 0x00);
    if (err == ESP_OK) err = soft_i2c_write_reg(addr, 0x1C, 0x00);
    if (err == ESP_OK && is_6500) err = soft_i2c_write_reg(addr, 0x1D, 0x03);
    if (err != ESP_OK) {
        verdict("Falha ao configurar a IMU (escrita nao aceita).", false);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "[4] Lendo 50 amostras (deixe a IMU PARADA)...");
    int ok = 0, fails = 0;
    float g_min = 99.0f, g_max = 0.0f, gyro_max = 0.0f, t_sum = 0.0f;
    for (int i = 0; i < 50; i++) {
        uint8_t b[14];
        if (soft_i2c_read_regs(addr, 0x3B, b, sizeof(b)) != ESP_OK) {
            fails++;
        } else {
            int16_t r[7];
            for (int k = 0; k < 7; k++) r[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
            const float ax = r[0] / 16384.0f, ay = r[1] / 16384.0f, az = r[2] / 16384.0f;
            const float g = sqrtf(ax * ax + ay * ay + az * az);
            g_min = fminf(g_min, g);
            g_max = fmaxf(g_max, g);
            for (int k = 4; k < 7; k++) gyro_max = fmaxf(gyro_max, fabsf(r[k] / 131.0f));
            t_sum += is_6500 ? r[3] / 333.87f + 21.0f : r[3] / 340.0f + 36.53f;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    const float t_avg = ok ? t_sum / ok : NAN;
    ESP_LOGI(TAG, "    ok=%d falhas=%d | |a| min=%.2f max=%.2f g | giro max=%.1f deg/s | T=%.1f C",
             ok, fails, g_min, g_max, gyro_max, t_avg);

    if (ok == 0) {
        verdict("Nenhuma leitura funcionou.", false);
    } else if (t_avg > 60.0f) {
        verdict("Temperatura interna muito alta: IMU em curto. Desconecte.", false);
    } else if (g_min < 0.8f || g_max > 1.2f || gyro_max > 20.0f) {
        verdict("Leituras fora do esperado com a IMU parada: sensor suspeito (ou foi mexida).", false);
    } else {
        verdict("IMU OK pelo I2C POR SOFTWARE. Se o I2C de hardware falha nestes mesmos pinos, "
                "o periferico I2C desta placa esta danificado.", true);
    }

    // ---------------- Leitura continua ----------------
    while (true) {
        uint8_t b[14];
        if (soft_i2c_read_regs(addr, 0x3B, b, sizeof(b)) == ESP_OK) {
            int16_t r[7];
            for (int k = 0; k < 7; k++) r[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
            ESP_LOGI(TAG, "acc:(%+.2f,%+.2f,%+.2f) g | giro:(%+7.1f,%+7.1f,%+7.1f) deg/s",
                     r[0] / 16384.0f, r[1] / 16384.0f, r[2] / 16384.0f,
                     r[4] / 131.0f, r[5] / 131.0f, r[6] / 131.0f);
        } else {
            ESP_LOGE(TAG, "falha na leitura");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
