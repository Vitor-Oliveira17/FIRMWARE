/**
 * @file imu_test.c
 * @brief Teste ISOLADO da IMU MPU-6500 (ou MPU-6050) no ESP32-C6.
 *
 * Nao usa motor, encoder nem controle. Responde: a IMU esta viva ou queimada?
 *
 * Etapas:
 *   1. Linhas SDA/SCL: com pull-up, as duas devem estar em nivel ALTO.
 *      Linha presa em 0 = curto para GND ou chip queimado segurando a linha.
 *   2. Varredura do barramento I2C (100 kHz, mais tolerante que 400 kHz).
 *   3. WHO_AM_I (0x75) nos enderecos 0x68 e 0x69.
 *   4. Acorda o sensor e le acelerometro, giroscopio e TEMPERATURA interna
 *      por alguns segundos, verificando se os valores fazem sentido.
 *   5. Veredito final e leitura continua a cada 1 s.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IMU_TEST";

// ============================================================================
// CONFIGURACAO (igual a ligacao do robo)
// ============================================================================
#define PIN_SDA             2
#define PIN_SCL             3
#define I2C_FREQ_HZ         100000      // 100 kHz para o diagnostico
#define I2C_TIMEOUT_MS      50

// Enables da ponte H: forcados em 0 para o motor nunca ligar neste teste.
#define PIN_BRIDGE_REN      5
#define PIN_BRIDGE_LEN      7

// ============================================================================

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t mpu;
static bool is_6500 = true;   // define a formula de temperatura

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

/** Etapa 1: com pull-up interno, SDA e SCL em repouso devem ler 1. */
static bool check_lines(void)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << PIN_SDA) | (1ULL << PIN_SCL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    vTaskDelay(pdMS_TO_TICKS(20));

    const int sda = gpio_get_level(PIN_SDA);
    const int scl = gpio_get_level(PIN_SCL);
    ESP_LOGI(TAG, "[1] Linhas em repouso: SDA(GPIO%d)=%d  SCL(GPIO%d)=%d  (esperado 1 e 1)",
             PIN_SDA, sda, PIN_SCL, scl);

    if (!sda) {
        ESP_LOGE(TAG, "    SDA presa em 0: curto para GND ou chip queimado segurando a linha.");
    }
    if (!scl) {
        ESP_LOGE(TAG, "    SCL presa em 0: curto para GND ou chip queimado segurando a linha.");
    }

    gpio_reset_pin(PIN_SDA);
    gpio_reset_pin(PIN_SCL);
    return sda && scl;
}

static esp_err_t bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = 0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &bus);
}

/** Etapa 2: lista tudo que responde no barramento. */
static int scan_bus(bool *found_68, bool *found_69)
{
    int count = 0;
    *found_68 = false;
    *found_69 = false;
    ESP_LOGI(TAG, "[2] Varredura I2C (0x08 a 0x77)...");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "    dispositivo em 0x%02X%s", addr,
                     addr == 0x68 ? "  <- MPU (AD0=GND)" :
                     addr == 0x69 ? "  <- MPU (AD0=VCC)" :
                     addr == 0x36 ? "  <- AS5600 (encoder)" : "");
            count++;
            if (addr == 0x68) *found_68 = true;
            if (addr == 0x69) *found_69 = true;
        }
    }
    if (count == 0) {
        ESP_LOGE(TAG, "    Nenhum dispositivo respondeu.");
    }
    return count;
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mpu, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t mpu_write(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(mpu, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

typedef struct {
    float ax, ay, az, gx, gy, gz, temp_c;
} mpu_sample_t;

static esp_err_t mpu_sample(mpu_sample_t *s, int16_t raw[7])
{
    uint8_t b[14];
    esp_err_t err = mpu_read(0x3B, b, sizeof(b));
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 7; i++) {
        raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    }
    s->ax = raw[0] / 16384.0f;
    s->ay = raw[1] / 16384.0f;
    s->az = raw[2] / 16384.0f;
    s->temp_c = is_6500 ? raw[3] / 333.87f + 21.0f : raw[3] / 340.0f + 36.53f;
    s->gx = raw[4] / 131.0f;
    s->gy = raw[5] / 131.0f;
    s->gz = raw[6] / 131.0f;
    return ESP_OK;
}

static void verdict(const char *msg, bool ok)
{
    ESP_LOGW(TAG, "==================================================");
    if (ok) {
        ESP_LOGI(TAG, "VEREDITO: %s", msg);
    } else {
        ESP_LOGE(TAG, "VEREDITO: %s", msg);
    }
    ESP_LOGW(TAG, "==================================================");
}

void app_main(void)
{
    bridge_off();
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGW(TAG, "=== TESTE ISOLADO DA IMU MPU-6500/6050 (SDA=GPIO%d, SCL=GPIO%d, %d kHz) ===",
             PIN_SDA, PIN_SCL, I2C_FREQ_HZ / 1000);

    // ---------------- 1. Linhas ----------------
    const bool lines_ok = check_lines();

    // ---------------- 2. Varredura ----------------
    esp_err_t err = bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar barramento I2C: %s", esp_err_to_name(err));
        verdict("Nao foi possivel iniciar o I2C (confira os pinos no codigo).", false);
        return;
    }
    bool f68 = false, f69 = false;
    scan_bus(&f68, &f69);

    if (!f68 && !f69) {
        verdict(lines_ok ?
                "IMU NAO RESPONDE. Com as linhas OK, isso indica IMU QUEIMADA ou "
                "sem alimentacao: meca 3.3 V entre VCC e GND da placa da IMU." :
                "IMU NAO RESPONDE e uma linha I2C esta presa em 0. Desconecte a IMU "
                "e rode de novo: se as linhas voltarem a 1, a IMU esta em curto (QUEIMADA).",
                false);
        return;
    }
    const uint8_t addr = f68 ? 0x68 : 0x69;

    // ---------------- 3. WHO_AM_I ----------------
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &mpu));

    uint8_t who = 0;
    err = mpu_read(0x75, &who, 1);
    ESP_LOGI(TAG, "[3] WHO_AM_I em 0x%02X = 0x%02X (%s)", addr, who, esp_err_to_name(err));
    if (err != ESP_OK) {
        verdict("O chip responde ao endereco mas FALHA ao enviar dados: comunicacao "
                "instavel ou chip danificado.", false);
        return;
    }
    switch (who) {
    case 0x68: ESP_LOGI(TAG, "    MPU-6050."); is_6500 = false; break;
    case 0x70: ESP_LOGI(TAG, "    MPU-6500."); break;
    case 0x71: ESP_LOGW(TAG, "    MPU-9250 (compativel)."); break;
    case 0x72: case 0x98: case 0x19:
        ESP_LOGW(TAG, "    Clone compativel do MPU-6050."); break;
    default:
        ESP_LOGE(TAG, "    Valor inesperado: chip diferente ou danificado.");
    }

    // ---------------- 4. Leituras ----------------
    mpu_write(0x6B, 0x80);                   // reset
    vTaskDelay(pdMS_TO_TICKS(100));
    if (mpu_write(0x6B, 0x01) != ESP_OK ||   // acorda (clock PLL)
        mpu_write(0x6C, 0x00) != ESP_OK ||   // todos os eixos ligados
        mpu_write(0x1A, 0x03) != ESP_OK ||   // DLPF giroscopio ~41 Hz
        mpu_write(0x1B, 0x00) != ESP_OK ||   // giro +-250 deg/s
        mpu_write(0x1C, 0x00) != ESP_OK ||   // accel +-2 g
        (is_6500 && mpu_write(0x1D, 0x03) != ESP_OK)) {   // DLPF accel ~45 Hz (6500)
        verdict("Falha ao configurar a IMU: escrita I2C nao aceita.", false);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "[4] Lendo 50 amostras (deixe a IMU PARADA)...");
    int ok = 0, fails = 0, stuck = 0;
    float g_min = 99.0f, g_max = 0.0f, gyro_max = 0.0f, t_sum = 0.0f;
    int16_t first[7] = {0};
    bool varied = false;

    for (int i = 0; i < 50; i++) {
        mpu_sample_t s;
        int16_t raw[7];
        if (mpu_sample(&s, raw) != ESP_OK) {
            fails++;
        } else {
            ok++;
            const float g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
            g_min = fminf(g_min, g);
            g_max = fmaxf(g_max, g);
            gyro_max = fmaxf(gyro_max, fmaxf(fabsf(s.gx), fmaxf(fabsf(s.gy), fabsf(s.gz))));
            t_sum += s.temp_c;
            bool all_same = true;
            for (int k = 0; k < 7; k++) {
                if (raw[k] != -1 && raw[k] != 0) all_same = false;
            }
            if (all_same) stuck++;
            if (i == 0) {
                for (int k = 0; k < 7; k++) first[k] = raw[k];
            } else {
                for (int k = 0; k < 7; k++) {
                    if (raw[k] != first[k]) varied = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    const float t_avg = ok ? t_sum / ok : NAN;
    ESP_LOGI(TAG, "    leituras ok=%d falhas=%d | |a| min=%.2f max=%.2f g | giro max=%.1f deg/s | "
             "temperatura interna=%.1f C", ok, fails, g_min, g_max, gyro_max, t_avg);

    // ---------------- 5. Veredito ----------------
    if (ok == 0) {
        verdict("Nenhuma leitura de dados funcionou: IMU DANIFICADA ou comunicacao ruim.", false);
    } else if (stuck == ok || !varied) {
        verdict("Os dados estao TRAVADOS (sempre iguais / 0 / 0xFFFF): sensor QUEIMADO "
                "por dentro, mesmo respondendo no I2C.", false);
    } else if (t_avg > 60.0f) {
        verdict("Temperatura interna muito ALTA: o chip esta aquecendo sozinho -> "
                "QUEIMADO / em curto. Desconecte.", false);
    } else if (g_min < 0.8f || g_max > 1.2f) {
        verdict("O acelerometro NAO mede 1 g parado: sensor DANIFICADO (ou a IMU estava "
                "sendo mexida durante o teste - repita parado).", false);
    } else if (gyro_max > 20.0f) {
        verdict("Giroscopio marcando rotacao com a IMU parada: giroscopio DANIFICADO "
                "(ou a IMU foi mexida - repita parado).", false);
    } else if (fails > 0) {
        verdict("IMU funciona, mas houve falhas de comunicacao: revise fios e pull-ups.", true);
    } else {
        verdict("IMU OK! Comunicacao e sensores funcionando normalmente.", true);
    }

    // ---------------- Leitura continua ----------------
    ESP_LOGI(TAG, "Leitura continua (1 s). Gire a IMU com a mao e veja os valores mudarem:");
    while (true) {
        mpu_sample_t s;
        int16_t raw[7];
        err = mpu_sample(&s, raw);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "acc:(%+.2f,%+.2f,%+.2f) g |a|=%.2f | giro:(%+7.1f,%+7.1f,%+7.1f) deg/s | T=%.1f C",
                     s.ax, s.ay, s.az, sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az),
                     s.gx, s.gy, s.gz, s.temp_c);
        } else {
            ESP_LOGE(TAG, "falha na leitura: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
