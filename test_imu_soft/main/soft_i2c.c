/**
 * @file soft_i2c.c
 * @brief Implementacao do I2C mestre por software (bit-bang) - versao rapida.
 *
 * As linhas sao acionadas escrevendo direto nos registradores do GPIO
 * (GPIO_OUT_W1TS/W1TC e GPIO_IN), sem passar pelo driver gpio_set_level(),
 * que e lento demais para uma leitura completa da IMU caber em 1 ms.
 * A temporizacao usa o contador de ciclos da CPU.
 */
#include "soft_i2c.h"

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define STRETCH_TIMEOUT_US  1000    // tempo maximo de clock stretching

static uint32_t mask_sda = 0;
static uint32_t mask_scl = 0;
static uint32_t half_cycles = 200;  // meio periodo em ciclos de CPU

// Open-drain: nivel 0 puxa a linha para GND; nivel 1 solta (pull-up leva a 1).
static inline void sda_low(void)  { REG_WRITE(GPIO_OUT_W1TC_REG, mask_sda); }
static inline void sda_rel(void)  { REG_WRITE(GPIO_OUT_W1TS_REG, mask_sda); }
static inline void scl_low(void)  { REG_WRITE(GPIO_OUT_W1TC_REG, mask_scl); }
static inline int  sda_read(void) { return (REG_READ(GPIO_IN_REG) & mask_sda) != 0; }
static inline int  scl_read(void) { return (REG_READ(GPIO_IN_REG) & mask_scl) != 0; }

static inline void dly(void)
{
    const uint32_t start = esp_cpu_get_cycle_count();
    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < half_cycles) {
    }
}

/** Solta SCL e espera ele subir (respeita clock stretching do escravo). */
static esp_err_t scl_rel(void)
{
    REG_WRITE(GPIO_OUT_W1TS_REG, mask_scl);
    if (scl_read()) {
        return ESP_OK;
    }
    for (int t = 0; t < STRETCH_TIMEOUT_US; t++) {
        esp_rom_delay_us(1);
        if (scl_read()) {
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;   // SCL preso em 0
}

static esp_err_t i2c_start(void)
{
    sda_rel();
    if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
    dly();
    if (!sda_read()) return ESP_ERR_INVALID_STATE;   // SDA presa em 0
    sda_low();
    dly();
    scl_low();
    dly();
    return ESP_OK;
}

static esp_err_t i2c_restart(void)
{
    sda_rel();
    dly();
    if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
    dly();
    sda_low();
    dly();
    scl_low();
    dly();
    return ESP_OK;
}

static void i2c_stop(void)
{
    sda_low();
    dly();
    scl_rel();
    dly();
    sda_rel();
    dly();
}

/** Envia um byte; retorna ESP_OK se o escravo deu ACK. */
static esp_err_t write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if (b & (1 << i)) sda_rel(); else sda_low();
        dly();
        if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
        dly();
        scl_low();
    }
    sda_rel();                       // libera SDA para o ACK
    dly();
    if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
    dly();
    const int nack = sda_read();
    scl_low();
    dly();
    return nack ? ESP_ERR_NOT_FOUND : ESP_OK;
}

static esp_err_t read_byte(uint8_t *b, bool ack)
{
    uint8_t v = 0;
    sda_rel();
    for (int i = 7; i >= 0; i--) {
        dly();
        if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
        dly();
        if (sda_read()) v |= (1 << i);
        scl_low();
    }
    if (ack) sda_low(); else sda_rel();
    dly();
    if (scl_rel() != ESP_OK) return ESP_ERR_TIMEOUT;
    dly();
    scl_low();
    sda_rel();
    dly();
    *b = v;
    return ESP_OK;
}

esp_err_t soft_i2c_init(int sda, int scl, uint32_t half_period_ns)
{
    if (sda < 0 || sda > 31 || scl < 0 || scl > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    mask_sda = 1UL << sda;
    mask_scl = 1UL << scl;

    const uint32_t ticks_per_us = esp_rom_get_cpu_ticks_per_us();
    half_cycles = (half_period_ns * ticks_per_us) / 1000U;
    if (half_cycles < 1) half_cycles = 1;

    gpio_config_t c = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&c);
    if (err != ESP_OK) return err;
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(100);
    soft_i2c_bus_recover();
    return ESP_OK;
}

void soft_i2c_read_lines(int *sda, int *scl)
{
    *sda = sda_read();
    *scl = scl_read();
}

void soft_i2c_bus_recover(void)
{
    sda_rel();
    for (int i = 0; i < 9; i++) {
        scl_low();
        dly();
        scl_rel();
        dly();
    }
    i2c_stop();
}

esp_err_t soft_i2c_probe(uint8_t addr)
{
    esp_err_t err = i2c_start();
    if (err == ESP_OK) err = write_byte((uint8_t)(addr << 1));
    i2c_stop();
    return err;
}

esp_err_t soft_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    esp_err_t err = i2c_start();
    if (err == ESP_OK) err = write_byte((uint8_t)(addr << 1));
    if (err == ESP_OK) err = write_byte(reg);
    if (err == ESP_OK) err = write_byte(value);
    i2c_stop();
    return err;
}

esp_err_t soft_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_start();
    if (err == ESP_OK) err = write_byte((uint8_t)(addr << 1));
    if (err == ESP_OK) err = write_byte(reg);
    if (err == ESP_OK) err = i2c_restart();
    if (err == ESP_OK) err = write_byte((uint8_t)((addr << 1) | 1));
    for (size_t i = 0; err == ESP_OK && i < len; i++) {
        err = read_byte(&data[i], i + 1 < len);   // NACK no ultimo byte
    }
    i2c_stop();
    return err;
}
