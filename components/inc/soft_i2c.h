/**
 * @file soft_i2c.h
 * @brief I2C mestre por software (bit-bang) em dois GPIOs quaisquer.
 *
 * Nao usa o periferico I2C do chip: SDA e SCL sao controlados diretamente
 * como saidas open-drain com pull-up, escrevendo direto nos registradores.
 * A velocidade e definida pelo meio periodo do clock, em nanossegundos
 * (1250 ns ~ 400 kHz, 5000 ns ~ 100 kHz).
 */
#ifndef SOFT_I2C_H_
#define SOFT_I2C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/** Configura os pinos e libera o barramento (inclui recuperacao de travamento). */
esp_err_t soft_i2c_init(int sda_pin, int scl_pin, uint32_t half_period_ns);

/** Le o nivel atual das linhas (1 = alto). */
void soft_i2c_read_lines(int *sda, int *scl);

/** Envia 9 pulsos de clock + STOP para soltar um escravo travado. */
void soft_i2c_bus_recover(void);

/** ESP_OK se algum dispositivo responder (ACK) no endereco de 7 bits. */
esp_err_t soft_i2c_probe(uint8_t addr);

/** Escreve um byte em um registrador. */
esp_err_t soft_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/** Le 'len' bytes a partir do registrador 'reg' (com repeated START). */
esp_err_t soft_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

#endif // SOFT_I2C_H_
