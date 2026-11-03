/*
 * x6100-bbfw -- shared I2C master driver (i2c.md).
 * I2C1 = PB6 SCL / PB7 SDA, I2C2 = PB10 SCL / PB11 SDA, both AF4, 100 kHz,
 * blocking/polling. Combined write-register / repeated-START / read-N, raw
 * write (START/addr/payload/STOP), address-only ACK probe, and the I2C1
 * 10-clock stuck-bus recovery.
 */
#ifndef X6100_I2C_H
#define X6100_I2C_H

#include <stdint.h>
#include "stm32f4xx_ll_i2c.h"

void i2c_pins_init(void);                     /* PB6/7 (I2C1) + PB10/11 (I2C2) */
void i2c_master_init(I2C_TypeDef *i2c);       /* 100 kHz standard mode         */

/* Address-only ACK probe (non-mutating): 1 = ACK, 0 = NAK, -1 = timeout. */
int i2c_probe(I2C_TypeDef *i2c, uint8_t addr7);

/* Combined write-register / repeated-START / read-N: 0 = ok, -1 = error. */
int i2c_read(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t n);

/* Raw write (START, addr+W, payload bytes, STOP): 0 = ok, -1 = error.
 * Matches FUN_08031890 -- no implicit register byte is inserted. */
int i2c_write_raw(I2C_TypeDef *i2c, uint8_t addr7, const uint8_t *buf, uint32_t n);

/* I2C1 stuck-bus recovery (i2c.md FUN_08031cfc); prints "i2c reseted". */
void i2c1_bus_recover(void);

#endif /* X6100_I2C_H */
