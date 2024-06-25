#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H

#include <ch32v00x.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void);

void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void);


typedef void (*i2c_write_callback_t)(uint8_t reg, uint8_t length);
typedef void (*i2c_read_callback_t)(uint8_t reg);

void I2C_Slave_Setup(uint8_t addr, volatile uint8_t* regs, uint8_t regs_cnt, i2c_read_callback_t pre_read, i2c_read_callback_t on_read, i2c_write_callback_t on_write);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */
