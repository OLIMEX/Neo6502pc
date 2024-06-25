#ifndef __GPIO_EXTENDER_H
#define __GPIO_EXTENDER_H

#include <ch32v00x.h>

#include "i2c_slave.h"

#define EXT_GPIO_PIN_FIRST   3
#define EXT_GPIO_PIN_LAST    15
#define EXT_GPIO_PIN_PWRSENS 15
#define EXT_GPIO_PIN_COUNT   16

#ifdef LOD_DEBUG_ENABLE
#define EXT_GPIO_PIN_SKIP 11
#else
#define EXT_GPIO_PIN_SKIP -1
#endif

#define EXT_GPIO_I2C_ADDR            0x33

#define EXT_GPIO_API_MAJOR           1
#define EXT_GPIO_API_MINOR           0

#define EXT_GPIO_REG_API_MAJOR       0x00
#define EXT_GPIO_REG_API_MINOR       0x01
#define EXT_GPIO_REG_SELECT          0x02
#define EXT_GPIO_REG_CONFIG          0x03
#define EXT_GPIO_REG_DATA            0x04
#define EXT_GPIO_REG_ALL_DATA_MSB    0x05
#define EXT_GPIO_REG_ALL_DATA_LSB    0x06
#define EXT_GPIO_REG_BATSENS_PERCENT 0x07
#define EXT_GPIO_REG_BATSENS_MSB     0x08
#define EXT_GPIO_REG_BATSENS_LSB     0x09
#define EXT_GPIO_REG_COUNT           0x0A

// BATSENS = 1000 * R14 * VBAT / (R14 + R13)
// VBAT range 4.2V - 3.2V
#define EXT_GPIO_BATSENS_MIN         870
#define EXT_GPIO_BATSENS_MAX         1140


/* Extender GPIO Mode enumeration */
typedef enum {
	ext_GPIO_Mode_OOD = 0x00,
	ext_GPIO_Mode_OPP = 0x01,

	ext_GPIO_Mode_IF  = 0x80,
	ext_GPIO_Mode_IPD = 0x81,
	ext_GPIO_Mode_IPU = 0x82,
} ext_GPIO_Mode_Type;

class GPIO_Extender {
protected:
	inline static uint8_t i2c_address = 0x33;

	/* 
		Registers

		0x00 - API version (major)
		0x01 - API version (minor)

		0x02 - pin select [3-14]

		0x03 - pin config
			0x00 - out - OD
			0x01 - out - PP 
			
			0x80 - in - float
			0x81 - in - PD
			0x82 - in - PU
		
		0x04 - data read / write

		0x05, 0x06 - Pins as bits

		0x07 - BATSENS percent

		0x08, 0x09 - BATSENS mV
	*/
	inline static volatile uint8_t i2c_registers[EXT_GPIO_REG_COUNT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

protected:
	static void Begin();
	static void setData(uint8_t pin, uint8_t value);
	static void setPin(uint8_t pin, uint8_t value);
	static uint8_t getPin(uint8_t pin);
	static uint8_t getBatPercent();
	static uint16_t getBatVoltage();
public:
	static uint16_t Version();

	static void Setup(uint8_t addr = EXT_GPIO_I2C_ADDR);

	static void preRegRead(uint8_t reg);
	static void onRegRead(uint8_t reg);
	static void onRegWrite(uint8_t reg, uint8_t length);
	
	static bool isPinOutOfRange(uint8_t pin);
	static bool isPinOutput(uint8_t pin);
	static bool isPinInput(uint8_t pin);

	static void pinConfig(uint8_t pin, ext_GPIO_Mode_Type mode, bool initial = false);
	static void pinBlink(uint8_t pin, int time_ms);

	GPIO_Extender() = delete;
};

#endif /* __GPIO_EXTENDER_H */
