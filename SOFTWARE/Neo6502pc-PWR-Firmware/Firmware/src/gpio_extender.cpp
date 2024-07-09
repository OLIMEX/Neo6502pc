#include "gpio_extender.h"

#include "log_debug.h"
#include "uptime.h"
#include "adc.h"

/* Extender GPIO Mapping */
typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
} ext_GPIO_Map_Type;

const ext_GPIO_Map_Type ext_GPIO_Map[EXT_GPIO_PIN_COUNT] = {
	{},                     //
	{},                     // +3.3V #1
	{},                     // GND   #2
	{GPIOA, GPIO_Pin_1},    // PA1   #3
	{GPIOC, GPIO_Pin_0},    // PC0   #4
	{GPIOA, GPIO_Pin_2},    // PA2   #5
	{GPIOC, GPIO_Pin_3},    // PC3   #6
	{GPIOD, GPIO_Pin_3},    // PD3   #7
	{GPIOC, GPIO_Pin_4},    // PC4   #8
	{GPIOD, GPIO_Pin_4},    // PD4   #9
	{GPIOC, GPIO_Pin_5},    // PC5   #10
	{GPIOD, GPIO_Pin_5},    // PD5   #11
	{GPIOC, GPIO_Pin_6},    // PC6   #12
	{GPIOD, GPIO_Pin_6},    // PD6   #13
	{GPIOC, GPIO_Pin_7},    // PC7   #14
	{GPIOD, GPIO_Pin_0},    // PD0   #15 PWR_SENS
};

// Initial GPIO extender cofiguration
// Odd pins  - input pull down
// Even pins - output push pull
ext_GPIO_Mode_Type ext_GPIO_Mode[EXT_GPIO_PIN_COUNT] = {
	ext_GPIO_Mode_OOD,    //
	ext_GPIO_Mode_IPD,    // +3.3V #1
	ext_GPIO_Mode_IPD,    // GND   #2
	ext_GPIO_Mode_IPD,    //       #3
	ext_GPIO_Mode_IPD,    //       #4
	ext_GPIO_Mode_IPD,    //       #5
	ext_GPIO_Mode_IPD,    //       #6
	ext_GPIO_Mode_IPD,    //       #7
	ext_GPIO_Mode_IPD,    //       #8
	ext_GPIO_Mode_IPD,    //       #9
	ext_GPIO_Mode_IPD,    //       #10
	ext_GPIO_Mode_IPD,    //       #11
	ext_GPIO_Mode_IPD,    //       #12
	ext_GPIO_Mode_IPD,    //       #13
	ext_GPIO_Mode_IPD,    //       #14
	ext_GPIO_Mode_IF,     //       #15 PWR_SENS
};

// Initial GPIO Extender data
uint8_t ext_GPIO_Data[EXT_GPIO_PIN_COUNT] = {
	0x00,    //
	0x01,    // +3.3V #1
	0x00,    // GND   #2
	0x00,    //       #3
	0x00,    //       #4
	0x00,    //       #5
	0x00,    //       #6
	0x00,    //       #7
	0x00,    //       #8
	0x00,    //       #9
	0x00,    //       #10
	0x00,    //       #11
	0x00,    //       #12
	0x00,    //       #13
	0x00,    //       #14
	0x00,    //       #15 PWR_SENS
};

uint16_t GPIO_Extender::Version() {
	return EXT_GPIO_REG_API_MAJOR * 256 + EXT_GPIO_REG_API_MINOR;
}

void GPIO_Extender::Setup(uint8_t addr) {
	i2c_address = addr;
	Begin();
}

void GPIO_Extender::Begin() {
	// GPIO configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	for (uint8_t pin=EXT_GPIO_PIN_FIRST; pin < EXT_GPIO_PIN_COUNT; pin++) {
		pinConfig(pin, ext_GPIO_Mode[pin], true);
	}

	// BAT_SENSE configuration
	if (ADC_Channel_Init(EXT_GPIO_BATSENS_PORT, EXT_GPIO_BATSENS_PIN, ADC_TIMEOUT_MS) == ADC_SUCCESS) {
		LOG_DEBUG("GPIO_Extender: BAT_SENSE configured" EOL);
	} else {
		LOG_DEBUG("GPIO_Extender:BAT_SENSE configuration FAILED" EOL);
	}
	
	I2C_Slave_Setup(i2c_address, i2c_registers, sizeof(i2c_registers), preRegRead, onRegRead, onRegWrite);

	LOG_DEBUG("GPIO_Extender: Initialized. Version %d.%d" EOL, EXT_GPIO_API_MAJOR, EXT_GPIO_API_MINOR);
}

void GPIO_Extender::preRegRead(uint8_t reg) {
	uint8_t selected_pin = i2c_registers[EXT_GPIO_REG_SELECT];
	switch (reg) {
		case EXT_GPIO_REG_API_MAJOR:
			i2c_registers[EXT_GPIO_REG_API_MAJOR] = EXT_GPIO_API_MAJOR;
		break;

		case EXT_GPIO_REG_API_MINOR:
			i2c_registers[EXT_GPIO_REG_API_MINOR] = EXT_GPIO_API_MINOR;
		break;

		case EXT_GPIO_REG_DATA:
			if (isPinInput(selected_pin)) {
				i2c_registers[EXT_GPIO_REG_DATA] = getPin(selected_pin);
			}
		break;

		case EXT_GPIO_REG_ALL_DATA_MSB:
			for (uint8_t pin=0x08; pin < EXT_GPIO_PIN_COUNT; pin++) {
				getPin(pin);
			}
		break;

		case EXT_GPIO_REG_ALL_DATA_LSB:
			for (uint8_t pin=EXT_GPIO_PIN_FIRST; pin < 0x08; pin++) {
				getPin(pin);
			}
		break;

		case EXT_GPIO_REG_BATSENS_PERCENT:
			getBatPercent();
		break;

		case EXT_GPIO_REG_BATSENS_MSB:
			getBatVoltage();
		break;

		case EXT_GPIO_REG_BATSENS_LSB:
			getBatVoltage();
		break;
	}
}

void GPIO_Extender::onRegRead(uint8_t reg) {
#ifdef LOD_DEBUG_ENABLE
	uint8_t selected_pin = i2c_registers[EXT_GPIO_REG_SELECT];
	switch (reg) {
		case EXT_GPIO_REG_API_MAJOR:
			LOG_DEBUG("GPIO_Extender: API version major %d" EOL, i2c_registers[EXT_GPIO_REG_API_MAJOR]);
		break;

		case EXT_GPIO_REG_API_MINOR:
			LOG_DEBUG("GPIO_Extender: API version minor %d" EOL, i2c_registers[EXT_GPIO_REG_API_MINOR]);
		break;

		case EXT_GPIO_REG_DATA:
			LOG_DEBUG("GPIO_Extender: Pin %d read data 0x%02X" EOL, selected_pin, ext_GPIO_Data[selected_pin]);
		break;

		case EXT_GPIO_REG_ALL_DATA_MSB:
			LOG_DEBUG("GPIO_Extender: All data read MSB 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_ALL_DATA_MSB]);
		break;

		case EXT_GPIO_REG_ALL_DATA_LSB:
			LOG_DEBUG("GPIO_Extender: All data read LSB 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_ALL_DATA_LSB]);
		break;

		case EXT_GPIO_REG_BATSENS_PERCENT:
			LOG_DEBUG("GPIO_Extender: BATSENS read %d%%" EOL, i2c_registers[EXT_GPIO_REG_BATSENS_PERCENT]);
		break;

		case EXT_GPIO_REG_BATSENS_MSB:
			LOG_DEBUG("GPIO_Extender: BATSENS read MSB 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_BATSENS_MSB]);
		break;

		case EXT_GPIO_REG_BATSENS_LSB:
			LOG_DEBUG("GPIO_Extender: BATSENS read LSB 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_BATSENS_LSB]);
		break;
	}
#endif
}

void GPIO_Extender::onRegWrite(uint8_t reg, uint8_t length) {
	uint8_t selected_pin = i2c_registers[EXT_GPIO_REG_SELECT];
	for (uint8_t r = reg; r < reg + length; r++) {
		switch (r) {
			case EXT_GPIO_REG_SELECT:
				LOG_DEBUG("GPIO_Extender: Pin %d selected" EOL, selected_pin);
				i2c_registers[EXT_GPIO_REG_CONFIG] = ext_GPIO_Mode[selected_pin];
				i2c_registers[EXT_GPIO_REG_DATA] = ext_GPIO_Data[selected_pin];
			break;

			case EXT_GPIO_REG_CONFIG:
				pinConfig(selected_pin, (ext_GPIO_Mode_Type)i2c_registers[EXT_GPIO_REG_CONFIG]);
			break;

			case EXT_GPIO_REG_DATA:
				LOG_DEBUG("GPIO_Extender: Set pin %d data to 0x%02X" EOL, selected_pin, i2c_registers[EXT_GPIO_REG_DATA]);
				if (isPinOutput(selected_pin)) {
					setPin(selected_pin, i2c_registers[EXT_GPIO_REG_DATA]);
				}
			break;

			case EXT_GPIO_REG_ALL_DATA_MSB:
				LOG_DEBUG("GPIO_Extender: Set all data MSB to 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_ALL_DATA_MSB]);
				for (uint8_t pin=0x08; pin < EXT_GPIO_PIN_COUNT; pin++) {
					uint8_t mask = 0x01 << (pin - 7);
					setPin(pin, mask & i2c_registers[EXT_GPIO_REG_ALL_DATA_MSB]);
				}
			break;

			case EXT_GPIO_REG_ALL_DATA_LSB:
				LOG_DEBUG("GPIO_Extender: Set all data LSB to 0x%02X" EOL, i2c_registers[EXT_GPIO_REG_ALL_DATA_LSB]);
				for (uint8_t pin=EXT_GPIO_PIN_FIRST; pin < 0x08; pin++) {
					uint8_t mask = 0x01 << pin;
					setPin(pin, mask & i2c_registers[EXT_GPIO_REG_ALL_DATA_LSB]);
				}
			break;
		}
	}
}

bool GPIO_Extender::isPinOutOfRange(uint8_t pin) {
	bool res = (pin < EXT_GPIO_PIN_FIRST || pin > EXT_GPIO_PIN_LAST || pin == EXT_GPIO_PIN_SKIP);
	if (res) {
		LOG_DEBUG("GPIO_Extender: pin [%d] is out of range" EOL, pin );
	}
	return res;
}

bool GPIO_Extender::isPinOutput(uint8_t pin) {
	if (isPinOutOfRange(pin)) {
		return false;
	}
	return ((uint8_t)ext_GPIO_Mode[pin] & 0x80) == 0;
}

bool GPIO_Extender::isPinInput(uint8_t pin) {
	if (isPinOutOfRange(pin)) {
		return false;
	}
	return ((uint8_t)ext_GPIO_Mode[pin] & 0x80) != 0;
}

void GPIO_Extender::setData(uint8_t pin, uint8_t value) {
	ext_GPIO_Data[pin] = (uint8_t)(value != 0);

	uint16_t data = i2c_registers[EXT_GPIO_REG_ALL_DATA_LSB] + 256 * i2c_registers[EXT_GPIO_REG_ALL_DATA_MSB];
	uint16_t mask = 0x0001 << pin;
	if (value != 0) {
		data |= mask;
	} else {
		data &= ~mask;
	}

	i2c_registers[EXT_GPIO_REG_ALL_DATA_LSB] = data & 0xFF;
	i2c_registers[EXT_GPIO_REG_ALL_DATA_MSB] = (data >> 8) & 0xFF;
}

void GPIO_Extender::setPin(uint8_t pin, uint8_t value) {
	if (isPinOutput(pin)) {
		setData(pin, value);
		if (value != 0) {
			GPIO_WriteBit(ext_GPIO_Map[pin].port, ext_GPIO_Map[pin].pin, BitAction::Bit_SET);
		} else {
			GPIO_WriteBit(ext_GPIO_Map[pin].port, ext_GPIO_Map[pin].pin, BitAction::Bit_RESET);
		}
	}
}

uint8_t GPIO_Extender::getPin(uint8_t pin) {
	if (isPinInput(pin)) {
		uint8_t value = GPIO_ReadInputDataBit(ext_GPIO_Map[pin].port, ext_GPIO_Map[pin].pin);
		setData(pin, value);
	}
	return ext_GPIO_Data[pin];
}

uint8_t GPIO_Extender::getBatPercent() {
	uint8_t value = 0;
	uint16_t measure = getBatVoltage(true);

	if (measure < EXT_GPIO_BATSENS_MIN) {
		value = 0;
	} else if (measure > EXT_GPIO_BATSENS_MAX) {
		value = 100;
	} else {
		value = (uint8_t)(100 * (measure - EXT_GPIO_BATSENS_MIN) / (EXT_GPIO_BATSENS_MAX - EXT_GPIO_BATSENS_MIN));
	}

	LOG_DEBUG("GPIO_Extender: BATSENS %d%%" EOL, value);
	i2c_registers[EXT_GPIO_REG_BATSENS_PERCENT] = value;

	return value;
}

uint16_t GPIO_Extender::getBatVoltage(bool average, uint8_t count) {
	uint16_t value = 0;

	if (getPin(EXT_GPIO_PIN_PWRSENS)) {
		// When external power is present measuring battery voltage is nonsense
		LOG_DEBUG("GPIO_Extender: On external power" EOL);
	} else {
		if (
			(average ?
				Get_ADC_Average(EXT_GPIO_BATSENS_CHANNEL, ADC_TIMEOUT_MS, count, &value)
				:
				Get_ADC_Val(EXT_GPIO_BATSENS_CHANNEL, ADC_TIMEOUT_MS, &value)
			) == ADC_SUCCESS
		) {
			// Map value to mV
			value = ADC_Map(value, 0, 1023, 0.0, 3.3);
		} else {
			LOG_DEBUG("GPIO_Extender: ADC error" EOL);
			value = 0;
		}
	}

	i2c_registers[EXT_GPIO_REG_BATSENS_MSB] = (uint8_t)((value >> 8) & 0x00FF);
	i2c_registers[EXT_GPIO_REG_BATSENS_LSB] = (uint8_t)(value & 0x00FF);
	LOG_DEBUG("GPIO_Extender: BATSENS %d mV" EOL, value);

	return value;
}

void GPIO_Extender::pinConfig(uint8_t pin, ext_GPIO_Mode_Type mode, bool initial) {
	LOG_DEBUG("GPIO_Extender::pinConfig(%d, 0x%02X)" EOL, pin, (uint8_t)mode);
	if (isPinOutOfRange(pin)) {
		return;
	}

	if (pin == EXT_GPIO_PIN_PWRSENS && !initial) {
		LOG_DEBUG("GPIO_Extender: PWRSENSE pin [%d] can not be reconfigured" EOL, pin );
		return;
	}
	
	ext_GPIO_Mode[pin] = mode;

	GPIO_InitTypeDef cfg = {0};
	
	switch (mode) {
		case ext_GPIO_Mode_OOD:
			cfg.GPIO_Mode = GPIO_Mode_Out_OD;
		break;

		case ext_GPIO_Mode_OPP:
			cfg.GPIO_Mode = GPIO_Mode_Out_PP;
		break;

		case ext_GPIO_Mode_IF:
			cfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		break;

		case ext_GPIO_Mode_IPD:
			cfg.GPIO_Mode = GPIO_Mode_IPD;
		break;

		case ext_GPIO_Mode_IPU:
			cfg.GPIO_Mode = GPIO_Mode_IPU;
		break;
	}

	cfg.GPIO_Pin = ext_GPIO_Map[pin].pin;
	cfg.GPIO_Speed = GPIO_Speed_10MHz;

	GPIO_Init(ext_GPIO_Map[pin].port, &cfg);

	setPin(pin, ext_GPIO_Data[pin]);
}

void GPIO_Extender::pinBlink(uint8_t pin, int time_ms) {
	if (isPinOutOfRange(pin)) {
		return;
	}

	GPIO_WriteBit(ext_GPIO_Map[pin].port, ext_GPIO_Map[pin].pin, BitAction::Bit_SET);
	Wait_Ms(time_ms);
	GPIO_WriteBit(ext_GPIO_Map[pin].port, ext_GPIO_Map[pin].pin, BitAction::Bit_RESET);
	Wait_Ms(time_ms);
}
