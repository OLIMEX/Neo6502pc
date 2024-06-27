/* 
 * Code for using the I2C peripheral in slave mode 
 * for OLIMEX Neo6502PC-PWR board
 * 
 * Project is based on https://github.com/cnlohr/ch32v003fun/blob/master/examples/i2c_slave/
 * 
 * MIT License
 * 
 * Copyright (c) 2024 Renze Nicolai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "i2c_slave.h"

// Disable LOG_DEBUG locally
#undef LOD_DEBUG_ENABLE
#include "log_debug.h"

static uint8_t _address = 0x00;
static uint8_t _first_write = 1;
static uint8_t _offset = 0;
static uint8_t _position = 0;
static uint8_t _writing = 0;

static volatile uint8_t* volatile _registers = NULL;
static uint8_t _registers_count = 0;

static i2c_read_callback_t _pre_read_callback = NULL;
static i2c_read_callback_t _read_callback = NULL;
static i2c_write_callback_t _write_callback = NULL;

void I2C_Slave_Begin();

void I2C_Slave_Setup(uint8_t addr, volatile uint8_t* regs, uint8_t regs_cnt, i2c_read_callback_t pre_read, i2c_read_callback_t on_read, i2c_write_callback_t on_write) {
	_address = addr;
	_first_write = 1;
	_offset = 0;
	_position = 0;

	_registers = regs;
	_registers_count = regs_cnt;

	_pre_read_callback = pre_read;
	_read_callback = on_read;
	_write_callback = on_write;

	I2C_Slave_Begin();
}

void I2C_Slave_Begin() {
    // SDA, SCL - Open-drain multiplexed output
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_PortC = {0};
	GPIO_PortC.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_PortC.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_PortC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_PortC);

	// Enable I2C1
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Reset I2C1 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	I2C1->CTLR1 |= I2C_CTLR1_SWRST;
	I2C1->CTLR1 &= ~I2C_CTLR1_SWRST;

	// Set module clock frequency
	uint32_t prerate = 2000000; // I2C Logic clock rate, must be higher than the bus clock rate
	I2C1->CTLR2 |= (SystemCoreClock / prerate) & I2C_CTLR2_FREQ;

	// Enable interrupts
	I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN;
	
	NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
	NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
	
	NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
	NVIC_SetPriority(I2C1_ER_IRQn, 2 << 4);

	// Set clock configuration
	uint32_t clockrate = 1000000; // I2C Bus clock rate, must be lower than the logic clock rate
	I2C1->CKCFGR = ((SystemCoreClock / (3 * clockrate)) & I2C_CKCFGR_CCR) | I2C_CKCFGR_FS; // Fast mode 33% duty cycle
	//I2C1->CKCFGR = ((SystemCoreClock/ (25 * clockrate)) & I2C_CKCFGR_CCR) | I2C_CKCFGR_DUTY | I2C_CKCFGR_FS; // Fast mode 36% duty cycle
	//I2C1->CKCFGR = (SystemCoreClock / (2 * clockrate)) & I2C_CKCFGR_CCR; // Standard mode good to 100kHz
	
	// Set I2C _address
	I2C1->OADDR1 = _address << 1;
	I2C1->OADDR2 = 0;

	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Acknowledge bytes when they are received
	I2C1->CTLR1 |= I2C_CTLR1_ACK;

	// Disable clock stretch
	// I2C1->CTLR1 |= I2C_CTLR1_NOSTRETCH;

	LOG_DEBUG("I2C_Slave: Initialized @ 0x%02X" EOL, _address);
}

void I2C_Slave_Start_Event() {
	// LOG_DEBUG("I2C_Slave: Start" EOL);
	_first_write = 1;    // Next write will be the _offset
	_position = _offset; // Reset _position
}

void I2C_Slave_Write_Event() {
	// LOG_DEBUG("I2C_Slave: Write" EOL);
	if (_first_write) {
		// First byte written, set the _offset
		_offset = I2C1->DATAR;
		_position = _offset;
		_first_write = 0;
		_writing = 0;
		LOG_DEBUG("I2C_Slave: Register 0x%02X" EOL, _offset);
	} else {
		// Normal register write
		_writing = 1;

		uint8_t data = I2C1->DATAR;
		LOG_DEBUG("I2C_Slave: Write 0x%02X" EOL, data);

		if (_position < _registers_count) {
			_registers[_position] = data;
			_position++;
		}
	}
}

void I2C_Slave_Read_Event() {
	// LOG_DEBUG("I2C_Slave: Read" EOL);
	_writing = 0;
	if (_position < _registers_count) {
		if (_pre_read_callback != NULL) {
			_pre_read_callback(_position);
		}

		I2C1->DATAR = _registers[_position];
		LOG_DEBUG("I2C_Slave: Read 0x%02X" EOL, I2C1->DATAR);
		
		if (_read_callback != NULL) {
			_read_callback(_position);
		}
		_position++;
	} else {
		LOG_DEBUG("I2C_Slave: Register 0x%02X out of range" EOL, _position);
		I2C1->DATAR = 0x00;
		LOG_DEBUG("I2C_Slave: Read 0x%02X" EOL, I2C1->DATAR);
	}
}

void I2C_Slave_Stop_Event() {
	LOG_DEBUG("I2C_Slave: Stop" EOL);
	if (_write_callback != NULL) {
		_write_callback(_offset, _position - _offset);
	}
}

void I2C1_EV_IRQHandler(void) {
	// LOG_DEBUG("I2C_Slave: Event" EOL);

	uint16_t STAR1, STAR2 __attribute__((unused));
	STAR1 = I2C1->STAR1;
	STAR2 = I2C1->STAR2;

	if (STAR1 & I2C_STAR1_ADDR) { 
		// Start event
		I2C_Slave_Start_Event();
	}

	if (STAR1 & I2C_STAR1_RXNE) { 
		// Write event
		I2C_Slave_Write_Event();
	}

	if (STAR1 & I2C_STAR1_TXE) { 
		// Read event
		I2C_Slave_Read_Event();
	}

	if (STAR1 & I2C_STAR1_STOPF) { 
		// Stop event
		// Clear stop
		I2C1->CTLR1 &= ~(I2C_CTLR1_STOP);

		I2C_Slave_Stop_Event();
	}
}

void I2C1_ER_IRQHandler(void) {
	LOG_DEBUG("I2C_Slave: ");

	uint16_t STAR1 = I2C1->STAR1;

	if (STAR1 & I2C_STAR1_BERR) { 
		// Bus error
		LOG_DEBUG("Bus error" EOL);

		// Clear error
		I2C1->STAR1 &= ~(I2C_STAR1_BERR); 
	}

	if (STAR1 & I2C_STAR1_ARLO) { 
		// Arbitration lost error
		LOG_DEBUG("Arbitration lost error" EOL);

		// Clear error
		I2C1->STAR1 &= ~(I2C_STAR1_ARLO); 
	}

	if (STAR1 & I2C_STAR1_AF) { 
		if (_writing) {
			// Acknowledge failure
			LOG_DEBUG("Acknowledge failure" EOL);
		} else {
			LOG_DEBUG("NAC" EOL);
		}

		// Clear error
		I2C1->STAR1 &= ~(I2C_STAR1_AF); 
	}
}
