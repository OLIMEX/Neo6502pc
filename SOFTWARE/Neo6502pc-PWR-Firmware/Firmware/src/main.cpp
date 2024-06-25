#include <ch32v00x.h>

#include "log_debug.h"
#include "uptime.h"
#include "gpio_extender.h"

extern "C" void NMI_Handler(void)       __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();

	Uptime_Init();
	LOG_DEBUG_Configure(115200, USART_StopBits_1, USART_Parity_No);

	LOG_DEBUG(EOL "System Clock: %ld" EOL, SystemCoreClock);

	GPIO_Extender::Setup();

	while (1) {
		// GPIO_Extender::pinBlink(4, 1000);
	}

}

void NMI_Handler(void) {
	LOG_DEBUG("NMI_Handler()" EOL);
}

void HardFault_Handler(void) {
	LOG_DEBUG("HardFault_Handler()" EOL);
	while (1) {
		
	}
}
