#ifndef	__ADC_H
#define	__ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch32v00x.h>

typedef enum {
	ADC_SUCCESS = 0,
	ADC_ERROR = 1
} ADC_Error_Type;

#define ADC_TIMEOUT_MS    100

ADC_Error_Type ADC_Channel_Init(GPIO_TypeDef *port, uint16_t pin, uint32_t timeoutMS);
ADC_Error_Type Get_ADC_Val(uint32_t timeoutMS, uint16_t *ADC_Value);
uint16_t ADC_Map(uint16_t value, uint16_t minValue, uint16_t maxValue, float minVoltage, float maxVoltage);

#ifdef __cplusplus
}
#endif


#endif /* __ADC_H */
