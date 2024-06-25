#include "adc.h"
#include "uptime.h"

static uint8_t ADC_Ready = 0;

ADC_Error_Type ADC_Channel_Init(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin, uint32_t timeoutMS) {
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	ADC_InitTypeDef ADC_InitStructure = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	GPIO_InitStructure.GPIO_Pin = GPIO_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_port, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);

	uint32_t t = Uptime_Ms();
	uint8_t timeout = 0;
	while(ADC_GetResetCalibrationStatus(ADC1)) {
		if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
			timeout = 1;
			break;
		}
	}
	if (timeout) {
		return ADC_ERROR;
	}

	ADC_StartCalibration(ADC1);

	t = Uptime_Ms();
	timeout = 0;
	while(ADC_GetCalibrationStatus(ADC1)){
		if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
			timeout = 1;
			break;
		}
	}

	if (timeout) {
		return ADC_ERROR;
	}

	ADC_Ready = 1;
	return ADC_SUCCESS;
}

ADC_Error_Type Get_ADC_Val(uint32_t timeoutMS, uint16_t *ADC_Value) {
	if (!ADC_Ready) {
		return ADC_ERROR;
	}
	uint32_t t = Uptime_Ms();

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_241Cycles);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	uint8_t timeout = 0;
	while(!ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC )) {
		if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
			timeout = 1;
			break;
		}
	}
	if (timeout) {
		return ADC_ERROR;
	}

	*ADC_Value = ADC_GetConversionValue(ADC1);

	return ADC_SUCCESS;
}

uint16_t ADC_Map(uint16_t value, uint16_t minValue, uint16_t maxValue, float minVoltage, float maxVoltage) {
	float tmp;
	tmp = (maxVoltage - minVoltage) / (maxValue - minValue);
	tmp = tmp * (value - minValue) + minVoltage;
	tmp = tmp * 1000;
	return (uint16_t) tmp;
}
