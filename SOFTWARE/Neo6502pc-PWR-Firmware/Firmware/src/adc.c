#include "adc.h"
#include "uptime.h"

static uint8_t ADC_Ready = 0;

ADC_Error_Type ADC_Channel_Init(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin, uint32_t timeoutMS) {
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	ADC_InitTypeDef ADC_InitStructure = {0};

	if (GPIO_port == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	} else if (GPIO_port == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	} else if (GPIO_port == GPIOD) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	}

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

	return ADC_Calibrate(timeoutMS);
}

ADC_Error_Type ADC_Calibrate(uint32_t timeoutMS) {
	ADC_Ready = 0;

	ADC_ResetCalibration(ADC1);

	uint32_t time = Uptime_Ms();
	while(ADC_GetResetCalibrationStatus(ADC1)) {
		if ((uint32_t)(Uptime_Ms() - time) >= timeoutMS) {
			// timeout
			return ADC_ERROR;
		}
	}

	ADC_StartCalibration(ADC1);

	time = Uptime_Ms();
	while(ADC_GetCalibrationStatus(ADC1)){
		if ((uint32_t)(Uptime_Ms() - time) >= timeoutMS) {
			// timeout
			return ADC_ERROR;
		}
	}

	ADC_Ready = 1;
	return ADC_SUCCESS;
}

ADC_Error_Type Get_ADC_Val(uint8_t ADC_Channel, uint32_t timeoutMS, uint16_t *ADC_Value) {
	ADC_Calibrate(timeoutMS);
	if (!ADC_Ready) {
		return ADC_ERROR;
	}

	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_241Cycles);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	uint32_t time = Uptime_Ms();
	while(!ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC )) {
		if ((uint32_t)(Uptime_Ms() - time) >= timeoutMS) {
			//timeout
			return ADC_ERROR;
		}
	}

	*ADC_Value = ADC_GetConversionValue(ADC1);

	return ADC_SUCCESS;
}

ADC_Error_Type Get_ADC_Average(uint8_t ADC_channel, uint32_t timeoutMS, uint8_t count, uint16_t *ADC_Value) {
    if (!ADC_Ready) {
		return ADC_ERROR;
	}

    uint32_t tmp_val = 0;
    uint16_t val;

    for(uint8_t t = 0; t < count; t++ ){
        if (Get_ADC_Val(ADC_channel, timeoutMS, &val) == ADC_ERROR) {
            return ADC_ERROR;
        }
        tmp_val += val;
        Wait_Ms(5);
    }

    *ADC_Value = tmp_val / count;

    return ADC_SUCCESS;
}

uint16_t ADC_Map(uint16_t value, uint16_t minValue, uint16_t maxValue, float minVoltage, float maxVoltage) {
	float tmp;
	tmp = (maxVoltage - minVoltage) / (maxValue - minValue);
	tmp = tmp * (value - minValue) + minVoltage;
	tmp = tmp * 1000;
	return (uint16_t) tmp;
}
