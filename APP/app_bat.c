#include "app_bat.h"
#include "app_motion.h"
#include "bsp_usart.h"

#include <string.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;


void adc_conv()
{

	uint16_t value;
	float out;
	char sbuff[100];

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//printf("adc value = %d\n", value);
	// calculateor
	// v * (7. 5 / (30 + 7.5)) = value / 4096 * 3.3V
	// v = value / 4096 * 3.3V / (0.2)
	// v = value / 4096 * 3.3 / 5

	out = (float) value* 1.0;
	out = ( out /  4096.0 ) * 3.3 * 5;

	sprintf(sbuff,"{\"batt\": %d }\n", (int)( out * 100)  );
	USART1_Send_ArrayU8_DMA( (uint8_t *) sbuff, strlen(sbuff));


}



