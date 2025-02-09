
#include "DRIVERS/CK_ADC.h"
#include "DRIVERS/CK_GPIO.h"

typedef enum{
	START_CONVERSION,
	WAIT_CONVERSION,

}adc_e;

typedef enum{
	LIPO_CONVERSION,
	CURRENT_CONVERSION,
	TEMPERATURE_CONVERSION,

}adc_conversion_e;

typedef struct{

	float lipo_adc_result;
	float current_adc_result;
	uint32_t temperature_adc_result;

	float lipo_result;
	float current_result;
	float temp_result;

	adc_e state;

	// current and temp uses same adc1 so their reading must be ordered
	// because first conversion is for current and second is for temp
	adc_conversion_e adc1_conversion;
	// adc2 now does not need this but in the future it can be implemented like adc1
	adc_conversion_e adc2_conversion;

	bool is_temp_reading_ready;
	bool is_lipo_reading_ready;
	bool is_current_reading_ready;

	bool is_data_ready; // for other libraries to check before reading data

}adc_s;

adc_s adc;

// ADC handle declaration
ADC_HandleTypeDef temp_sensor;
ADC_HandleTypeDef lipo_adc;
ADC_HandleTypeDef current_adc;

int is_done1 = 0;
int is_done2 = 0;

void CK_ADC_Init(void){

	adc.lipo_adc_result = 0.0f;
	adc.current_adc_result = 0.0f;
	adc.temperature_adc_result = 0;

	adc.lipo_result = 0.0f;
	adc.current_result = 0.0f;
	adc.temp_result = 0.0f;

	adc.state = START_CONVERSION;
	adc.adc1_conversion = CURRENT_CONVERSION; // assigned the first channel reading
	adc.adc2_conversion = LIPO_CONVERSION;

	adc.is_current_reading_ready = false;
	adc.is_lipo_reading_ready = false;
	adc.is_temp_reading_ready = false;
	adc.is_data_ready = false;

	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	PeriphClkInitStruct.PeriphClockSelection 	= RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M 				= 1;
	PeriphClkInitStruct.PLL2.PLL2N 				= 18;
	PeriphClkInitStruct.PLL2.PLL2P 				= 1;
	PeriphClkInitStruct.PLL2.PLL2Q 				= 2;
	PeriphClkInitStruct.PLL2.PLL2R 				= 2;
	PeriphClkInitStruct.PLL2.PLL2RGE 			= RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL 		= RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN		 	= 6144;
	PeriphClkInitStruct.AdcClockSelection 		= RCC_ADCCLKSOURCE_PLL2;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	__HAL_RCC_ADC12_CLK_ENABLE();
	__HAL_RCC_ADC3_CLK_ENABLE();

	CK_ADC_ADC1Init();
	CK_ADC_ADC2Init();
	CK_ADC_ADC3Init();

	HAL_NVIC_EnableIRQ(ADC_IRQn);

}

void CK_ADC_ADC1Init(void){

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	lipo_adc.Instance 						= ADC1;
	lipo_adc.Init.ClockPrescaler 			= ADC_CLOCK_ASYNC_DIV1;
	lipo_adc.Init.Resolution 				= ADC_RESOLUTION_16B;
	lipo_adc.Init.ScanConvMode 				= ADC_SCAN_DISABLE;
	lipo_adc.Init.EOCSelection 				= ADC_EOC_SINGLE_CONV;
	lipo_adc.Init.LowPowerAutoWait 			= DISABLE;
	lipo_adc.Init.ContinuousConvMode 		= DISABLE;
	lipo_adc.Init.NbrOfConversion 			= 1;
	lipo_adc.Init.DiscontinuousConvMode 	= DISABLE;
	lipo_adc.Init.ExternalTrigConv 			= ADC_SOFTWARE_START;
	lipo_adc.Init.ExternalTrigConvEdge 		= ADC_EXTERNALTRIGCONVEDGE_NONE;
	lipo_adc.Init.ConversionDataManagement 	= ADC_CONVERSIONDATA_DR;
	lipo_adc.Init.Overrun 					= ADC_OVR_DATA_PRESERVED;
	lipo_adc.Init.LeftBitShift 				= ADC_LEFTBITSHIFT_NONE;
	lipo_adc.Init.OversamplingMode 			= DISABLE;
	HAL_ADC_Init(&lipo_adc);

	multimode.Mode = ADC_MODE_INDEPENDENT;

	HAL_ADCEx_Calibration_Start(&lipo_adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	HAL_ADCEx_MultiModeConfigChannel(&lipo_adc, &multimode);

	sConfig.Channel 		= ADC_CHANNEL_10;
	sConfig.Rank 			= ADC_REGULAR_RANK_1;
	sConfig.SamplingTime 	= ADC_SAMPLETIME_810CYCLES_5; // 12microsec conversion time
	sConfig.SingleDiff 		= ADC_SINGLE_ENDED;
	sConfig.OffsetNumber 	= ADC_OFFSET_NONE;
	sConfig.Offset 			= 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	HAL_ADC_ConfigChannel(&lipo_adc, &sConfig);

}

void CK_ADC_ADC2Init(void){

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	current_adc.Instance 						= ADC2;
	current_adc.Init.ClockPrescaler 			= ADC_CLOCK_ASYNC_DIV1;
	current_adc.Init.Resolution 				= ADC_RESOLUTION_16B;
	current_adc.Init.ScanConvMode 				= ADC_SCAN_DISABLE;
	current_adc.Init.EOCSelection 				= ADC_EOC_SINGLE_CONV;
	current_adc.Init.LowPowerAutoWait 			= DISABLE;
	current_adc.Init.ContinuousConvMode 		= DISABLE;
	current_adc.Init.NbrOfConversion 			= 1;
	current_adc.Init.DiscontinuousConvMode 		= DISABLE;
	current_adc.Init.ExternalTrigConv 			= ADC_SOFTWARE_START;
	current_adc.Init.ExternalTrigConvEdge 		= ADC_EXTERNALTRIGCONVEDGE_NONE;
	current_adc.Init.ConversionDataManagement 	= ADC_CONVERSIONDATA_DR;
	current_adc.Init.Overrun 					= ADC_OVR_DATA_PRESERVED;
	current_adc.Init.LeftBitShift 				= ADC_LEFTBITSHIFT_NONE;
	current_adc.Init.OversamplingMode 			= DISABLE;
	HAL_ADC_Init(&current_adc);

	multimode.Mode = ADC_MODE_INDEPENDENT;

	//HAL_ADCEx_Calibration_Start(&current_adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	HAL_ADCEx_MultiModeConfigChannel(&current_adc, &multimode);

	sConfig.Channel 		= ADC_CHANNEL_11;
	sConfig.Rank 			= ADC_REGULAR_RANK_1;
	sConfig.SamplingTime 	= ADC_SAMPLETIME_810CYCLES_5;
	sConfig.SingleDiff 		= ADC_SINGLE_ENDED;
	sConfig.OffsetNumber 	= ADC_OFFSET_NONE;
	sConfig.Offset 			= 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	HAL_ADC_ConfigChannel(&current_adc, &sConfig);

}

void CK_ADC_ADC3Init(void){

	temp_sensor.Instance = ADC3;

	HAL_ADC_DeInit(&temp_sensor);

	temp_sensor.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;          /* Asynchronous clock mode, input ADC clock divided by 2*/
	temp_sensor.Init.Resolution               = ADC_RESOLUTION_16B;            /* 16-bit resolution for converted data */
	temp_sensor.Init.ScanConvMode             = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	temp_sensor.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
	temp_sensor.Init.LowPowerAutoWait         = DISABLE;                       /* Auto-delayed conversion feature disabled */
	temp_sensor.Init.ContinuousConvMode       = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
	temp_sensor.Init.NbrOfConversion          = 1;                             /* Parameter discarded because sequencer is disabled */
	temp_sensor.Init.DiscontinuousConvMode    = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	temp_sensor.Init.NbrOfDiscConversion      = 1;                             /* Parameter discarded because sequencer is disabled */
	temp_sensor.Init.ExternalTrigConv         = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	temp_sensor.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	temp_sensor.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;         /* DR register used as output (DMA mode disabled) */
	temp_sensor.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;         /* Left shift of final results */
	temp_sensor.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	temp_sensor.Init.OversamplingMode         = DISABLE;                       /* Oversampling disable */

	// Initialize ADC peripheral according to the passed parameters
	HAL_ADC_Init(&temp_sensor);

	// Start calibration
	HAL_ADCEx_Calibration_Start(&temp_sensor, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	// ADC channel configuration structure declaration
	ADC_ChannelConfTypeDef sConfig;

	// Channel configuration
	sConfig.Channel                = ADC_CHANNEL_TEMPSENSOR;     /* Sampled channel number */
	sConfig.Rank                   = ADC_REGULAR_RANK_1;         /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5;  /* Sampling time (number of clock cycles unit) */
	sConfig.SingleDiff             = ADC_SINGLE_ENDED;           /* Single input channel */
	sConfig.OffsetNumber           = ADC_OFFSET_NONE;            /* No offset subtraction */
	sConfig.Offset                 = 0;                          /* Parameter discarded because offset correction is disabled */
	sConfig.OffsetRightShift       = DISABLE;                    /* No Right Offset Shift */
	sConfig.OffsetSignedSaturation = DISABLE;                    /* No Signed Saturation */
	HAL_ADC_ConfigChannel(&temp_sensor, &sConfig);

}

void CK_ADC_Update(void){

	if(adc.state == START_CONVERSION){

		// These flags are used internally do not take them out with a isReady function
		adc.is_current_reading_ready = false;
		adc.is_lipo_reading_ready = false;
		adc.is_temp_reading_ready = false; // not used in h7 yet

		adc.is_data_ready = false;

		CK_ADC_StartConversion();

		adc.state = WAIT_CONVERSION;
	}
	else if(adc.state == WAIT_CONVERSION){

		// Are all readings ready
		if(adc.is_current_reading_ready && adc.is_lipo_reading_ready){
			adc.lipo_result = 3.3f * (adc.lipo_adc_result / powf(2, ADC_BITS));
			adc.current_result = 3.3f * (adc.current_adc_result / powf(2, ADC_BITS));
			CK_ADC_CalculateTemperature();

			adc.is_data_ready = true;
			adc.state = START_CONVERSION;
		}
	}
}

bool isDataReady(void){
	return adc.is_data_ready;
}

float CK_ADC_GetLipoResult(void){
	return adc.lipo_result;
}

float CK_ADC_GetCurrentResult(void){
	return adc.current_result ;
}

float CK_ADC_GetTemperatureResult(void){
	return adc.temp_result;
}

void CK_ADC_StartConversion(void){

#if SCOPE_CHECK_ADC
		CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
#endif

	HAL_ADC_Start_IT(&lipo_adc);

	HAL_ADC_Start_IT(&current_adc);

	HAL_ADC_Start(&temp_sensor); // Interrupt is not working i manually read data

}

void CK_ADC_CalculateTemperature(void){

	uint16_t ts_cal1 = *(uint16_t*) (0x1FF1E820);
	uint16_t ts_cal2 = *(uint16_t*) (0x1FF1E840);

	adc.temperature_adc_result = HAL_ADC_GetValue(&temp_sensor);

	adc.temp_result = (80 * (adc.temperature_adc_result - ts_cal1)) / (ts_cal2 - ts_cal1) + 30;
}

void ADC_IRQHandler(void){

	// Regular channel end of conversion
	if((ADC1->ISR & (1u << 2))){

		HAL_ADC_IRQHandler(&lipo_adc);

		adc.lipo_adc_result = (float)HAL_ADC_GetValue(&lipo_adc);

		adc.is_lipo_reading_ready = true;

#if SCOPE_CHECK_ADC
		CK_GPIO_ClearPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
#endif

	}

	// Regular channel end of conversion
	if((ADC2->ISR & (1u << 2))){

		HAL_ADC_IRQHandler(&current_adc);

		adc.current_adc_result = (float)HAL_ADC_GetValue(&current_adc);

		adc.is_current_reading_ready = true;
	}

}

