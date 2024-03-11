#ifndef DAC_HPP
#define DAC_HPP

#include <stm32l432xx.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_dac.h>
#include <stm32l4xx_hal_rcc.h>

//Initialise the DAC module
uint32_t DAC_Init(DAC_HandleTypeDef hdac);

//Configure DAC module
uint32_t DAC_Config(DAC_HandleTypeDef hdac);

//Enable the DAC module
uint32_t DAC_Start(DAC_HandleTypeDef hdac);

// Transmit the value to the DAC
uint32_t DACSetValue(DAC_HandleTypeDef hdac,uint32_t Data);

#endif