#include "Dac.h"

uint32_t DAC_Init(DAC_HandleTypeDef hdac){
  // Initialize DAC Channel
  HAL_Init();

  __HAL_RCC_DAC1_CLK_ENABLE();

  hdac.Instance = DAC1;
    
  return (uint32_t) HAL_DAC_Init(&hdac);
  
}

uint32_t DAC_Config(DAC_HandleTypeDef hdac){
  //Config DAC channel
  DAC_ChannelConfTypeDef sConfig;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE; 
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  return (uint32_t) HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
  
}

uint32_t DAC_Start(DAC_HandleTypeDef hdac){
    // Start the DAC
    return (uint32_t) HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

uint32_t DACSetValue(DAC_HandleTypeDef hdac,uint32_t Data){
  // Send value to the DAC
  return (uint32_t) HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Data);
}



  

  