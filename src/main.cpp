#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <stm32l432xx.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_dma.h>
#include <stm32l4xx_hal_dac.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_gpio.h>

// Buffer Size
#define BUFFER_SIZE       100
#define pi 3.14155926

// Init variables
DAC_HandleTypeDef hdac;
GPIO_InitTypeDef GPIO_InitStruct;
DMA_HandleTypeDef hdma_dac;

uint32_t dacBuffer[BUFFER_SIZE];

void setup() {

    Serial.begin(9600);

    HAL_Init();

    pinMode(A3,OUTPUT);
    

    __HAL_RCC_DMA1_CLK_ENABLE();
  
    hdma_dac.Instance = DMA2_Channel4;
    hdma_dac.Init.Request = DMA_REQUEST_3;
    hdma_dac.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac.Init.Mode = DMA_CIRCULAR;
    hdma_dac.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_dac) != HAL_OK) {
      // DMA Initialization Error
      Serial.println("Error: DMA Init");
      while (1);
    }

    // HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    // Initialize DAC
    __HAL_RCC_DAC1_CLK_ENABLE();

    hdac.Instance = DAC1;
    
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
      // Initialization Error
      Serial.println("Error: DAC Init");
      while (1);
    }
    

    // Initialize DAC Channel
    DAC_ChannelConfTypeDef sConfig;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE; // DAC_TRIGGER_NONE because we're not using DMA_TRIGGER
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
      // Channel Configuration Error
      Serial.println("Error: DAC Channel Config");
      while (1);
    }

    for(int i = 0; i< BUFFER_SIZE; i++){
      dacBuffer[i] = (sin(i*2*pi/BUFFER_SIZE) + 1) * 4096/2 ;
      Serial.println(dacBuffer[i]);
    }

    // Link DMA Handle to DAC Channel
    __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac);

    Serial.println("Setup complete!");
    
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
      // DAC Start Error
      Serial.println("Error: DAC Start ");
      while (1);
    }

    // Start DAC with DMA
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacBuffer, BUFFER_SIZE, DAC_ALIGN_12B_R) != HAL_OK) {
      // DAC Start Error
      Serial.println("Error: DAC Start DMA");
      while (1);
    }


}

void loop() {
  // for(int i =0;i<BUFFER_SIZE;i++){
  //   HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacBuffer[i]);
  //   delayMicroseconds(10); // Adjust delay as needed for your application
  // }
}
