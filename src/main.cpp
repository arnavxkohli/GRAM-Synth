
#include <Arduino.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_dac.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_gpio.h>

// Buffer Size
#define BUFFER_SIZE       100

// DAC Output Buffer
uint16_t dacBuffer[BUFFER_SIZE];

// DAC Handle
DAC_HandleTypeDef hdac;
GPIO_InitTypeDef GPIO_InitStruct;

// DMA Handle
DMA_HandleTypeDef hdma_dac;

// Function prototypes
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);

void setup() {

    // Configure DAC_OUT1 pin (PA4) in analog mode
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Serial.begin(9600);

    HAL_Init();

    // Initialize DAC
    hdac.Instance = DAC1;
    __HAL_RCC_DAC1_CLK_ENABLE();
    
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
      // Initialization Error
      Serial.println("DAC Init Error");
      while (1);
    }

    // Initialize DAC Channel
    DAC_ChannelConfTypeDef sConfig;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE; // DAC_TRIGGER_NONE because we're not using DMA_TRIGGER
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
      // Channel Configuration Error
      Serial.println("DAc Config Error");
      while (1);
    }

    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_dac.Instance = DMA1_Channel3;
    hdma_dac.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac.Init.Mode = DMA_CIRCULAR;
    hdma_dac.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_dac);

    // Link DMA Handle to DAC Channel
    __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac);

    // Generate sample data
    for (int i = 0; i < BUFFER_SIZE; i++) {
      dacBuffer[i] = i * (4095 / BUFFER_SIZE); // Linear ramp from 0 to 4095
    }

    // Start DAC with DMA
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacBuffer, BUFFER_SIZE, DAC_ALIGN_12B_R) != HAL_OK) {
      // DAC Start Error
      Serial.println("DAC Transfer Error");
      while (1);
    }

}

void loop() {
    Serial.println("Transfer ongoing...");
    delay(1000);
}

