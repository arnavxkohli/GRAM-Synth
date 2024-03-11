// #include <Arduino.h>
// #include <stdint.h>
// #include <math.h>
// #include <stm32l432xx.h>
// #include <stm32l4xx_hal.h>
// #include <stm32l4xx_hal_cortex.h>
// #include <stm32l4xx_hal_dma.h>
// #include <stm32l4xx_hal_dac.h>
// #include <stm32l4xx_hal_rcc.h>
// #include <stm32l4xx_hal_gpio.h>

// // Buffer Size
// #define BUFFER_SIZE       100
// #define pi 3.14155926

// // Init variables
// DAC_HandleTypeDef hdac;

// uint32_t dacBuffer[BUFFER_SIZE];

// void setup() {

//     Serial.begin(9600);

//     HAL_Init();

//     pinMode(A3,OUTPUT);
  
//     // Initialize DAC
//     __HAL_RCC_DAC1_CLK_ENABLE();

//     hdac.Instance = DAC1;
    
//     if (HAL_DAC_Init(&hdac) != HAL_OK) {
//       // Initialization Error
//       Serial.println("Error: DAC Init");
//       while (1);
//     }
    

//     // Initialize DAC Channel
//     DAC_ChannelConfTypeDef sConfig;
//     sConfig.DAC_Trigger = DAC_TRIGGER_NONE; // DAC_TRIGGER_NONE because we're not using DMA_TRIGGER
//     sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
//     if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
//       // Channel Configuration Error
//       Serial.println("Error: DAC Channel Config");
//       while (1);
//     }

//     for(int i = 0; i< BUFFER_SIZE; i++){
//       dacBuffer[i] = (sin(i*2*pi/BUFFER_SIZE) + 1) * 4096/2 ;
//       Serial.println(dacBuffer[i]);
//     }

//     Serial.println("Setup complete!");
    
//     if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
//       // DAC Start Error
//       Serial.println("Error: DAC Start ");
//       while (1);
//     }

    


// }

// void loop() {
//   for(int i =0;i<BUFFER_SIZE;i++){
//     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacBuffer[i] >> 4);
//     delayMicroseconds(10); // Adjust delay as needed for your application
//   }
// }
