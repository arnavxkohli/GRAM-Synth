#include "Globals.h"
#include "Knob.h"
#include "waveform.h"
#include <Arduino.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>
#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stm32l432xx.h>
#include <stm32l4xx_hal_gpio.h>
#include <string>

#define SAMPLE_BUFFER_SIZE 1024

#define RECEIVER
#define TRANSMITTER

#ifdef TRANSMITTER
bool transmitter = true;
#else
bool transmitter = false;
#endif

#ifdef RECEIVER
bool receiver = true;
#else
bool receiver = false;
#endif

// DAC_HandleTypeDef hdac;
struct {
  int tone_idx[6];
  int nok;
  SemaphoreHandle_t mutex;
} protectedGlobals;

struct {
  uint32_t sampleBuffer0[SAMPLE_BUFFER_SIZE / 2];
  uint32_t sampleBuffer1[SAMPLE_BUFFER_SIZE / 2];
  volatile bool writeBuffer1 = false;
  SemaphoreHandle_t doubleBufferSemaphore;
} doubleBuffer;

Knob volumeKnob(8, 0, 1);
Knob dampKnob(2, 0, 1);
Knob instrumentKnob(2, 0, 1);
Knob octaveKnob(4, 2, 1);

QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

SemaphoreHandle_t CAN_TX_Semaphore;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN

  delayMicroseconds(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[bitIdx & 0x01]);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[bitIdx & 0x02]);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[bitIdx & 0x04]);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, gpio_state[value]); // OUT_PIN
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);      // REN_PIN
  delayMicroseconds(2);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
}

void setRow(uint8_t rowidx) {
  std::bitset<3> Ridx = std::bitset<3>(rowidx);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
  delayMicroseconds(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[Ridx[0]]); // RA0_PIN
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[Ridx[1]]); // RA1_PIN
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[Ridx[2]]); // RA2_PIN
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);        // REN_PIN
  delayMicroseconds(2);
}

void CAN_RX_ISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR(void) { xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL); }

void scanKeysFunction(bool *press_list, float *damp_list, uint8_t *TX_Message) {
  for (int i = 0; i < 12; i++) {
    setRow(i / 4); // Floor division
    delayMicroseconds(3);
    // 3 scnarios:
    // 1: A key was not presses before and is now being presses
    if (!HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]) && !press_list[i]) {
      press_list[i] = true; // Set the "is-pressed" entry for that key to true
      if (xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
        // Critical section
        // Perform operations inside the critical section
        tone_idx[nok] = i + 1 + octave * 12;
        nok++; // Increase the no. of key being presses
        key = keystrings[i + octave * 12];
        xSemaphoreGive(protectedGlobals.mutex); // Release the mutex
      }
      TX_Message[0] = 'P';
      TX_Message[2] =
          i + 1 +
          octave *
              12; // Assign number of note played to 3rd entry in transmission
      if (transmitter) {
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
    // 2: A key was presses before and is now being released
    else if (HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]) && press_list[i]) {
      press_list[i] = false; // Set the "is-pressed" entry for that key to true
      if (xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
        // Critical section
        // Perform operations inside the critical section
        key = "";
        int *p = std::find(std::begin(tone_idx), std::end(tone_idx),
                           i + 1 + octave * 12);
        int idx = std::distance(tone_idx, p);
        tone_idx[idx] = 0;
        nok--;
        xSemaphoreGive(protectedGlobals.mutex); // Release the mutex
      }
      TX_Message[0] = 'R';
      TX_Message[2] = i + 1 + octave * 12;
      if (transmitter) {
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
    // 3: A key was not presses and is nor currently being presses or a key was
    // presses and is still being pressed Skip the key detetction for those keys
  }

  setRow(3); // Select the knob row
  delayMicroseconds(3);

  volumeKnob.updateRotation(
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) +
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)));
  dampKnob.updateRotation(std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) +
                          std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)));

  setRow(4);
  delayMicroseconds(3);
  instrumentKnob.updateRotation(
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) +
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)));
  octaveKnob.updateRotation(
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) +
      std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)));

  if (xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
    // Critical section
    // Perform operations inside the critical section
    instru = instrumentKnob.getRotationISR();
    damp_factor = damp_list[dampKnob.getRotationISR()];
    octave = octaveKnob.getRotationISR();
    xSemaphoreGive(protectedGlobals.mutex); // Release the mutex
  }
}

void displayUpdateFunction(std::string *instru_list, std::string *damp_str,
                           uint32_t ID) {
  // Update display
  u8g2.clearBuffer(); // clear the internal memory

  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  std::string display_vol =
      "Vol: " + std::to_string(volumeKnob.getRotation() * 125 / 10) + "%";
  u8g2.drawStr(2, 20, display_vol.c_str());
  u8g2.setCursor(2, 30);
  u8g2.print(instru_list[instrumentKnob.getRotation()].c_str());
  std::string octave_str =
      "Octave: " + std::to_string(octaveKnob.getRotation() + 1);
  u8g2.drawStr(60, 20, octave_str.c_str());
  u8g2.drawStr(60, 30, damp_str[dampKnob.getRotation()].c_str());

  if (xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
    // Critical section
    // Perform operations inside the critical section
    u8g2.drawStr(2, 10, key);
    xSemaphoreGive(protectedGlobals.mutex); // Release the mutex
  }

  u8g2.sendBuffer(); // transfer internal memory to the display

  // Toggle LED
  digitalToggle(LED_BUILTIN);
}

void CAN_RX_Function(uint8_t *local_RX) {
  xQueueReceive(msgInQ, local_RX, portMAX_DELAY);

  if (!transmitter || xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
    // Critical section
    // Perform operations inside the critical section

    for (int i = 0; i < 6; i++) {
      if (i == 0 && local_RX[0] == 'P') {
        tone_idx[0] = local_RX[2];
      } else {
        tone_idx[i] = 0;
      }
    }
    if (local_RX[0] == 'P') {
      nok = 1;
    } else {
      nok = 0;
    }

    xSemaphoreGive(protectedGlobals.mutex);
  }
}

void CAN_TX_Function(uint8_t *msgOut) {
  xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
  xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
  CAN_TX(0x125, msgOut);
}

void doubleBufferFunction() {
  for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE / 2; writeCtr++) {

    uint32_t localRotation = volumeKnob.getRotationISR();
    // If there's at least a key being presses, do something
    if (nok != 0) {
      // damp for those keys that are being pressed
      for (int i = 0; i < nok; i++) {
        damp[i] *= damp_factor;
      }
      Vout = (waveform_lut[instru][(t * 917 / Ts[tone_idx[0]]) % 917] *
                  damp[0] * (Ts[tone_idx[0]] != 1) +
              waveform_lut[instru][(t * 917 / Ts[tone_idx[1]]) % 917] *
                  damp[1] * (Ts[tone_idx[1]] != 1) +
              waveform_lut[instru][(t * 917 / Ts[tone_idx[2]]) % 917] *
                  damp[2] * (Ts[tone_idx[2]] != 1) +
              waveform_lut[instru][(t * 917 / Ts[tone_idx[3]]) % 917] *
                  damp[3] * (Ts[tone_idx[3]] != 1) +
              waveform_lut[instru][(t * 917 / Ts[tone_idx[4]]) % 917] *
                  damp[4] * (Ts[tone_idx[4]] != 1) +
              waveform_lut[instru][(t * 917 / Ts[tone_idx[5]]) % 917] *
                  damp[5] * (Ts[tone_idx[5]] != 1)) /
             max(nok, 1); // Divide the amplitude by the totoal number of keys
                          // being pressed

      Vout = Vout >> (8 - localRotation);
      t++; // increment timer
    }
    // If no keys are being pressed the timer resets along with the damp factors
    else {
      t = 0;
      damp[0] = 1;
      damp[1] = 1;
      damp[2] = 1;
      damp[3] = 1;
      damp[4] = 1;
      damp[5] = 1;
      Vout = 0;
    }

    if (doubleBuffer.writeBuffer1) {
      // doubleBuffer.sampleBuffer1[writeCtr] = Vout + 128;
      doubleBuffer.sampleBuffer1[writeCtr] = Vout;
    } else {
      // doubleBuffer.sampleBuffer0[writeCtr] = Vout + 128;
      doubleBuffer.sampleBuffer0[writeCtr] = Vout;
    }
  }
}

void scanKeysTask(void *pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  TX_Message[1] = 4; // Octave to be changed later - try auto assigning this.
  float damp_list[3] = {0.99995, 1, 0.9995};
  bool press_list[12] = {false};

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    scanKeysFunction(press_list, damp_list, TX_Message);
  }
}

void displayUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  std::string damp_str[3] = {"Normal", "Sustain", "Overdamp"};
  std::string instru_list[3] = {"sawtooth", "sine", "piano"};

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    displayUpdateFunction(instru_list, damp_str, ID);
  }
}

void CAN_RX_Task(void *pvParameters) {
  uint8_t local_RX[8];

  while (1) {
    CAN_RX_Function(local_RX);
  }
}

void CAN_TX_Task(void *pvParameters) {
  uint8_t msgOut[8];

  while (1) {
    CAN_TX_Function(msgOut);
  }
}

void doubleBufferISR() {
  static uint32_t readCtr = 0;
  //   static uint32_t Data;

  if (readCtr == SAMPLE_BUFFER_SIZE / 2) {
    readCtr = 0;
    doubleBuffer.writeBuffer1 = !doubleBuffer.writeBuffer1;
    xSemaphoreGiveFromISR(doubleBuffer.doubleBufferSemaphore, NULL);
  }

  if (doubleBuffer.writeBuffer1) {
    // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
    // DAC_ALIGN_12B_R,doubleBuffer.sampleBuffer0[readCtr++]);
    //  analogWriteResolution(12);
    analogWrite(OUTR_PIN, doubleBuffer.sampleBuffer0[readCtr++]);
  } else {
    // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
    // DAC_ALIGN_12B_R,doubleBuffer.sampleBuffer1[readCtr++]);
    //  analogWriteResolution(12);
    analogWrite(OUTR_PIN, doubleBuffer.sampleBuffer1[readCtr++]);
  }
}

void doubleBufferTask(void *pvParameters) {

  while (1) {
    xSemaphoreTake(doubleBuffer.doubleBufferSemaphore, portMAX_DELAY);
    if (xSemaphoreTake(protectedGlobals.mutex, portMAX_DELAY) == pdTRUE) {
      // Critical section
      // Perform operations inside the critical section
      doubleBufferFunction();
      xSemaphoreGive(protectedGlobals.mutex); // Release the mutex
    }
  }
}

// void dacInitializationTask(void * pvParameters) {
// 	// DAC_Init(hdac);
// 	// DAC_Config(hdac);
// 	// DAC_Start(hdac);
// 	// Initialize DAC Channel
// 	HAL_Init();

// 	__HAL_RCC_DAC1_CLK_ENABLE();

// 	hdac.Instance = DAC1;

// 	HAL_DAC_Init(&hdac);

// 	//Config DAC channel
// 	DAC_ChannelConfTypeDef sConfig;
// 	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
// 	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

// 	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

// 	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

// 	//Serial.println("DAC Setup Complete!");
// 	vTaskDelete(NULL); // Delete this task as it's no longer needed
// }

void setup() {
  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  waveform_lut.push_back(sawtooth());
  waveform_lut.push_back(sinewave());
  waveform_lut.push_back(piano1());

  // Initialise UART
  Serial.begin(9600);

  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(30000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(doubleBufferISR);
  sampleTimer->resume();

  CAN_Init(receiver && transmitter);
  setCANFilter();
  // setCANFilter(0x123,0x7ff);
  if (receiver) {
    CAN_RegisterRX_ISR(CAN_RX_ISR);
  }
  if (transmitter) {
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  }
  CAN_Start();

  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  TaskHandle_t CAN_RXHandle = NULL;
  TaskHandle_t CAN_TXHandle = NULL;
  TaskHandle_t doubleBufferHandle = NULL;
  TaskHandle_t dacInitializationHandle = NULL;

  protectedGlobals.mutex = xSemaphoreCreateMutex();

#ifndef TEST_SCANKEYS

  // xTaskCreate(
  //     	dacInitializationTask,
  //     	"dacInit",
  //     	128,
  //     	NULL,
  //     	2,  // Priority may need adjustment
  //     	&dacInitializationHandle
  // );

  xTaskCreate(displayUpdateTask,   /* Function that implements the task */
              "displayUpdate",     /* Text name for the task */
              256,                 /* Stack size in words, not bytes */
              NULL,                /* Parameter passed into the task */
              1,                   /* Task priority */
              &displayUpdateHandle /* Pointer to store the task handle */
  );

  xTaskCreate(scanKeysTask,   /* Function that implements the task */
              "scanKeys",     /* Text name for the task */
              256,            /* Stack size in words, not bytes */
              NULL,           /* Parameter passed into the task */
              4,              /* Task priority */
              &scanKeysHandle /* Pointer to store the task handle */
  );

  if (transmitter) {
    xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 128, NULL, 2, &CAN_TXHandle);
  }

  if (receiver) {
    xTaskCreate(CAN_RX_Task, "CAN_RX_Task", 128, NULL, 3, &CAN_RXHandle);

    xTaskCreate(doubleBufferTask, "doubleBuffer", 256, NULL, 5,
                &doubleBufferHandle);
  }

#endif

#ifdef TEST
  std::cout << "" << std::endl;
  std::cout << "Hardware timing diagnosis started." << std::endl;
  std::cout << "" << std::endl;
  HAL_Delay(1000);

  transmitter = false;
  int tone_idx[6] = {0};

  uint32_t startTime0 = micros();
  for (int iter = 0; iter < 32; iter++) {
    doubleBufferISR();
  }
  uint32_t elapsed0 = (micros() - startTime0) / 32;
  std::cout << "Double buffer ISR task latent execution time: " << elapsed0
            << " microseconds" << std::endl;

  uint32_t startTime = micros();
  nok = 1;
  for (int iter = 0; iter < 32; iter++) {
    doubleBufferFunction();
  }
  uint32_t elapsed1 = (micros() - startTime) / 32 + 1000;
  std::cout << "Double buffer task latent execution time: " << elapsed1
            << " microseconds" << std::endl;

  float damp_list[3] = {0.99995, 1, 0.9995};
  bool press_list[12] = {false};
  uint8_t TX_Message[8] = {0};
  startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    scanKeysFunction(press_list, damp_list, TX_Message);
  }
  uint32_t elapsed2 = (micros() - startTime) / 32 + 1000;
  std::cout << "Key scanning task latent execution time: " << elapsed2
            << " microseconds" << std::endl;

  std::string damp_str[3] = {"Normal", "Sustain", "Overdamp"};
  std::string instru_list[3] = {"sawtooth", "sine", "piano"};
  uint32_t ID;
  uint8_t localRX[8];
  startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    displayUpdateFunction(instru_list, damp_str, ID, localRX);
  }
  uint32_t elapsed3 = (micros() - startTime) / 32 + 1000;
  std::cout << "Display update task latent execution time: " << elapsed3
            << " microseconds" << std::endl;

  uint8_t msgOut[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  startTime = micros();
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
  CAN_TX(0x125, msgOut);
  uint32_t elapsed4 = (micros() - startTime) * 36 + 1000;
  std::cout << "Data transmission task latent execution time: " << elapsed4
            << " microseconds" << std::endl;

  uint8_t local_RX[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  startTime = micros();
  xQueueSend(msgInQ, local_RX, portMAX_DELAY);
  xQueueReceive(msgInQ, local_RX, portMAX_DELAY);
  rxMessage.receiveMessage(local_RX);
  if (!transmitter) {
    for (int i = 0; i < 6; i++) {
      if (i == 0 && local_RX[0] == 'P') {
        tone_idx[0] = local_RX[2];
      } else {
        tone_idx[i] = 0;
      }
    }
    if (local_RX[0] == 'P') {
      nok = 1;
    } else {
      nok = 0;
    }
  }
  __atomic_store_n(&currentStepSize, rxMessage.getStepSize(), __ATOMIC_RELAXED);
  uint32_t elapsed5 = (micros() - startTime) * 36 + 1000;
  std::cout << "Data receiving task latent execution time: " << elapsed5
            << " microseconds" << std::endl;

  uint32_t total_elapsed = micros() - startTime0;
  std::cout << "Total execution time: " << total_elapsed << " microseconds"
            << std::endl;
  uint32_t utilisation =
      (elapsed0 / 46. + elapsed1 / 17000. + elapsed2 / 20000. +
       elapsed3 / 100000. + elapsed4 / 60000. + elapsed5 / 25200.) *
      100;

  std::cout << "" << std::endl;
  std::cout << "CPU utilisation: " << std::to_string(utilisation) << "%"
            << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Hardware diagnosis complete." << std::endl;
  while (1);
#endif

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

  doubleBuffer.doubleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(doubleBuffer.doubleBufferSemaphore);

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}
