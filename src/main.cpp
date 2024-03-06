#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <cstddef>
#include <string>
#include <cstring>
#include "Knob.h"
#include "Globals.h"
#include "RX_Message.h"
#include "Inputs.h"
#include <ES_CAN.h>

Knob Knob3;
RX_Message rxMessage;
Inputs inputs;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

SemaphoreHandle_t CAN_TX_Semaphore;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN,value);
  digitalWrite(REN_PIN,HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols(){
  std::bitset<4> result;

  result[0] = !digitalRead(C0_PIN);
  result[1] = !digitalRead(C1_PIN);
  result[2] = !digitalRead(C2_PIN);
  result[3] = !digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  std::bitset<3> rowBits = std::bitset<3>(rowIdx);

  digitalWrite(RA0_PIN, rowBits[0]);
  digitalWrite(RA1_PIN, rowBits[1]);
  digitalWrite(RA2_PIN, rowBits[2]);

  digitalWrite(REN_PIN, HIGH);
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  uint8_t localRotation = Knob3.getRotationISR();

  phaseAcc += localCurrentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;

  // localRotation used for volume before or after the 128?
  Vout = Vout >> (8 - localRotation);

  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  TX_Message[1] = 4; // Octave to be changed later - try auto assigning this.

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    uint32_t localCurrentStepSize = 0;

    std::bitset<32> currentInputs = inputs.getCurrentInputs();
    std::bitset<32> previousInputs = inputs.getPreviousInputs();

    for(int row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(3);
      for (int bit = 0; bit < 4; ++bit) {
          currentInputs[row * 4 + bit] = readCols()[bit];
      }
    }

    Knob3.updateRotation(std::to_string(currentInputs[13]) + std::to_string(currentInputs[12]));

    for(int i = 0; i < 12; i++){
      if(currentInputs[i]){
        localCurrentStepSize = stepSizes[i];
        TX_Message[2] = i;
        if(!previousInputs[i]){
          TX_Message[0] = 'P';
          __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
        }
      }
      else if (previousInputs[i]) {
        TX_Message[0] = 'R';
      }
    }

    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);

    inputs.updateInputs(currentInputs);
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  uint8_t localRX[8];

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();         // clear the internal memory

    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2, 10, "Music Synth");  // write something to the internal memory
    u8g2.setCursor(2, 20);

    u8g2.print(Knob3.getRotation());

    // xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    // u8g2.drawStr(2, 30, sysState.notePlayed);
    // xSemaphoreGive(sysState.mutex);
    memcpy(localRX, rxMessage.getRX_Message(), 8);
    u8g2.setCursor(66,30);
    u8g2.print((char) localRX[0]);
    u8g2.print(localRX[1]);
    u8g2.print(localRX[2]);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void CAN_RX_Task(void* pvParameters){
  uint8_t local_RX[8];

  while(1){
    xQueueReceive(msgInQ, local_RX, portMAX_DELAY);
    rxMessage.receiveMessage(local_RX);
    __atomic_store_n(&currentStepSize, rxMessage.getStepSize(), __ATOMIC_RELAXED);
  }
}

void CAN_TX_Task (void * pvParameters) {
  uint8_t msgOut[8];

  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
    //xSemaphoreGive(CAN_TX_Semaphore);
  }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
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

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  TaskHandle_t CAN_RXHandle = NULL;
  TaskHandle_t CAN_TXHandle = NULL;

  xTaskCreate(
    displayUpdateTask, /* Function that implements the task */
    "displayUpdate", /* Text name for the task */
    256, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    1, /* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  );

  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  );

  xTaskCreate(
    CAN_RX_Task,
    "CAN_RX_Task",
    64,
    NULL,
    1,
    &CAN_RXHandle
  );

  xTaskCreate(
    CAN_TX_Task,
    "CAN_TX_Task",
    64,
    NULL,
    1,
    &CAN_TXHandle
  );

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}
