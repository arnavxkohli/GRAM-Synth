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
#include <ES_CAN.h>
#include "waveform.h"
#include <stm32l4xx_hal_gpio.h>
#include <stm32l432xx.h>
#include <algorithm>


Knob volumeKnob(12.0f, 0.0f, 1.0f);
Knob decayKnob(0.99999f, 0.9995f, -0.00005f);
Knob instrumentKnob(3.0f, 0.0f, 1.0f);

RX_Message rxMessage;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

SemaphoreHandle_t CAN_TX_Semaphore;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// This is used so that gpio_state can be selected by integers
const GPIO_PinState gpio_state[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};
// This stores the gpio pinouts corresponding to column idx C0 - 3
const uint32_t key_cols[4] = {GPIO_PIN_3, GPIO_PIN_8, GPIO_PIN_7, GPIO_PIN_9};
//// Additional variables
std::string key;
int keynums;
int tone_idx[6] = {0}; // The indices (0 - 12) of the waveform periods that the are being selected to play in the audio
std::vector<std::vector<std::vector<uint32_t>>> waveform_luts; // the waveform lut stores the insturment waveforms.
int nok = 0; // No. of keys currently being presses simutaneously
int instru = 0; // Instrument idx, current there are 4 instruments
// Recodes whether a key is being presses. This filters out the pressed keys and enables the key to be detected in the order of their presses
bool press_list[12] = {false};

const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
std::string keystrings[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
uint32_t Ts[13] = {};

int t = 0; // Initialised timer
float decay[6] = {1};
float decay_factor = 0.99999;
uint32_t Vout;

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN

	delayMicroseconds(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[bitIdx & 0x01]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[bitIdx & 0x02]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[bitIdx & 0x04]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, gpio_state[value]); // OUT_PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // REN_PIN
	delayMicroseconds(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
}

void setRow(uint8_t rowidx){
	std::bitset<3> Ridx = std::bitset<3>(rowidx);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
	delayMicroseconds(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[Ridx[0]]); // RA0_PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[Ridx[1]]); // RA1_PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[Ridx[2]]); // RA2_PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // REN_PIN
	delayMicroseconds(2);
}

void sampleISR() {
  uint32_t localRotation = static_cast<uint32_t>(volumeKnob.getRotationISR());
	// Serial.print(nok);
	// If there's at least a key being presses, do something
	if (nok != 0) {
		// Decay for those keys that are being pressed
		for (int i = 0; i < nok; i++) {
			decay[i] *= decay_factor;
		}
		// tone_idx[i] = the period index corresponding to that particular key
		// Ts = the 13 periods of the keys, the first period is 1 corresponding to no keys
		// instru = Selects the instrument, currrent is 0 - 3
    uint32_t instru = static_cast<uint32_t>(instrumentKnob.getRotationISR());
		Vout = (waveform_luts[instru][tone_idx[0]][(t % Ts[tone_idx[0]])] * decay[0] +
 					waveform_luts[instru][tone_idx[1]][(t % Ts[tone_idx[1]])] * decay[1] +
				 	waveform_luts[instru][tone_idx[2]][(t % Ts[tone_idx[2]])] * decay[2] +
					waveform_luts[instru][tone_idx[3]][(t % Ts[tone_idx[3]])] * decay[3] +
					waveform_luts[instru][tone_idx[4]][(t % Ts[tone_idx[4]])] * decay[4] +
					waveform_luts[instru][tone_idx[5]][(t % Ts[tone_idx[5]])] * decay[5]) / std::max(nok, 1); // Divide the amplitude by the totoal number of keys being pressed
    // Vout = Vout >> (12 - localRotation);
    // Sets audio resolution to 12
    analogWriteResolution(12);
    // The Vout is 12 bits already
    analogWrite(OUTR_PIN, Vout >> (12 - localRotation));
    t ++; // increment timer
  }
  	// If no keys are being pressed the timer resets along with the decay factors
	else {
		t = 0;
		decay[0] = 1;
		decay[1] = 1;
		decay[2] = 1;
		decay[3] = 1;
		decay[4] = 1;
		decay[5] = 1;
	}
}

// void sampleISR(){
//   static uint32_t phaseAcc = 0;
//   uint32_t localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
//   uint32_t localRotation = volumeKnob.getRotationISR();

//   phaseAcc += localCurrentStepSize;

//   int32_t Vout = (phaseAcc >> 24) - 128;

//   // localRotation used for volume before or after the 128?
//   Vout = Vout >> (8 - localRotation);

//   analogWrite(OUTR_PIN, Vout + 128);
// }


void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysFunction(uint8_t* TX_Message) {
	// uint32_t localCurrentStepSize = 0;

	// std::bitset<32> currentInputs = inputs.getCurrentInputs();
	// std::bitset<32> previousInputs = inputs.getPreviousInputs();

	// for(int row = 0; row < 4; row++){
	//   setRow(row);
	//   delayMicroseconds(3);
	//   for (int bit = 0; bit < 4; ++bit) {
	//       currentInputs[row * 4 + bit] = readCols()[bit];
	//   }
	// }

	for (int i = 0; i < 12; i++) {
			setRow(i / 4); // Floor division
			delayMicroseconds(3);
			// xSemaphoreTake(sysState.mutex, portMAX_DELAY);
			// sysState.inputs[i] = !HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]); // i % 4 iterate between 0 - 3 as i increases from 0 - 11
			// xSemaphoreGive(sysState.mutex);
			// 3 scnarios:
			// 1: A key was not presses before and is now being presses
			if (!HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]) && !press_list[i]) {
				press_list[i] = true; // Set the "is-pressed" entry for that key to true
				tone_idx[nok] = i + 1;
				TX_Message[0] = 'P';
		TX_Message[2] = i; // Assign number of note played to 3rd entry in transmission
				key = key + keystrings[i];
				nok ++; // Increase the no. of key being presses
			}
			// 2: A key was presses before and is now being released
			else if (HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]) && press_list[i]) {
				press_list[i] = false; // Set the "is-pressed" entry for that key to true
				key = "";
				TX_Message[0] = 'R';
				nok --;
				int* p = std::find(std::begin(tone_idx), std::end(tone_idx), i + 1);
				int idx = std::distance(tone_idx, p);
				tone_idx[idx] = 0;
			}
			// 3: A key was not presses and is nor currently being presses or a key was presses and is still being pressed
			// Skip the key detetction for those keys
		}

		setRow(3); // Select the knob row
		delayMicroseconds(3);

	volumeKnob.updateRotation(std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) +
		std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)));
	decayKnob.updateRotation(std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) +
		std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)));

	setRow(4);
	delayMicroseconds(3);
	instrumentKnob.updateRotation(std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) +
		std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)));

	// for(int i = 0; i < 12; i++){
	//   if(currentInputs[i]){
	//     localCurrentStepSize = stepSizes[i];
	//     TX_Message[2] = i;
	//     if(!previousInputs[i]){
	//       TX_Message[0] = 'P';
	//       __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
	//     }
	//   }
	//   else if (previousInputs[i]) {
	//     TX_Message[0] = 'R';
	//   }
	// }

	xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void displayUpdateFunction(uint32_t ID, uint8_t* localRX) {
	//Update display
	u8g2.clearBuffer();         // clear the internal memory

	u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
	u8g2.drawStr(2, 10, "Music Synth");  // write something to the internal memory
	u8g2.setCursor(2, 20);

	u8g2.print(instrumentKnob.getRotation());

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

void CAN_RX_Function(uint8_t* local_RX) {
		xQueueReceive(msgInQ, local_RX, portMAX_DELAY);
		rxMessage.receiveMessage(local_RX);
		__atomic_store_n(&currentStepSize, rxMessage.getStepSize(), __ATOMIC_RELAXED);
}

void CAN_TX_Function(uint8_t* msgOut) {
	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
	xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
	CAN_TX(0x123, msgOut);
	//xSemaphoreGive(CAN_TX_Semaphore);
}

void scanKeysTask(void * pvParameters) {
	const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t TX_Message[8] = {0};
	TX_Message[1] = 4; // Octave to be changed later - try auto assigning this.

	while(1) {
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		scanKeysFunction(TX_Message);
	}
}

void displayUpdateTask(void * pvParameters){
	const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t ID;
	uint8_t localRX[8];

	while(1){
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		displayUpdateFunction(ID, localRX);
	}
}

void CAN_RX_Task(void* pvParameters){
	uint8_t local_RX[8];

	while(1) {
		CAN_RX_Function(local_RX);
	}
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];

	while (1) {
		CAN_TX_Function(msgOut);
	}
}

void setup() {
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

	waveform_luts.push_back(sawtooth1());
	waveform_luts.push_back(sinewave());
	waveform_luts.push_back(piano1());
	waveform_luts.push_back(sawtooth2());

	Ts[0] = 1;
	for (int i = 1; i <= 12; i++) {
		Ts[i] = 22000 / frequencies[i - 1];
	}

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

	#ifndef TEST_SCANKEYS
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
	#endif

	#ifdef TEST_SCANKEYS
		uint8_t TX_Message[8] = {0};
		uint32_t startTime = micros();
		for (int iter = 0; iter < 32; iter++) {
			scanKeysFunction(TX_Message);
		}
		std::cout << "Key scanning task latent execution time: " << (micros()-startTime) / 32 << " microseconds" << std::endl;

		uint32_t ID;
		uint8_t localRX[8];
		startTime = micros();
		for (int iter = 0; iter < 32; iter++) {
			displayUpdateFunction(ID, localRX);
		}
		std::cout << "Display update task latent execution time: " << (micros()-startTime) / 32 << " microseconds" << std::endl;

		// uint8_t msgOut[8];
		// startTime = micros();
		// for (int iter = 0; iter < 1; iter++) {
		// 	CAN_TX_Function(msgOut);
		// }
		// std::cout << "Data transmission task latent execution time: " << (micros()-startTime) / 1 << " microseconds" << std::endl;

		// uint8_t local_RX[8];
		// startTime = micros();
		// for (int iter = 0; iter < 1; iter++) {
		// 	CAN_RX_Function(local_RX);
		// }
		// std::cout << "Data receiving task latent execution time: " << (micros()-startTime) / 1 << " microseconds" << std::endl;

		nok = 1;
		startTime = micros();
		for (int iter = 0; iter < 32; iter++) {
			sampleISR();
		}
		std::cout << "DAC ISR task latent execution time: " << (micros()-startTime) / 32 << " microseconds" << std::endl;

		std::cout << "" << std::endl;
		while(1);
	#endif

	CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

	vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}
