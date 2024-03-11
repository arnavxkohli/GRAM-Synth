// #include <iostream>
// #include <iomanip>
// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <cmath>
// #include <bitset>
// #include <STM32FreeRTOS.h>
// #include <ES_CAN.h>
// #include <stm32l4xx_hal_gpio.h>
// #include <stm32l432xx.h>
// #include <vector>
// #include <waveform.h>

// //Constants
// const uint32_t interval = 100; //Display update interval

// //Pin definitions
// //Row select and enable
// const int RA0_PIN = D3;
// const int RA1_PIN = D6;
// const int RA2_PIN = D12;
// const int REN_PIN = A5;

// //Matrix input and output
// const uint16_t C0_PIN = A2;
// const uint16_t C1_PIN = D9;
// const uint16_t C2_PIN = A6;
// const uint16_t C3_PIN = D1;
// const uint16_t OUT_PIN = D11;

// //Audio analogue out
// const int OUTL_PIN = A4;
// const int OUTR_PIN = A3;

// //Joystick analogue in
// const int JOYY_PIN = A0;
// const int JOYX_PIN = A1;

// //Output multiplexer bits
// const int DEN_BIT = 3;
// const int DRST_BIT = 4;
// const int HKOW_BIT = 5;
// const int HKOE_BIT = 6;

// const GPIO_PinState gpio_state[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};
// const uint32_t key_cols[4] = {GPIO_PIN_3, GPIO_PIN_8, GPIO_PIN_7, GPIO_PIN_9};

// std::string key;
// int keynums;
// uint32_t tone_idx[4] = {0};
// std::vector<std::vector<std::vector<uint32_t>>> waveform_luts;
// int nok = 0;
// double reverb = 0.0005;
// int instru = 0;

// //Display driver object
// U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
// std::string keystrings[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
// uint32_t Ts[13] = {};

// struct {
// 	std::bitset<32> inputs;
// 	double vol_KnobCount = 0;
// 	SemaphoreHandle_t mutex;
// } sysState;

// //Function to set outputs using key matrix
// void setOutMuxBit(const uint8_t bitIdx, const bool value) {

// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
// 	delayMicroseconds(2);
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[bitIdx & 0x01]);
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[bitIdx & 0x02]);
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[bitIdx & 0x04]);
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, gpio_state[value]); // OUT_PIN
// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // REN_PIN
// 	delayMicroseconds(2);
// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
// }

// void setRow(uint8_t rowidx){
// 	std::bitset<3> Ridx = std::bitset<3>(rowidx);
// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // REN_PIN
// 	delayMicroseconds(2);
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_state[Ridx[0]]); // RA0_PIN
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, gpio_state[Ridx[1]]); // RA1_PIN
// 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, gpio_state[Ridx[2]]); // RA2_PIN
// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // REN_PIN
// 	delayMicroseconds(2);
// }

// // std::bitset<4> readCols() {
// //   std::bitset<4> result;

// //   result[0] = ~digitalRead(C0_PIN);
// //   result[1] = ~digitalRead(C1_PIN);
// //   result[2] = ~digitalRead(C2_PIN);
// //   result[3] = ~digitalRead(C3_PIN);

// //   return result;
// // }
// static uint32_t amp;
// int t[4] = {0};
// static uint32_t nok_prev = 0;
// uint32_t Vout;

// void sampleISR() {
// 	// Serial.println(nok);
// 	// Serial.print(t);
// 	if (nok != 0) {
// 		Vout = (waveform_luts[instru][tone_idx[0]][(t[0] % Ts[tone_idx[0]])] + 
// 					  waveform_luts[instru][tone_idx[1]][(t[0] % Ts[tone_idx[1]])] + 
// 					  waveform_luts[instru][tone_idx[2]][(t[0] % Ts[tone_idx[2]])] + 
// 					  waveform_luts[instru][tone_idx[3]][(t[0] % Ts[tone_idx[3]])]) / std::max(nok, 1);
// 		Vout = Vout / (1 + t[0] * reverb);
// 		analogWriteResolution(12);
// 		analogWrite(OUTR_PIN, Vout >> (12 - static_cast<uint32_t>(sysState.vol_KnobCount)));
// 		t[0] ++;
// 		// t[1] ++;
// 		// t[2] ++;
// 		// t[3] ++;
// 	}
// 	else {
// 		t[0] = 0;
// 		// t[1] = 0;
// 	}
// }

// void scanKeysTask(void * pvParameters) {
// 	const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
// 	TickType_t xLastWakeTime = xTaskGetTickCount();
// 	std::string BA3_prev("00");
// 	std::string BA3_new("00");
// 	std::string BA2_prev("00");
// 	std::string BA2_new("00");
// 	std::string BA1_prev("00");
// 	std::string BA1_new("00");
// 	uint8_t TX_Message[8] = {0};
// 	TX_Message[1] = 4;
// 	while(1) {
// 		vTaskDelayUntil( &xLastWakeTime, xFrequency );
// 		uint32_t tone_idx_temp[4] = {0};
// 		int nok_temp = 0;
// 		TX_Message[0] = 'R';
// 		std::string key = "";
// 		int j = 0;
// 		for (int i = 0; i < 12; i++) {
// 			setRow(i / 4);
// 			delayMicroseconds(3);
// 			xSemaphoreTake(sysState.mutex, portMAX_DELAY);
// 			sysState.inputs[i] = !HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4]);
// 			xSemaphoreGive(sysState.mutex);
// 			if (!HAL_GPIO_ReadPin(GPIOA, key_cols[i % 4])) {
// 				tone_idx_temp[j] = i + 1;
// 				TX_Message[0] = 'P';
// 				key = key + keystrings[i];
// 				nok_temp ++;
// 				j ++;
// 			}
// 		}
// 		for (int i = 0; i < 4; i++) {
// 			tone_idx[i] = tone_idx_temp[i];
// 		}
// 		nok = nok_temp;

// 		setRow(3);
// 		delayMicroseconds(3);
// 		// xSemaphoreTake(sysState.mutex, portMAX_DELAY);
// 		// sysState.inputs[12] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
// 		// sysState.inputs[13] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
// 		// xSemaphoreGive(sysState.mutex);
// 		BA3_new = std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) + std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
// 		BA2_new = std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) + std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7));
// 		setRow(4);
// 		BA1_new = std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) + std::to_string(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));

// 		if (BA3_prev + BA3_new == "0001" || BA3_prev + BA3_new == "1110") {
// 			xSemaphoreTake(sysState.mutex, portMAX_DELAY);
// 			sysState.vol_KnobCount ++;
// 			xSemaphoreGive(sysState.mutex);
// 		}
// 		if (BA3_prev + BA3_new == "0100" || BA3_prev + BA3_new == "1011") {
// 			xSemaphoreTake(sysState.mutex, portMAX_DELAY);
// 			sysState.vol_KnobCount --;
// 			xSemaphoreGive(sysState.mutex);
// 		}
// 		if (BA2_prev + BA2_new == "0001" || BA2_prev + BA2_new == "1110") {
// 			reverb -= 0.0001;
// 		}
// 		if (BA2_prev + BA2_new == "0100" || BA2_prev + BA2_new == "1011") {
// 			reverb += 0.0001;
// 		}
// 		if (BA1_prev + BA1_new == "0001" || BA1_prev + BA1_new == "1110") {
// 			instru ++;
// 		}
// 		if (BA1_prev + BA1_new == "0100" || BA1_prev + BA1_new == "1011") {
// 			instru --;
// 		}
// 		sysState.vol_KnobCount = std::max(std::min(sysState.vol_KnobCount, 12.), 0.);
// 		reverb = std::max(std::min(reverb, 0.001), 0.0001);
// 		instru = std::max(std::min(instru, 3), 0);
// 		BA3_prev = BA3_new;
// 		BA2_prev = BA2_new;
// 		BA1_prev = BA1_new;

// 		TX_Message[2] = 0;
// 		CAN_TX(0x123, TX_Message);
// 	}
// }

// void displayUpdateTask(void * pvParameters) {
// 	const TickType_t xFrequency = 50 /portTICK_PERIOD_MS;
// 	TickType_t xLastWakeTime = xTaskGetTickCount();
// 	uint32_t ID;
// 	uint8_t RX_Message[8]={0};

// 	while(1) {
// 		while (CAN_CheckRXLevel()) {
// 		CAN_RX(ID, RX_Message);
// 		}
// 	vTaskDelayUntil( &xLastWakeTime, xFrequency );
// 	//Update display
// 	u8g2.clearBuffer();         // clear the internal memory
// 	u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
// 	u8g2.drawStr(2,10,"Keyboard");  // write something to the internal memory
// 	u8g2.setCursor(2,20);
// 	u8g2.print(static_cast<const char*>(key.c_str()));
// 	std::string display_vol = "Volume: " + std::to_string(static_cast<int>(sysState.vol_KnobCount / 12 * 100)) + "%";
// 	u8g2.drawStr(2, 30, display_vol.c_str());
// 	u8g2.setCursor(80,30);
// 	u8g2.print((char) RX_Message[0]);
// 	// u8g2.print(RX_Message[1]);
// 	// u8g2.print(RX_Message[2]);
// 	u8g2.print(reverb * 1000);
// 	u8g2.print(instru);

// 	u8g2.sendBuffer();          // transfer internal memory to the display  
// 	//Toggle LED
// 	digitalToggle(LED_BUILTIN);
// 	}
// }

// void setup() {
// 	//Initialise UART
// 	Serial.begin(9600);
// 	waveform_luts.push_back(sawtooth1());
// 	waveform_luts.push_back(sinewave());
// 	waveform_luts.push_back(piano1());
// 	waveform_luts.push_back(sawtooth2());
// 	// Define the octave for this keyboard
// 	CAN_Init(true);
// 	setCANFilter(0x123,0x7ff);
// 	CAN_Start();
// 	//Set pin directions
// 	pinMode(RA0_PIN, OUTPUT);
// 	pinMode(RA1_PIN, OUTPUT);
// 	pinMode(RA2_PIN, OUTPUT);
// 	pinMode(REN_PIN, OUTPUT);
// 	pinMode(OUT_PIN, OUTPUT);
// 	pinMode(OUTL_PIN, OUTPUT);
// 	pinMode(OUTR_PIN, OUTPUT);
// 	pinMode(LED_BUILTIN, OUTPUT);

// 	pinMode(C0_PIN, INPUT);
// 	pinMode(C1_PIN, INPUT);
// 	pinMode(C2_PIN, INPUT);
// 	pinMode(C3_PIN, INPUT);
// 	pinMode(JOYX_PIN, INPUT);
// 	pinMode(JOYY_PIN, INPUT);

// 	Ts[0] = 1;
// 	for (int i = 1; i <= 12; i++) {
// 		Ts[i] = 22000 / frequencies[i - 1];
// 	}

// 	TIM_TypeDef *Instance = TIM1;
// 	HardwareTimer *sampleTimer = new HardwareTimer(Instance);
// 	sampleTimer -> setOverflow(22000, HERTZ_FORMAT);
// 	sampleTimer -> attachInterrupt(sampleISR);
// 	sampleTimer -> resume();

// 	//Initialise display
// 	setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
// 	delayMicroseconds(2);
// 	setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
// 	u8g2.begin();
// 	setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

// 	sysState.mutex = xSemaphoreCreateMutex();
// 	TaskHandle_t scanKeysHandle = NULL;
// 	TaskHandle_t displayUpdateHandle = NULL;

// 	// #define DISABLE_THREADS

// 	#ifndef DISABLE_THREADS
// 		xTaskCreate(
// 			displayUpdateTask,		/* Function that implements the task */
// 			"displayUpdate",		/* Text name for the task */
// 			256,      		/* Stack size in words, not bytes */
// 			NULL,			/* Parameter passed into the task */
// 			1,			/* Task priority */
// 			&displayUpdateHandle);	/* Pointer to store the task handle */

// 		xTaskCreate(
// 			scanKeysTask,		/* Function that implements the task */
// 			"scanKeys",		/* Text name for the task */
// 			64,      		/* Stack size in words, not bytes */
// 			NULL,			/* Parameter passed into the task */
// 			2,			/* Task priority */
// 			&scanKeysHandle);	/* Pointer to store the task handle */
// 	#endif

// 	// #define TEST_SCANKEYS

// 	#ifdef TEST_SCANKEYS
// 		uint32_t startTime = micros();
// 		for (int iter = 0; iter < 32; iter++) {
// 			scanKeysTask(NULL);
// 			Serial.println(iter);
// 		}
// 		Serial.println(micros()-startTime);
// 		while(1);
// 	#endif
// 	// 3.125 ms worst case scenario
// 	// 2.813 ms typically

// 	vTaskStartScheduler();
// }

// void loop() {
// // put your main code here, to run repeatedly:
// }