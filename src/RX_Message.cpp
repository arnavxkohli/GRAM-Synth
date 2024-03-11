#include "RX_Message.h"
#include <STM32FreeRTOS.h>
#include <cstring>
#include "globals.h"

void RX_Message::receiveMessage(uint8_t message[8]){
    uint32_t localCurrentStepSize = 0;

    if(message[0] == 'P'){
        localCurrentStepSize = (stepSizes[message[2]] << message[1]) >> 4;
    }

    xSemaphoreTake(this->mutex, portMAX_DELAY);
    this->currentStepSize = localCurrentStepSize;
    memcpy(this->RX_Message, message, 8);
    xSemaphoreGive(this->mutex);
}

uint32_t RX_Message::getStepSize(){
    uint32_t stepSize = 0;

    xSemaphoreTake(this->mutex, portMAX_DELAY);
    stepSize = this->currentStepSize;
    xSemaphoreGive(this->mutex);

    return stepSize;
}

uint8_t* RX_Message::getRX_Message(){
    uint8_t* rxMessage;

    xSemaphoreTake(this->mutex, portMAX_DELAY);
    rxMessage = this->RX_Message;
    xSemaphoreGive(this->mutex);

    return rxMessage;
}