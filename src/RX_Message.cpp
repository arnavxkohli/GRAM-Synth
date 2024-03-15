#include "RX_Message.h"
#include <STM32FreeRTOS.h>
#include <cstring>
#include "globals.h"

void RX_Message::receiveMessage(const std::array<uint8_t, 8>& message) {
    uint32_t localCurrentStepSize = 0;

    if (message[0] == 'P') {
        localCurrentStepSize = (stepSizes[message[2]] << message[1]) >> 4;
    }

    // Lock the mutex before accessing shared data
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    // Update currentStepSize and rxMessage
    this->currentStepSize = localCurrentStepSize;
    this->rxMessage = message;

    // Release the mutex
    xSemaphoreGive(this->mutex);
}

uint32_t RX_Message::getStepSize() {
    // Lock the mutex before accessing shared data
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    // Get currentStepSize
    uint32_t stepSize = this->currentStepSize;

    // Release the mutex
    xSemaphoreGive(this->mutex);

    return stepSize;
}

const std::array<uint8_t, 8>& RX_Message::getRX_Message() {
    // Lock the mutex before accessing shared data
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    // Return rxMessage
    const std::array<uint8_t, 8>& message = this->rxMessage;

    // Release the mutex
    xSemaphoreGive(this->mutex);

    return message;
}
