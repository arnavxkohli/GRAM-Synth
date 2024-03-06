#include "Inputs.h"
#include <bitset>

std::bitset<32> Inputs::getCurrentInputs() {
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    std::bitset<32> curInputs = this->currentInputs;
    xSemaphoreGive(this->mutex);

    return curInputs;
}

std::bitset<32> Inputs::getPreviousInputs() {
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    std::bitset<32> prevInputs = this->previousInputs;
    xSemaphoreGive(this->mutex);

    return prevInputs;
}

void Inputs::updateInputs(std::bitset<32> inputs) {
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    this->previousInputs = this->currentInputs;
    this->currentInputs = inputs;
    xSemaphoreGive(this->mutex);
}
