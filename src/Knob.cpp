#include "Knob.h"
#include <STM32FreeRTOS.h>

uint8_t Knob::getRotationISR(){
    uint8_t rotation = __atomic_load_n(&this->rotation, __ATOMIC_RELAXED);
    return rotation;
}

uint8_t Knob::getRotation(){
  uint8_t rotation;
  xSemaphoreTake(this->mutex, portMAX_DELAY);  // Acquire the mutex
  rotation = this->rotation;  // Safely read the value of rotation
  xSemaphoreGive(this->mutex);  // Release the mutex

  return rotation;
}

void Knob::updateRotation(std::string BA_curr){
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    std::string state = this->BA_prev + BA_curr;

    uint8_t localRotation = this->rotation;

    if (state == "0001" || state == "1110") {
      localRotation = localRotation < 8 ? localRotation + 1 : localRotation;
      this->incrementLast = true;
    }

    if (state == "0100" || state == "1011") {
      localRotation = localRotation > 0 ? localRotation - 1 : localRotation;
      this->incrementLast = false;
    }

    if(state == "1100" || state == "1001" || state == "0110" || state == "0011"){
      if(this->incrementLast){
        localRotation = localRotation < 8 ? localRotation + 1 : localRotation;
      } else {
        localRotation = localRotation > 0 ? localRotation - 1 : localRotation;
      }
    }

    this->rotation = localRotation;
    this->BA_prev = BA_curr;

    xSemaphoreGive(this->mutex);
}