#include "VolumeKnob.h"

void VolumeKnob::updateRotation(std::string BA_curr){
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    std::string state = this->BA_prev + BA_curr;

    uint8_t localRotation = this->rotation;

    if (state == "0001" || state == "1110") {
      localRotation = localRotation < 12 ? localRotation + 1 : localRotation;
      this->incrementLast = true;
    }

    if (state == "0100" || state == "1011") {
      localRotation = localRotation > 0 ? localRotation - 1 : localRotation;
      this->incrementLast = false;
    }

    if(state == "1100" || state == "1001" || state == "0110" || state == "0011"){
      if(this->incrementLast){
        localRotation = localRotation < 12 ? localRotation + 1 : localRotation;
      } else {
        localRotation = localRotation > 0 ? localRotation - 1 : localRotation;
      }
    }

    this->rotation = localRotation;
    this->BA_prev = BA_curr;

    xSemaphoreGive(this->mutex);
}