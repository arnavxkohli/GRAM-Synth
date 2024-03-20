#include "Knob.h"

Knob::Knob(int upperBound, int lowerBound, int increment)
  : rotationUpperBound(upperBound), rotationLowerBound(lowerBound), increments(increment) {
  this->rotation = lowerBound;
  this->rotationISR = lowerBound;
}

uint32_t Knob::getRotationISR() {
  return __atomic_load_n(&this->rotationISR, __ATOMIC_RELAXED);
}

int Knob::getRotation() {
  int rotation;
  xSemaphoreTake(this->mutex, portMAX_DELAY);  // Acquire the mutex
  rotation = this->rotation;  // Safely read the value of rotation
  xSemaphoreGive(this->mutex);  // Release the mutex

  return rotation;
}

void Knob::updateRotation(std::string BA_curr) {
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    std::string state = this->BA_prev + BA_curr;

    int localRotation = this->rotation;

    if (state == "0001" || state == "1110") {
      localRotation = localRotation < this->rotationUpperBound ? localRotation + this->increments : localRotation;
      this->incrementLast = true;
    }

    if (state == "0100" || state == "1011") {
      localRotation = localRotation > this->rotationLowerBound ? localRotation - this->increments : localRotation;
      this->incrementLast = false;
    }

    if(state == "1100" || state == "1001" || state == "0110" || state == "0011"){
      if(this->incrementLast){
        localRotation = localRotation < this->rotationUpperBound ? localRotation + this->increments : localRotation;
      } else {
        localRotation = localRotation > this->rotationLowerBound ? localRotation - this->increments : localRotation;
      }
    }

    this->rotation = localRotation;
    this->rotationISR = localRotation;
    this->BA_prev = BA_curr;

    xSemaphoreGive(this->mutex);
}