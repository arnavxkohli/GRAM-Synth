#include "Knob.h"

Knob::Knob(float upperBound, float lowerBound, float increment)
  : rotationUpperBound(upperBound), rotationLowerBound(lowerBound), increments(increment) {
  this->rotation = lowerBound;
  this->rotationISR = static_cast<uint32_t>(lowerBound * (1 << 10));
}

float Knob::getRotationISR() {
  uint32_t unShiftedRotation = __atomic_load_n(&this->rotationISR, __ATOMIC_RELAXED);
  return static_cast<float>(unShiftedRotation) / (1 << 10);
}

float Knob::getRotation() {
  float rotation;
  xSemaphoreTake(this->mutex, portMAX_DELAY);  // Acquire the mutex
  rotation = this->rotation;  // Safely read the value of rotation
  xSemaphoreGive(this->mutex);  // Release the mutex

  return rotation;
}

void Knob::updateRotation(std::string BA_curr) {
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    std::string state = this->BA_prev + BA_curr;

    float localRotation = this->rotation;

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
    this->rotationISR = static_cast<uint32_t>(localRotation * (1 << 10)); // Scale factor: 2^10
    this->BA_prev = BA_curr;

    xSemaphoreGive(this->mutex);
}
