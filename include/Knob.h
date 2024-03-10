#ifndef KNOB_HPP
#define KNOB_HPP

#include <string>
#include "SysState.h"
#include <STM32FreeRTOS.h>
#include <cmath>

class Knob : public SysState {
public:
    Knob(float upperBound, float lowerBound, float increment);
    uint32_t getRotationISR();
    float getRotation();
    void updateRotation(std::string BA_curr);

protected:
    uint32_t rotationISR;
    float rotation;
    float rotationUpperBound;
    float rotationLowerBound;
    float increments;
    std::string BA_prev;
    bool incrementLast;
};

#endif
