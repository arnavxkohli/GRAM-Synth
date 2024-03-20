#ifndef KNOB_HPP
#define KNOB_HPP

#include <string>
#include "SysState.h"
#include <STM32FreeRTOS.h>
#include <cmath>

class Knob : public SysState {
public:
    Knob(int upperBound, int lowerBound, int increment);
    uint32_t getRotationISR();
    int getRotation();
    void updateRotation(std::string BA_curr);

protected:
    uint32_t rotationISR;
    int rotation;
    int rotationUpperBound;
    int rotationLowerBound;
    int increments;
    std::string BA_prev;
    bool incrementLast;
};

#endif
