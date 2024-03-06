#ifndef KNOB_HPP
#define KNOB_HPP

#include <string>
#include "SysState.h"
#include <STM32FreeRTOS.h>

class Knob : public SysState {
public:
    uint8_t getRotationISR();
    uint8_t getRotation();
    void updateRotation(std::string BA_curr);

private:
    uint8_t rotation;
    std::string BA_prev;
    bool incrementLast;
};

#endif
