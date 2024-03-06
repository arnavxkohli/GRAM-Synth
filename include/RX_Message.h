#ifndef RX_MESSAGE_HPP
#define RX_MESSAGE_HPP

#include "SysState.h"
#include <stdint.h>
#include <STM32FreeRTOS.h>

class RX_Message : public SysState{
    public:
        void receiveMessage(uint8_t message[8]);
        uint32_t getStepSize();
        uint8_t* getRX_Message();

    private:
        uint8_t RX_Message[8];
        uint32_t currentStepSize;
};

#endif