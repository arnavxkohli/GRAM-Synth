#ifndef RX_MESSAGE_HPP
#define RX_MESSAGE_HPP

#include "SysState.h"
#include <array>
#include <cstdint>
#include <STM32FreeRTOS.h>

class RX_Message : public SysState {
public:
    void receiveMessage(const std::array<uint8_t, 8>& message);
    uint32_t getStepSize();
    const std::array<uint8_t, 8>& getRX_Message();

private:
    std::array<uint8_t, 8> rxMessage;
    uint32_t currentStepSize;
};

#endif // RX_MESSAGE_HPP
