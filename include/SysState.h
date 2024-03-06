#ifndef SYSSTATE_HPP
#define SYSSTATE_HPP

#include <STM32FreeRTOS.h>

class SysState{
    public:
        SysState();
        ~SysState();

    protected:
        SemaphoreHandle_t mutex;
};

#endif