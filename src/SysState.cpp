#include "SysState.h"
#include <STM32FreeRTOS.h>

SysState::SysState(){
    this->mutex = xSemaphoreCreateMutex();
}

SysState::~SysState(){
    vSemaphoreDelete(this->mutex);
}