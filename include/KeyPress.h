#ifndef PRESS_H
#define PRESS_H

#include "SysState.h"
#include <stdint.h>
#include <string>

class keyPress : public SysState {
    public:
        keyPress();
        bool getPressIdx(int index);
        void toggleKeyPress(int index);
        int getNumKeys();
        std::string getKeyString();
        uint8_t* getTxMessage();

    private:
        bool pressList[12];
        int numberOfKeys;
        std::string key;
        std::string keystrings[12];
        uint8_t TX_Message[8];
};

#endif