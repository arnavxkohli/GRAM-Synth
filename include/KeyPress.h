#ifndef PRESS_HPP
#define PRESS_HPP

#include "SysState.h"
#include <array>
#include <cstdint>
#include <string>

class keyPress : public SysState {
public:
    keyPress();
    bool getPressIdx(int index);
    void toggleKeyPress(int index);
    int getNumKeys();
    int getNumKeysISR();
    std::string getKeyString();
    const std::array<uint8_t, 8>& getTxMessage();

private:
    std::array<bool, 12> pressList;
    int numberOfKeys;
    std::string key;
    std::array<std::string, 12> keystrings;
    std::array<uint8_t, 8> TX_Message;
};

#endif
