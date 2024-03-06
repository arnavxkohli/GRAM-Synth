#ifndef INPUTS_HPP
#define INPUTS_HPP

#include "SysState.h"
#include <bitset>

class Inputs : public SysState{
    public:
        std::bitset<32> getCurrentInputs();
        std::bitset<32> getPreviousInputs();
        void updateInputs(std::bitset<32> inputs);

    private:
        std::bitset<32> previousInputs;
        std::bitset<32> currentInputs;
};

#endif
