#include "KeyPress.h"

keyPress::keyPress() : pressList{false}, keystrings{"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"}, TX_Message{0} {
    this->TX_Message[1] = 4;
}

bool keyPress::getPressIdx(int index){
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    bool pressIdx = this->pressList[index];
    xSemaphoreGive(this->mutex);
    return pressIdx;
}

int keyPress::getNumKeys(){
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    int numKeys = this->numberOfKeys;
    xSemaphoreGive(this->mutex);
    return numKeys;
}

std::string keyPress::getKeyString(){
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    std::string keyString = this->key;
    xSemaphoreGive(this->mutex);
    return keyString;
}

uint8_t* keyPress::getTxMessage(){
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    uint8_t* txMessage = this->TX_Message;
    xSemaphoreGive(this->mutex);
    return txMessage;
}

// You still need to take care of the tone_idx bits in both cases

void keyPress::toggleKeyPress(int index){
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    if(this->pressList[index]){
        this->numberOfKeys--;
        this->key = "";
        this->TX_Message[0] = 'R';
    } else {
        this->numberOfKeys++;
        this->key += this->keystrings[index];
        this->TX_Message[0] = 'P';
    }

    this->TX_Message[2] = index;

    this->pressList[index] = !this->pressList[index];
    xSemaphoreGive(this->mutex);
}