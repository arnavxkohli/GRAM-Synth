#include "Globals.h"
#include <stdint.h>
#include <Arduino.h>

const uint32_t interval = 100; //Display update interval

const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

volatile uint32_t currentStepSize = 0;

const uint32_t stepSizes[] = {51149155, 54077542, 57396381, 60715219, 64424509,
68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

const char* notes[] = {"C", "C#", "D", "D#", "E", "F",
                       "F#", "G", "G#", "A", "A#", "B"};