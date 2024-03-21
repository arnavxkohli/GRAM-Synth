#include "Globals.h"
#include <stdint.h>
#include <Arduino.h>
#include <iostream>

//Pin definitions
//Row select and enable

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

volatile uint32_t currentStepSize;

// This is used so that gpio_state can be selected by integers
const GPIO_PinState gpio_state[2] = {GPIO_PIN_RESET, GPIO_PIN_SET}; ////
// This stores the gpio pinouts corresponding to column idx C0 - 3
const uint32_t key_cols[4] = {GPIO_PIN_3, GPIO_PIN_8, GPIO_PIN_7, GPIO_PIN_9}; ////
//// Additional variables
char* key; ////

int tone_idx[6] = {0}; // The indices (0 - 12) of the waveform periods that the are being selected to play in the audio
std::vector<std::vector<uint8_t>> waveform_lut; // the waveform lut stores the insturment waveforms.
int nok = 0; // No. of keys currently being presses simutaneously
int instru = 0; // Instrument idx, current there are 4 instruments
// Recodes whether a key is being presses. This filters out the pressed keys and enables the key to be detected in the order of their presses

char* keystrings[72] = {"C1", "C1#", "D1", "D1#", "E1", "F1", "F1#", "G1", "G1#", "A1", "A1#", "B1", "C2", "C2#", "D2", "D2#", "E2", "F2", "F2#", "G2", "G2#", "A2", "A2#", "B2", "C3", "C3#", "D3", "D3#", "E3", "F3", "F3#", "G3", "G3#", "A3", "A3#", "B3", "C4", "C4#", "D4", "D4#", "E4", "F4", "F4#", "G4", "G4#", "A4", "A4#", "B4", "C5", "C5#", "D5", "D5#", "E5", "F5", "F5#", "G5", "G5#", "A5", "A5#", "B5", "C6", "C6#", "D6", "D6#", "E6", "F6", "F6#", "G6", "G6#", "A6", "A6#", "B6"};
uint16_t Ts[73] = {1, 917, 866, 817, 771, 728, 687, 649, 612, 578, 545, 515, 486, 459, 433, 409, 386, 364, 344, 324, 306, 289, 273, 257, 243, 229, 216, 204, 193, 182, 172, 162, 153, 144, 136, 129, 121, 115, 108, 102, 96, 91, 86, 81, 77, 72, 68, 64, 61, 57, 54, 51, 48, 46, 43, 41, 38, 36, 34, 32, 30, 29, 27, 26, 24, 23, 21, 20, 19, 18, 17, 16, 15};

int t = 0; // Initialised timer ////
float damp[6] = {1}; ////
float damp_factor = 0.99999; ////
uint16_t Vout; ////
int octave; ////

const uint32_t stepSizes[] = {51149155, 54077542, 57396381, 60715219, 64424509,
68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};