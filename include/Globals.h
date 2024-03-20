#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <vector>
#include <Arduino.h>
#include <stdint.h>

//Pin definitions
//Row select and enable

extern const int RA0_PIN;
extern const int RA1_PIN;
extern const int RA2_PIN;
extern const int REN_PIN;

//Matrix input and output
extern const int C0_PIN;
extern const int C1_PIN;
extern const int C2_PIN;
extern const int C3_PIN;
extern const int OUT_PIN;

//Audio analogue out
extern const int OUTL_PIN;
extern const int OUTR_PIN;

//Joystick analogue in
extern const int JOYY_PIN;
extern const int JOYX_PIN;

//Output multiplexer bits
extern const int DEN_BIT;
extern const int DRST_BIT;
extern const int HKOW_BIT;
extern const int HKOE_BIT;

extern volatile uint32_t currentStepSize;

// This is used so that gpio_state can be selected by integers
extern const GPIO_PinState gpio_state[2];
// This stores the gpio pinouts corresponding to column idx C0 - 3
extern const uint32_t key_cols[4];
//// Additional variables
extern std::string key;
extern int keynum;
extern int tone_idx[6]; // The indices (0 - 12) of the waveform periods that the are being selected to play in the audio
extern std::vector<std::vector<uint8_t>> waveform_lut; // the waveform lut stores the insturment waveforms.
extern int nok; // No. of keys currently being presses simutaneously
extern int instru; // Instrument idx, current there are 4 instruments
// Recodes whether a key is being presses. This filters out the pressed keys and enables the key to be detected in the order of their presses
extern bool press_list[12];

extern std::string keystrings[72];
extern uint16_t Ts[73];
extern std::string instru_list[3];

extern int t; // Initialised timer
extern float decay_list[3];
extern float decay[6];
extern float decay_factor;
extern uint16_t Vout;
extern int octave;

extern const uint32_t stepSizes[]; // Step size array

#endif