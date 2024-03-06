#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

extern const uint32_t interval; // Display update interval

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

extern const uint32_t stepSizes[]; // Step size array
extern const char* notes[]; // The note being played

#endif