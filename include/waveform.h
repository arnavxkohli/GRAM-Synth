#include <iostream>
#include <cmath>
#include <Arduino.h>
#include <U8g2lib.h>
#include <vector>
#include <fstream>

std::vector<uint8_t> sinewave() {
    const double half_range = pow(2, 7) - 1; // Half of the maximum value of uint16_t
    uint16_t T = 917;
    std::vector<uint8_t> sinewave_lut;
    sinewave_lut.reserve(T); // Reserve memory for efficiency
    for (int j = 0; j < T; j++) {
        sinewave_lut.push_back((sin(2 * M_PI * j / T) + 1) * half_range);
    }

    return sinewave_lut;
}

std::vector<uint8_t> sawtooth() {
    const double max_range = pow(2, 8) - 1; // Maximum value of uint16_t
    std::vector<uint8_t> sawtooth_lut;
    double T = 917;
    sawtooth_lut.reserve(T); // Reserve memory for efficiency
    for (int j = 0; j < T; j++) {
        sawtooth_lut.push_back((j/T) * max_range);
    }

    return sawtooth_lut;
}

std::vector<uint8_t> piano1() {
    std::vector<uint8_t> piano_lut;
    double tone;
    uint16_t T = 917;
    piano_lut.reserve(T); // Reserve memory for efficiency
    for (int j = 0; j < T; j++) {
        tone = (0.6 * sin(2*M_PI*j/T) + 0.4 * sin(4*M_PI*j/T) + pow((0.6 * sin(2*M_PI*j/T) + 0.4 * sin(4*M_PI*j/T)), 3)) * 167;
        piano_lut.push_back(tone);
    }

    return piano_lut;
}

std::vector<std::vector<uint16_t>> base1() {
    const double half_range = pow(2, 12) - 1; // Half of the maximum value of uint16_t
    const uint16_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
    std::vector<std::vector<uint16_t>> base_lut;
    std::vector<uint16_t> zero_vec;
    zero_vec.push_back(0);
    base_lut.push_back(zero_vec);
    for (int i = 0; i < 12; i++) {
        double T = 22000 / frequencies[i];
        std::vector<uint16_t> base;
        base.reserve(T); // Reserve memory for efficiency
        for (int j = 0; j < T * 5; j++) {
            base.push_back(sin(j * exp(-j/(T * 0.7)) + 1) * half_range);
        }
        base_lut.push_back(base);
        // std::cout << i << std::endl;
    }
    return base_lut;
}

std::vector<std::vector<uint16_t>> base2() {
    const double max_range = pow(2, 12) - 1; // Half of the maximum value of uint16_t
    const uint16_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
    std::vector<std::vector<uint16_t>> base_lut;
    std::vector<uint16_t> zero_vec;
    zero_vec.push_back(0);
    base_lut.push_back(zero_vec);
    for (int i = 0; i < 12; i++) {
        double T = 22000 / frequencies[i];
        std::vector<uint16_t> base;
        base.reserve(T); // Reserve memory for efficiency
        for (int j = 0; j < T * 5; j++) {
            base.push_back(sin(j * exp(-j/(T * 5))) * 1024);
        }
        base_lut.push_back(base);
        // std::cout << i << std::endl;
    }
    return base_lut;
}