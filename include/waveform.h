// #include <iostream>
// #include <cmath>
// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <vector>
// #include <fstream>

// std::vector<std::vector<uint32_t>> sinewave() {
//     const double half_range = 2047.25; // Half of the maximum value of uint32_t
//     const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
//     std::vector<std::vector<uint32_t>> sinewave_lut;
//     std::vector<uint32_t> zero_vec;
//     zero_vec.push_back(0);
//     sinewave_lut.push_back(zero_vec);
//     for (int i = 0; i < 12; i++) {
//         uint32_t T = 22000 / frequencies[i];
//         std::vector<uint32_t> sinewave;
//         sinewave.reserve(T); // Reserve memory for efficiency
//         for (int j = 0; j < T; j++) {
//             sinewave.push_back((sin(2 * M_PI * j / T) + 1) * half_range);
//         }
//         sinewave_lut.push_back(sinewave);
//     }
//     return sinewave_lut;
// }

// std::vector<std::vector<uint32_t>> sawtooth1() {
//     const double max_range = pow(2, 12) - 1; // Half of the maximum value of uint32_t
//     const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
//     std::vector<std::vector<uint32_t>> sawtooth_lut;
//     std::vector<uint32_t> zero_vec;
//     zero_vec.push_back(0);
//     sawtooth_lut.push_back(zero_vec);
//     for (int i = 0; i < 12; i++) {
//         double T = 22000 / frequencies[i];
//         std::vector<uint32_t> sawtooth;
//         sawtooth.reserve(T); // Reserve memory for efficiency
//         for (int j = 0; j < T; j++) {
//             sawtooth.push_back((j/T) * max_range);
//         }
//         sawtooth_lut.push_back(sawtooth);
//     }
//     return sawtooth_lut;
// }

// std::vector<std::vector<uint32_t>> piano1() {
//     const double half_range = 2047.25; // Half of the maximum value of uint32_t
//     const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
//     std::vector<std::vector<uint32_t>> piano_lut;
//     std::vector<uint32_t> zero_vec;
//     double tone;
//     zero_vec.push_back(0);
//     piano_lut.push_back(zero_vec);
//     for (int i = 0; i < 12; i++) {
//         uint32_t T = 22000 / frequencies[i];
//         std::vector<uint32_t> pianowave;
//         pianowave.reserve(T); // Reserve memory for efficiency
//         for (int j = 0; j < T; j++) {
//             tone = (0.6 * sin(2*M_PI*j/T) + 0.4 * sin(4*M_PI*j/T) + pow((0.6 * sin(2*M_PI*j/T) + 0.4 * sin(4*M_PI*j/T)), 3)) * 2650;
//             pianowave.push_back(tone);
//         }
//         piano_lut.push_back(pianowave);
//     }
//     return piano_lut;
// }

// std::vector<std::vector<uint32_t>> sawtooth2() {
//     const double max_range = pow(2, 12) - 1; // Half of the maximum value of uint32_t
//     const uint32_t frequencies[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
//     std::vector<std::vector<uint32_t>> sawtooth_lut;
//     std::vector<uint32_t> zero_vec;
//     zero_vec.push_back(0);
//     sawtooth_lut.push_back(zero_vec);
//     for (int i = 0; i < 12; i++) {
//         double T = 22000 / frequencies[i];
//         std::vector<uint32_t> sawtooth;
//         sawtooth.reserve(T); // Reserve memory for efficiency
//         for (int j = 0; j < T; j++) {
//             sawtooth.push_back(((T - j)/T) * max_range);
//         }
//         sawtooth_lut.push_back(sawtooth);
//     }
//     return sawtooth_lut;
// }