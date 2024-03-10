#ifndef VOLUME_HPP
#define VOLUME_HPP

#include "Knob.h"

class VolumeKnob : public Knob{
    public:
        void updateRotation(std::string BA_curr) override;
};

#endif