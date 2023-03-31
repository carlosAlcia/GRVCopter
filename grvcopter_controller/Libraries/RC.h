//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Messages_GRVCopter.h"

static constexpr int CH_ROLL = 0;
static constexpr int CH_PITCH = 1;
static constexpr int CH_THROTTLE = 2;
static constexpr int CH_YAW = 3;
static constexpr int CH_GRVCOPTER_ENABLED = 9-1; //Channel 9 in the RC, one less because of beginning in 0.

class RC {


    private:
        float channels[MSG_GRVCOPTER::MAX_NUM_CHANNEL] {0};

    public:
        RC(){}
        ~RC(){}

        void operator=(float *_rc){
            for (int i = 0; i < MSG_GRVCOPTER::MAX_NUM_CHANNEL; i++){
                channels[i] = _rc[i];
            }
        }

        void operator=(RC _rc){
            for (int i = 0; i < MSG_GRVCOPTER::MAX_NUM_CHANNEL; i++){
                channels[i] = _rc.get_channel(i);
            }
        }

        float get_channel(int ch){return channels[ch];};

        float get_throttle(){return channels[CH_THROTTLE];};

        bool grvcopter_enabled(){
            return channels[CH_GRVCOPTER_ENABLED] > 0.5;
        }



};