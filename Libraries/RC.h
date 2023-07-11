//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Messages_GRVCopter.h"
#include <iostream>

static constexpr int CH_ROLL = 0;
static constexpr int CH_PITCH = 1;
static constexpr int CH_THROTTLE = 2;
static constexpr int CH_YAW = 3;
static constexpr int CH_MODE = 4;
static constexpr int CH_GRVCOPTER_ENABLED = 9-1; //Channel 9 in the RC, one less because of beginning in 0.
static constexpr int CH_ARMED = 6;

class RC {


    private:
        float channels[MSG_GRVCOPTER::MAX_NUM_CHANNEL] {0};
        bool negative_throttle = false;

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

        float get_z_speed(){
            float channel_throttle = channels[CH_THROTTLE];
            if (channel_throttle < -0.1){
                negative_throttle = true;
            }
            if (negative_throttle){
                return channel_throttle;
            } else {
                return (channel_throttle-0.5)*2.0;
            }
        }

        bool grvcopter_enabled(){
            return channels[CH_GRVCOPTER_ENABLED] > 0.5;
        }

        bool is_armed(){
            //std::cout << channels[CH_ARMED] << std::endl;
            return channels[CH_ARMED] > 0.5;
        }



};