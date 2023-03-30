//Created by Carlos Álvarez Cía 2023
#pragma once

#include "PID.h"

class AttitudeControl {
    private:
        PID pid_roll = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_pitch = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_yaw = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);

    public:
        AttitudeControl(){}

        void reset_pids(){
            pid_roll.reset_i();
            pid_pitch.reset_i();
            pid_yaw.reset_i();
        }
};