//Created by Carlos Álvarez Cía 2023
#pragma once

#include "PID.h"
#include "Attitude.h"

class AttitudeControl {
    private:
        PID pid_roll_ang = PID(DEFAULT_KP_PID*8, 0, 0); //Estaban a 3
        PID pid_pitch_ang = PID(DEFAULT_KP_PID*8, 0, 0);
        PID pid_yaw_ang = PID(DEFAULT_KP_PID*2, 0, 0);

        PID pid_roll_rate = PID(DEFAULT_KP_PID*0.15, DEFAULT_KI_PID*0.001, DEFAULT_KD_PID*0.01); //El KD estaba a 0.01
        PID pid_pitch_rate = PID(DEFAULT_KP_PID*0.15, DEFAULT_KI_PID*0.001, DEFAULT_KD_PID*0.01);
        PID pid_yaw_rate = PID(DEFAULT_KP_PID*0.1, DEFAULT_KI_PID*0.05, DEFAULT_KD_PID*0.01);

    public:
        AttitudeControl(){}

        void reset_pids(){
            pid_roll_rate.reset_i();
            pid_pitch_rate.reset_i();
            pid_yaw_rate.reset_i();
            pid_roll_ang.reset_i();
            pid_pitch_ang.reset_i();
            pid_yaw_ang.reset_i();
        }

              //@brief Function to reset the integral term of PID roll.
        void reset_pid_roll(){
            pid_roll_ang.reset_i();
            pid_roll_rate.reset_i();
        }

        //@brief Function to reset the integral term of PID pitch.
        void reset_pid_pitch()
        {
            pid_pitch_ang.reset_i();
            pid_pitch_rate.reset_i();
        }
        //@brief Function to reset the integral term of PID yaw.
        void reset_pid_yaw()
        {
            pid_yaw_ang.reset_i();
            pid_yaw_rate.reset_i();
        }

        Angular_Acceleration run(Attitude* target, Attitude *current_att, Rate *current_rate);

};