//Created by Carlos Álvarez Cía 2023
#pragma once

#include "PID.h"
#include "Attitude.h"

class AttitudeControl {
    private:
        PID pid_roll_ang = PID(DEFAULT_KP_PID*8, 0, 0); //Estaban a 3
        PID pid_pitch_ang = PID(DEFAULT_KP_PID*8, 0, 0);
        PID pid_yaw_ang = PID(DEFAULT_KP_PID*2, 0, 0);

        PID pid_roll_rate = PID(DEFAULT_KP_PID*0.15, DEFAULT_KI_PID*0.001, DEFAULT_KD_PID*0.01, 1); //El KD estaba a 0.01
        PID pid_pitch_rate = PID(DEFAULT_KP_PID*0.15, DEFAULT_KI_PID*0.001, DEFAULT_KD_PID*0.01, 1);
        PID pid_yaw_rate = PID(DEFAULT_KP_PID*0.1, DEFAULT_KI_PID*0.05, DEFAULT_KD_PID*0.01, 1);

        float ff_ang_rp = (M_PI/180)*0.15; //Feedforward factor.
        float ff_ang_yaw = (M_PI/180)*0.15; //Feedforward factor.

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

        PID* get_roll_ang_pid(){
            return &pid_roll_ang;
        }

        PID* get_pitch_ang_pid(){
            return &pid_pitch_ang;
        }

        PID* get_yaw_ang_pid(){
            return &pid_yaw_ang;
        }

        PID* get_roll_rate_pid(){
            return &pid_roll_rate;
        }

        PID* get_pitch_rate_pid(){
            return &pid_pitch_rate;
        }

        PID* get_yaw_rate_pid(){
            return &pid_yaw_rate;
        }

        void set_ff_rp_value(float new_value){
            ff_ang_rp = new_value;
        }

        void set_ff_yaw_value(float new_value){
            ff_ang_yaw = new_value;
        }

        //@brief Run the attitude controller to compute the desired angular acceleration.
        //@param target: [Attitude*] Desired attitude in rad.
        //@param current_att: [Attitude*] Current attitude in rad.
        //@param current_rate: [Rate*] Current rate in rad/s.
        //@return [Angular_Acceleration] Angular acceleration to reach the desired attitude, in rad/s².
        Angular_Acceleration run(Attitude* target, Attitude *current_att, Rate *current_rate);

};