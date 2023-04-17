//Created by Carlos Álvarez Cía 2023


#pragma once
#include "Helper.h"
#include "math.h"
#include "iostream"

static constexpr float DEFAULT_KP_PID = 1;
static constexpr float DEFAULT_KI_PID = 1;
static constexpr float DEFAULT_KD_PID = 1;
static constexpr float DT = 0.01;


class PID {
    private:
        float _kp, _ki, _kd;
        float int_error {0.0};
        float error_a {0.0};
        float max_i = 10;
        float error_a_filtered = {0.0};
        float last_der_error {0.0};
        const int num_samples = 5;
        int sample = 0;

    public:
        PID(float kp, float ki, float kd){
            _kp = kp;
            _ki = ki;
            _kd = kd;
        }

        //@brief Function to compute the PID action.
        //@param target: [float]
        //@param current: [float]
        //@returns [float] The PID action.
        float update_pid(float target, float current){
            float error = target - current;
            return update_pid(error);
        }

        //@brief Function to compute the PID action.
        //@param error: [float]
        //@returns [float] The PID action.
        float update_pid(float error){
            int_error += error*DT;
            saturation(int_error, -max_i, max_i);
            last_der_error = error - error_a_filtered;
            float out_controller = _kp*error + _ki*int_error + _kd*last_der_error;
            error_a += error/num_samples;
            sample--;
            if (sample <= 0){
                error_a_filtered = error_a;
                error_a = 0.0;
                sample = num_samples;
            }
            return out_controller;
        }

        //@brief Function to reset the integral term.
        void reset_i(){
            int_error = 0.0;
        }

        float get_I(){return int_error;}

        float get_Der(){return error_a_filtered;}

        //@brief Function to get or set the kp.
        float& kp(){return _kp;}

        //@brief Function to get or set the ki.
        float& ki(){return _ki;}
        
        //@brief Function to get or set the kd.
        float& kd(){return _kd;}

};