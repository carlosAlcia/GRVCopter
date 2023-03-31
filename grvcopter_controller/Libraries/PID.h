//Created by Carlos Álvarez Cía 2023


#pragma once
#include "Helper.h"

static constexpr float DEFAULT_KP_PID = 1;
static constexpr float DEFAULT_KI_PID = 1;
static constexpr float DEFAULT_KD_PID = 1;
static constexpr float DT = 0.01;


class PID {
    private:
        float _kp, _ki, _kd;
        float int_error {0.0};
        float error_a {0.0};
        float max_i = 0.5;


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
            int_error += error*DT;
            saturation(int_error, -max_i, max_i);
            float der_error = (error-error_a)/DT;
            float out_controller = _kp*error + _ki*int_error + _kd*der_error;
            error_a = error;
            return out_controller;
        }

        //@brief Function to compute the PID action.
        //@param error: [float]
        //@returns [float] The PID action.
        float update_pid(float error){
            int_error += error*DT;
            saturation(int_error, -max_i, max_i);
            float der_error = (error-error_a)/DT;
            float out_controller = _kp*error + _ki*int_error + _kd*der_error;
            error_a = error;
            return out_controller;
        }

        //@brief Function to reset the integral term.
        void reset_i(){
            int_error = 0.0;
        }

        //@brief Function to get or set the kp.
        float& kp(){return _kp;}

        //@brief Function to get or set the ki.
        float& ki(){return _ki;}
        
        //@brief Function to get or set the kd.
        float& kd(){return _kd;}

};