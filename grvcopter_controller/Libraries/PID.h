//Created by Carlos Álvarez Cía 2023


#pragma once

static constexpr float DEFAULT_KP_PID = 1.0;
static constexpr float DEFAULT_KI_PID = 0.1;
static constexpr float DEFAULT_KD_PID = 0.1;
static constexpr float DT = 0.01;


class PID {
    private:
        float _kp, _ki, _kd;
        float int_error {0.0};
        float error_a {0.0};


    public:
        PID(float kp, float ki, float kd){
            _kp = kp;
            _ki = ki;
            _kd = kd;
        }

        float update_pid(float target, float current){
            float error = target - current;
            int_error += error*DT;
            float der_error = (error-error_a)/DT;
            float out_controller = _kp*error + _ki*int_error + _kd*der_error;
            error_a = error;
            return out_controller;
        }

        float update_pid(float error){
            int_error += error*DT;
            float der_error = (error-error_a)/DT;
            float out_controller = _kp*error + _ki*int_error + _kd*der_error;
            error_a = error;
            return out_controller;
        }

        void reset_i(){
            int_error = 0.0;
        }

        float& kp(){return _kp;}
        float& ki(){return _ki;}
        float& kd(){return _kd;}

};