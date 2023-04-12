#include "Attitude_Controller.h"

Angular_Acceleration AttitudeControl::run(Attitude* target, Attitude *current_att, Rate *current_rate){
    Attitude att_error;
    att_error = Vector::error(target, current_att);
    att_error = att_error * 180/M_PI;
    //Compute rate desired:
    Rate rate_des_from_pid;
    rate_des_from_pid.roll() = pid_roll_ang.update_pid(att_error.roll());
    rate_des_from_pid.pitch() = pid_roll_ang.update_pid(att_error.pitch());
    rate_des_from_pid.yaw() = pid_roll_ang.update_pid(att_error.yaw());

    Rate rate_error;
    rate_error = Vector::error(&rate_des_from_pid, current_rate);
    
    Angular_Acceleration ang_accel_from_pid;
    ang_accel_from_pid.roll() = pid_roll_rate.update_pid(rate_error.roll());
    ang_accel_from_pid.pitch() = pid_roll_rate.update_pid(rate_error.pitch());
    ang_accel_from_pid.yaw() = pid_roll_rate.update_pid(rate_error.yaw());

    return ang_accel_from_pid;

}