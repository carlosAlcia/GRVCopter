#include "Attitude_Controller.h"

Angular_Acceleration AttitudeControl::run(Attitude* target, Attitude *current_att, Rate *current_rate){
    
    Attitude att_error;
    att_error = Attitude::compute_error(*target, *current_att);
    att_error = Attitude::from_rad_to_degrees(att_error);

    //Compute rate desired:
    Rate rate_des_from_pid; //Rate desired in Degrees/s
    rate_des_from_pid.roll() = pid_roll_ang.update_pid(att_error.roll());
    rate_des_from_pid.pitch() = pid_pitch_ang.update_pid(att_error.pitch());
    rate_des_from_pid.yaw() = pid_yaw_ang.update_pid(att_error.yaw());

    Angular_Acceleration feedforward;
    feedforward = rate_des_from_pid*FF;

    Rate rate_error; 
    ///Current rate comes in rad/s.
    rate_error = Attitude::compute_error(Attitude::from_degrees_to_rad(rate_des_from_pid), *current_rate);
    rate_error = Attitude::from_rad_to_degrees(rate_error);

    Angular_Acceleration ang_accel_from_pid;
    ang_accel_from_pid.roll() = pid_roll_rate.update_pid(rate_error.roll());
    ang_accel_from_pid.pitch() = pid_pitch_rate.update_pid(rate_error.pitch());
    ang_accel_from_pid.yaw() = pid_yaw_rate.update_pid(rate_error.yaw());

    //Add feedforward:
    ang_accel_from_pid = ang_accel_from_pid + feedforward;
    return ang_accel_from_pid;

}