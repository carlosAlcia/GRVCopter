//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"

#include "UAV.h"

extern COMMON::Common& common;

void Controller::run(){

    if (common.start_grvcopter()){
        position_control.reset_pids();
        attitude_control.reset_pids();
    }

    Acceleration accel_xyz_mss;
    Force force_xyz_n;
    if (common.has_position()) {
        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
        //Convert acceleration in forces with the UAV mass:
        force_xyz_n = accel_xyz_mss*UAV::mass_kg;
        //Rotate forces from fixed frame to UAV frame:
        force_xyz_n.rotate_to_uav_frame(common.get_current_attitude()->yaw());
    } 

    //Compute the desired attitude from the position controller.
    Attitude attitude_from_position_controller;
    if (!UAV::fully_actuated) {
        attitude_from_position_controller = Attitude::get_angles_from_forces(force_xyz_n);
    }

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    ang_accel_des = attitude_control.run(&attitude_from_position_controller, common.get_current_attitude(), common.get_current_rate());

    //Compute the desired torques given the UAV inertia
    Torques torques_nm;
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;

    //Get force contribution of each motor with the mixer:
    float force_motor[UAV::num_motors]{0.0};
    Mixer::get_forces_each_motor(&mixer, &force_xyz_n, &torques_nm, force_motor);

    normalize_sign(force_motor, UAV::num_motors);

    float pwms[UAV::num_motors]{0.0};
    Mixer::force_to_pwm(force_motor, pwms);

    MSG_GRVCOPTER::Message_Bytes msg;
    MSG_GRVCOPTER::pack_pwm_message(UAV::num_motors, pwms, &msg);
    char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};
    MSG_GRVCOPTER::get_bytes_msg(&msg, buffer);
    common.get_socket()->Send(buffer);
    

}