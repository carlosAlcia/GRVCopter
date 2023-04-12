//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"

#include "UAV.h"

extern COMMON::Common& common;
extern LOG::Logger& logger;


void Controller::run(){

    if (common.start_grvcopter()){
        position_control.reset_pids();
        attitude_control.reset_pids();
    }

    Force force_xyz_n;
    Torques torques_nm;


    switch (common.get_mode())
    {
    case UAV::MODE_STABILIZE:
        run_stabilize_control(force_xyz_n, torques_nm);
        break;
    case UAV::MODE_ALTITUDE:
        run_altitude_control(force_xyz_n, torques_nm);
        break;
    case UAV::MODE_POSITION:
        run_position_control(force_xyz_n, torques_nm);
        break;
    
    default:
        break;
    }

    logger.save_float_data("ForceX", force_xyz_n[0]);
    logger.save_float_data("ForceY", force_xyz_n[1]);
    logger.save_float_data("ForceZ", force_xyz_n[2]);
    logger.save_float_data("TorqueRoll", torques_nm[0]);
    logger.save_float_data("TorquePitch", torques_nm[1]);
    logger.save_float_data("TorqueYaw", torques_nm[2]);


    //Get force contribution of each motor with the mixer:
    float force_motor[UAV::num_motors]{0.0};
    
    Mixer::get_forces_each_motor(&mixer, &force_xyz_n, &torques_nm, force_motor);

    float pwms[UAV::num_motors]{0.0};
    Mixer::force_to_pwm(force_motor, pwms);

    MSG_GRVCOPTER::Message_Bytes msg;
    MSG_GRVCOPTER::pack_pwm_message(UAV::num_motors, pwms, &msg);
    char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};

    MSG_GRVCOPTER::get_bytes_msg(&msg, buffer);
    common.get_socket()->Send(buffer);

    for (int i = 0; i < UAV::num_motors; i++){
        logger.save_float_data("PWM" + std::to_string(i), pwms[i]);
    }

    

}

void Controller::run_stabilize_control(Force& forces, Torques& torques){
    Force force_xyz_n;
    force_xyz_n.z() = (common.get_rc()->get_throttle()+1.0);

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    ang_accel_des = attitude_control.run(common.get_target_attitude(), common.get_current_attitude(), common.get_current_rate());

    //Compute the desired torques given the UAV inertia
    Torques torques_nm;
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;

    forces = force_xyz_n;
    torques = torques_nm;
}

void Controller::run_position_control(Force& forces, Torques& torques){
    Acceleration accel_xyz_mss;
    Force force_xyz_n;
    Torques torques_nm;

    if (common.has_position()) {
        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
        //Convert acceleration in forces with the UAV mass:
        force_xyz_n = accel_xyz_mss*UAV::mass_kg;
        //Add gravity (Need to change with orientation, but for now constant in Z):
        force_xyz_n.z() -= UAV::mass_kg*9.81;
        //Rotate forces from fixed frame to UAV frame:
        force_xyz_n.rotate_to_uav_frame(common.get_current_attitude()->yaw());
        force_xyz_n.from_NED_to_NEU();
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
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;
    
    Mixer::normalize_force_by_mass(force_xyz_n);
    

    forces = force_xyz_n;
    torques = torques_nm;
}

void Controller::run_altitude_control(Force& forces, Torques& torques){
    Acceleration accel_xyz_mss;
    Force force_xyz_n;
    Torques torques_nm;

    if (common.has_position()) {
        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
        //Convert acceleration in forces with the UAV mass:
        force_xyz_n = accel_xyz_mss*UAV::mass_kg;
        //Add gravity (Need to change with orientation, but for now constant in Z):
        force_xyz_n.z() -= UAV::mass_kg*9.81;
        //Rotate forces from fixed frame to UAV frame:
        force_xyz_n.rotate_to_uav_frame(common.get_current_attitude()->yaw());
        force_xyz_n.from_NED_to_NEU();
    } 
    
    //Cancel XY forces, only altitude control:
    force_xyz_n[0] = 0.0;
    force_xyz_n[1] = 0.0;

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    ang_accel_des = attitude_control.run(common.get_target_attitude(), common.get_current_attitude(), common.get_current_rate());

    //Compute the desired torques given the UAV inertia
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;
    
    Mixer::normalize_force_by_mass(force_xyz_n);

    forces = force_xyz_n;
    torques = torques_nm;
}