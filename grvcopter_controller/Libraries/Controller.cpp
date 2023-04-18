//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"
#include "UAV.h"
#include "Parameters.h"

extern COMMON::Common& common;
extern LOG::Logger& logger;


void Controller::run(){

    if (common.start_grvcopter()){
        position_control.reset_pids();
        attitude_control.reset_pids();
    }

    PARAMS::Params* params = common.get_params();
    /*if (params->check_constant_pid_changed()){
        params->update_angle_pid_constants(attitude_control.get_roll_ang_pid(), attitude_control.get_pitch_ang_pid(), attitude_control.get_yaw_ang_pid());
        params->update_rate_pid_constants(attitude_control.get_roll_rate_pid(), attitude_control.get_pitch_rate_pid(), attitude_control.get_yaw_rate_pid());
        params->update_pos_pid_constants(position_control.get_x_pos_pid(), position_control.get_y_pos_pid(), position_control.get_z_pos_pid());
        params->update_vel_pid_constants(position_control.get_x_vel_pid(), position_control.get_y_vel_pid(), position_control.get_z_vel_pid());
        params->pid_constants_updated();
    }*/

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

    logger.save_float_data(LOG_C::FORCE_X_DES_ID, force_xyz_n[0]);
    logger.save_float_data(LOG_C::FORCE_Y_DES_ID, force_xyz_n[1]);
    logger.save_float_data(LOG_C::FORCE_Z_DES_ID, force_xyz_n[2]);
    logger.save_float_data(LOG_C::TORQUE_ROLL_DES_ID, torques_nm[0]);
    logger.save_float_data(LOG_C::TORQUE_PITCH_DES_ID, torques_nm[1]);
    logger.save_float_data(LOG_C::TORQUE_YAW_DES_ID, torques_nm[2]);


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

    for (uint8_t i = 0; i < UAV::num_motors; i++){
        logger.save_float_data(LOG_C::PWM1_ID+i, pwms[i]);
    }

    

}

void Controller::run_stabilize_control(Force& forces, Torques& torques){
    compute_yaw_target_from_rc();
    compute_targets_from_rc_stabilize_control();
    Force force_xyz_n;
    force_xyz_n.z() = (common.get_rc()->get_throttle()+1.0);

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    if (UAV::fully_actuated){
        //Transform target attitude to XY forces:
        Force from_angles;
        from_angles = Attitude::get_forces_by_angles(*common.get_target_attitude());
        force_xyz_n.x() = from_angles[0];
        force_xyz_n.y() = from_angles[1];

        Attitude att_only_yaw = *common.get_target_attitude();
        att_only_yaw.roll() = 0.0;
        att_only_yaw.pitch() = 0.0;
        ang_accel_des = attitude_control.run(&att_only_yaw, common.get_current_attitude(), common.get_current_rate());

    } else {
        //Cancel XY forces, only altitude control:
        force_xyz_n[0] = 0.0;
        force_xyz_n[1] = 0.0;
        ang_accel_des = attitude_control.run(common.get_target_attitude(), common.get_current_attitude(), common.get_current_rate());
    }
    //Compute the desired torques given the UAV inertia
    Torques torques_nm;
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;

    forces = force_xyz_n;
    torques = torques_nm;
}

void Controller::run_position_control(Force& forces, Torques& torques){
    compute_yaw_target_from_rc();
    compute_targets_from_rc_pos_control();
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
        force_xyz_n.limit_to_xy(1000);
    } 
    
    //Compute the desired attitude from the position controller.
    Attitude attitude_from_position_controller;
    if (!UAV::fully_actuated) {
        attitude_from_position_controller = Attitude::get_angles_from_forces(force_xyz_n);
        //Limit lean angle to UAV::max_lean_angle.
        attitude_from_position_controller = Attitude::limit_lean_angle(attitude_from_position_controller, UAV::max_lean_angle);
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
    compute_yaw_target_from_rc();
    compute_targets_from_rc_alt_control();
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

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;

    if (UAV::fully_actuated){
        //Transform target attitude to XY forces:
        Force from_angles;
        from_angles = Attitude::get_forces_by_angles(*common.get_target_attitude());
        force_xyz_n.x() = from_angles[0];
        force_xyz_n.y() = from_angles[1];

        Attitude att_only_yaw = *common.get_target_attitude();
        att_only_yaw.roll() = 0.0;
        att_only_yaw.pitch() = 0.0;
        ang_accel_des = attitude_control.run(&att_only_yaw, common.get_current_attitude(), common.get_current_rate());

    } else {
        //Cancel XY forces, only altitude control:
        force_xyz_n[0] = 0.0;
        force_xyz_n[1] = 0.0;
        ang_accel_des = attitude_control.run(common.get_target_attitude(), common.get_current_attitude(), common.get_current_rate());
    }
    //Compute the desired torques given the UAV inertia
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;
    
    Mixer::normalize_force_by_mass(force_xyz_n);

    forces = force_xyz_n;
    torques = torques_nm;
}

void Controller::compute_targets_from_rc_stabilize_control(){
    RC* rc = common.get_rc();
    Attitude* att_target = common.get_target_attitude();
    att_target->roll() = UAV::max_lean_angle*rc->get_channel(CH_ROLL);
    att_target->pitch() = UAV::max_lean_angle*rc->get_channel(CH_PITCH);
}

void Controller::compute_targets_from_rc_pos_control(){
    RC* rc = common.get_rc();
    float Z_speed_NED = -rc->get_throttle();
    Position* target = common.get_target_position();
    target->z() = target->z() + Z_speed_NED*DT;
    target->x() = target->x() + rc->get_channel(CH_PITCH)*DT;
    target->y() = target->y() + rc->get_channel(CH_ROLL)*DT;
}

void Controller::compute_targets_from_rc_alt_control(){
    RC* rc = common.get_rc();
    float z_speed_NED = -rc->get_throttle();
    Position* target = common.get_target_position();
    target->z() = target->z() + z_speed_NED*DT;
    
    Attitude* att_target = common.get_target_attitude();
    att_target->roll() = UAV::max_lean_angle*rc->get_channel(CH_ROLL);
    att_target->pitch() = UAV::max_lean_angle*rc->get_channel(CH_PITCH);
}

void Controller::compute_yaw_target_from_rc(){
    RC* rc = common.get_rc();
    float yaw_speed = rc->get_channel(CH_YAW);
    //Apply dead band to avoid undesired yaw rotation due to small values in RC.
    yaw_speed = dead_band(yaw_speed, UAV::dead_band_width_yaw_rc);
    Attitude* att_target = common.get_target_attitude();
    att_target->yaw() = att_target->yaw() + yaw_speed*DT;
}