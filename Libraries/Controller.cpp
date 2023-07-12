//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"
#include "UAV.h"
#include "Parameters.h"

extern COMMON::Common& common;
extern LOG::Logger& logger;

void Controller::run(){

    if (common.start_grvcopter() || common.reset_pid()){
        position_control.reset_pids();
        attitude_control.reset_pids();
    }

    //Resets Pids when arming and set offset angles.
    if (common.is_armed().rising_edge()){
        position_control.reset_pids();
        attitude_control.reset_pids();
        set_offset_angles();
        set_z_terrain();
        set_factor_throttle_to_fz();
        std::cout << "Armed" << std::endl;
    }

    if (common.is_armed().falling_edge()){
        std::cout << "Disarmed" << std::endl;
    }

    PARAMS::Params* params = common.get_params();
    if (params->check_param_changed()){
        params->update_angle_pid_constants(attitude_control.get_roll_ang_pid(), attitude_control.get_pitch_ang_pid(), attitude_control.get_yaw_ang_pid());
        params->update_rate_pid_constants(attitude_control.get_roll_rate_pid(), attitude_control.get_pitch_rate_pid(), attitude_control.get_yaw_rate_pid());
        params->update_pos_pid_constants(position_control.get_x_pos_pid(), position_control.get_y_pos_pid(), position_control.get_z_pos_pid());
        params->update_vel_pid_constants(position_control.get_x_vel_pid(), position_control.get_y_vel_pid(), position_control.get_z_vel_pid());
        params->pid_constants_updated();
    }

    Force force_xyz_n;
    Torques torques_nm;

    switch (common.get_mode())
    {
    case UAV::MODE_STABILIZE:
        run_stabilize_control(force_xyz_n, torques_nm);
        from_stabilize = true;
        break;
    case UAV::MODE_ALTITUDE:
        run_altitude_control(force_xyz_n, torques_nm);
        from_stabilize = false;
        break;
    case UAV::MODE_POSITION:
        run_position_control(force_xyz_n, torques_nm);
        from_stabilize = false;
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
    /*float force_motor[UAV::num_motors]{0.0};
    Mixer::get_forces_each_motor(&mixer, &force_xyz_n, &torques_nm, force_motor);

    float pwms[UAV::num_motors]{0.0};
    Mixer::force_to_pwm(force_motor, pwms);*/

    //Nuevo para mixer fisico:
    //force_xyz_n.x() *= 0.5;
    //force_xyz_n.y() *= 0.5;
    //torques_nm = torques_nm * 0.1;
    //torques_nm.yaw() *= 1.5;
    //force_xyz_n.z() *= 32.5;//Valor sacado experimentalmente para tener un valor equivalente de pwm con los dos mixers. 
    //Deberá verse reflejado en las constantes del controlador.

    float force_motor_phys[UAV::num_motors]{0.0};
    Mixer_Physical::get_forces_each_motor(&mixer_physical, &force_xyz_n, &torques_nm, force_motor_phys);

    float pwms_physical[UAV::num_motors]{0};
    mixer_physical.force_to_pwm(force_motor_phys, pwms_physical);
    //Fin nuevo mixer fisico.

    MSG_GRVCOPTER::Message_Bytes msg;
    MSG_GRVCOPTER::pack_pwm_message(UAV::num_motors, pwms_physical, &msg);
    char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};

    MSG_GRVCOPTER::get_bytes_msg(&msg, buffer);
    common.get_socket()->Send(buffer);

    for (uint8_t i = 0; i < UAV::num_motors; i++){
        logger.save_float_data(LOG_C::PWM1_ID+i, pwms_physical[i]);
    }

    //To log fisicos PWMs:
    /*for (uint8_t i = UAV::num_motors; i < (UAV::num_motors*2); i++){
        logger.save_float_data(LOG_C::PWM1_ID+i, pwms_physical[i-UAV::num_motors]);
    }*/

}

void Controller::run_stabilize_control(Force& forces, Torques& torques){
    compute_yaw_target_from_rc();
    compute_targets_from_rc_stabilize_control();
    Force force_xyz_n;
    force_xyz_n.z() = pow(common.get_rc()->get_throttle()+1,2)*factor_thr_fz;

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    if (UAV::fully_actuated){
        //Transform target attitude to XY forces:
        Force from_angles;
        from_angles = Attitude::get_forces_by_angles(*common.get_target_attitude());
        //The factor 0.5 was needed when changed to physical mixer.
        force_xyz_n.x() = from_angles[0]*0.5;
        force_xyz_n.y() = from_angles[1]*0.5;

        Attitude att_only_yaw = *common.get_target_attitude();

        att_only_yaw.roll() = offset_angles.roll();
        att_only_yaw.pitch() = offset_angles.pitch();

        
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

void Controller::run_altitude_control(Force& forces, Torques& torques){

    //Check last time altitude control has been running to reset the target altitude if necessary.
    static int64_t last_time = 0;
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if ((now - last_time) > 300){
        common.set_target_position(*common.get_current_position());
        if (from_stabilize) {
            set_hover_throttle();
        }
    }
    last_time = now;
    
    //Obtain targets:
    compute_yaw_target_from_rc();
    compute_targets_from_rc_alt_control();

    Acceleration accel_xyz_mss;
    Force force_xyz_n;
    Torques torques_nm;


    if (common.has_position()) {
        if (common.has_position().rising_edge()){
            std::cout << "Receiving position. Changed to Altitude Mode." << std::endl;
        }
        //std::cout << "Current Altitude: " << common.get_current_position()->z() << std::endl;

        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
        //Convert acceleration in forces with the UAV mass:
        force_xyz_n = accel_xyz_mss*UAV::mass_kg;
        

        //Rotate forces from fixed frame to UAV frame:
        force_xyz_n.rotate_to_uav_frame(common.get_current_attitude()->yaw());
        force_xyz_n.from_NED_to_NEU();
    }   else {
        if (common.has_position().falling_edge()){
            std::cout << "Lost position. Changed to Stabilize Mode." << std::endl;
        }
        run_stabilize_control(forces, torques);
        return;
    }

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;

    if (UAV::fully_actuated){
        //Transform target attitude to XY forces:
        Force from_angles;
        from_angles = Attitude::get_forces_by_angles(*common.get_target_attitude());
        //The factor 0.5 was needed when changed to physical mixer.
        force_xyz_n.x() = from_angles[0]*0.5;
        force_xyz_n.y() = from_angles[1]*0.5;

        Attitude att_only_yaw = *common.get_target_attitude();
        att_only_yaw.roll() = offset_angles.roll();
        att_only_yaw.pitch() = offset_angles.pitch();
        
        ang_accel_des = attitude_control.run(&att_only_yaw, common.get_current_attitude(), common.get_current_rate());

    } else {
        //Cancel XY forces, only altitude control:
        force_xyz_n[0] = 0.0;
        force_xyz_n[1] = 0.0;
        ang_accel_des = attitude_control.run(common.get_target_attitude(), common.get_current_attitude(), common.get_current_rate());
    }
    //Compute the desired torques given the UAV inertia
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;
    
    //Not necessary with the physical mixer:
    //Mixer::normalize_force_by_mass(force_xyz_n);

    //Add feedforward from RC controller as Stabilize control. This way there is no initial gap when switching from stabilize to altitude.
    force_xyz_n.z() += hover_throttle;
    //force_xyz_n.apply_limits(100.0, 100.0, UAV::mass_kg*GRAVITY*2);
    force_xyz_n.z() = saturation(force_xyz_n.z(), (float) (0.5*UAV::mass_kg*GRAVITY), (float)(1.5*UAV::mass_kg*GRAVITY));
    //Nuevo:
    //force_xyz_n.z() = saturation(force_xyz_n.z(), (float) (stabilize_throttle*0.8), (float)(stabilize_throttle*1.5));
    

    forces = force_xyz_n;

    //std::cout << "Force Z Altitude: " << forces.z() << "   Stabilize Throttle: " << stabilize_throttle << std::endl;
    torques = torques_nm;
}

void Controller::run_position_control(Force& forces, Torques& torques){

    //Check last time position control has been running to reset the target position if necessary.
    static int64_t last_time = 0;
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if ((now - last_time) > 300){
        common.set_target_position(*common.get_current_position());
        if (from_stabilize) {
            set_hover_throttle();
        }
    }
    last_time = now;

    //Obtain targets.
    compute_yaw_target_from_rc();
    compute_targets_from_rc_pos_control();

    //std::cout << "Targets PosX:" << common.get_current_position()->x() << "PosY:" << common.get_current_position()->y() << std::endl;

    Acceleration accel_xyz_mss;
    Force force_xyz_n;
    Torques torques_nm;

    if (common.has_position()) {
        if (common.has_position().rising_edge()){
            std::cout << "Receiving position. Changed to Position Mode." << std::endl;
        }
        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
        //Convert acceleration in forces with the UAV mass:
        force_xyz_n = accel_xyz_mss*UAV::mass_kg;
        //Rotate forces from fixed frame to UAV frame:

        force_xyz_n.rotate_to_uav_frame(common.get_current_attitude()->yaw());
        force_xyz_n.from_NED_to_NEU();
        force_xyz_n.limit_to_xy(100);
    }   else {
        if (common.has_position().falling_edge()){
            std::cout << "Lost position. Changed to Stabilize Mode." << std::endl;
        }
        run_stabilize_control(forces, torques);
        return;
    }
    
    //Compute the desired attitude from the position controller.
    Attitude attitude_from_position_controller;
    if (UAV::fully_actuated) {
        //If fully actuated, set the attitude target to the offset_angles.
        attitude_from_position_controller.roll() = offset_angles.roll();
        attitude_from_position_controller.pitch() = offset_angles.pitch();    
    } else {
        //Get the angles from the forces XY.
        attitude_from_position_controller = Attitude::get_angles_from_forces(force_xyz_n);
        //Limit lean angle to UAV::max_lean_angle.
        attitude_from_position_controller = Attitude::limit_lean_angle_rad(attitude_from_position_controller, UAV::max_lean_angle);
    }
    //Set the Yaw target
    attitude_from_position_controller.yaw() = common.get_target_attitude()->yaw();

    //Compute the desired angular acceleration.
    Angular_Acceleration ang_accel_des;
    ang_accel_des = attitude_control.run(&attitude_from_position_controller, common.get_current_attitude(), common.get_current_rate());

    //Compute the desired torques given the UAV inertia
    torques_nm = ang_accel_des*UAV::inertia_kg_m2;
    
    //Normalize the forces by mass.
    Mixer::normalize_force_by_mass(force_xyz_n);

    //Add feedforward from RC controller as Stabilize control. This way there is no initial gap when switching from stabilize to altitude.
    force_xyz_n.z() += hover_throttle;
    force_xyz_n.apply_limits(2.5, 2.5, 1.0);
    //Nuevo:
    force_xyz_n.z() = saturation(force_xyz_n.z(), (float) (hover_throttle*0.9), (float)(hover_throttle*1.5));
       
    
    forces = force_xyz_n;
    torques = torques_nm;
}


//@brief Function to obtain the target attitude Roll and Pitch in Stabilize Mode.
void Controller::compute_targets_from_rc_stabilize_control(){
    RC* rc = common.get_rc();
    Attitude* att_target = common.get_target_attitude();
    att_target->roll() =  Attitude::from_degrees_to_rad(-UAV::max_lean_angle*(rc->get_channel(CH_ROLL)));
    att_target->pitch() = Attitude::from_degrees_to_rad(UAV::max_lean_angle*(rc->get_channel(CH_PITCH)));
}

//@brief Function to obtain the target altitude and attitude in Altitude Hold Mode.
void Controller::compute_targets_from_rc_alt_control(){
    RC* rc = common.get_rc();
    float z_speed_NED = -rc->get_z_speed();

    //Apply dead band to maintain a constant altitude
    z_speed_NED = dead_band(z_speed_NED, UAV::dead_band_rc);
    moving_z_pos = abs(z_speed_NED) > UAV::dead_band_rc;

    //Update target altitude
    Position* target = common.get_target_position();
    target->z() = target->z() + z_speed_NED*DT*0.2;

    if (moving_z_pos.falling_edge()){
        std::cout << "Reset Z Altitude" << std::endl;
        //target->z() = common.get_current_position()->z();
    }

    target->z() = saturation(target->z(), (float)-100.0, (float)(z_terrain+0.2));
    //std::cout << "Target Altitude: " << target->z() << std::endl;

    //Control the attitude as in stabilize mode.
    Attitude* att_target = common.get_target_attitude();
    att_target->roll() = Attitude::from_degrees_to_rad(-UAV::max_lean_angle*rc->get_channel(CH_ROLL));
    att_target->pitch() = Attitude::from_degrees_to_rad(UAV::max_lean_angle*rc->get_channel(CH_PITCH));
}

//@brief Function to obtain the target position in Position Hold Mode.
void Controller::compute_targets_from_rc_pos_control(){
    //Get RC values.
    RC* rc = common.get_rc();
    float X_speed_NED = -rc->get_channel(CH_PITCH);
    float Y_speed_NED = rc->get_channel(CH_ROLL);
    float Z_speed_NED = -rc->get_z_speed();


    //Apply dead band to maintain a constant position.
    X_speed_NED = dead_band(X_speed_NED, UAV::dead_band_rc);
    moving_x_pos = abs(X_speed_NED) >= UAV::dead_band_rc;

    Y_speed_NED = dead_band(Y_speed_NED, UAV::dead_band_rc);
    moving_y_pos = abs(Y_speed_NED) >= UAV::dead_band_rc;

    Z_speed_NED = dead_band(Z_speed_NED, UAV::dead_band_rc);
    moving_z_pos = abs(Z_speed_NED) > UAV::dead_band_rc;


    //Update targets position.
    Position* target = common.get_target_position();
    Position* current = common.get_current_position();

    target->x() = target->x() + X_speed_NED*DT;
    //Stop moving in X if stick is in the middle.
    if (moving_x_pos.falling_edge()){
        target->x() = current->x();
    }

    target->y() = target->y() + Y_speed_NED*DT;
    //Stop moving in Y if stick is in the middle.
    if (moving_y_pos.falling_edge()){
        target->y() = current->y();
    }


    target->z() = target->z() + Z_speed_NED*DT*0.1;
    
    /*if (moving_z_pos.falling_edge()){
        target->z() = current->z();
    }*/
}


//@brief Function to obtain the target yaw from RC. For many flight modes.
void Controller::compute_yaw_target_from_rc(){
    RC* rc = common.get_rc();
    float yaw_speed = rc->get_channel(CH_YAW);
    //Apply dead band to avoid undesired yaw rotation due to small values in RC.
    yaw_speed = dead_band(yaw_speed, UAV::dead_band_width_yaw_rc);
    //Update target yaw angle:
    Attitude* att_target = common.get_target_attitude();

    att_target->yaw() = wrap_PI(att_target->yaw() + Attitude::from_degrees_to_rad(yaw_speed));
}

//@brief Function to set the current attitude as an angle offset. This should be called when arming to set the desired attitude as the current attitude. 
//Very useful in Fully Actuated UAVs and to takeoff with the current yaw and avoid to spin when taking off.
void Controller::set_offset_angles(){

    //Get current attitude and save it in the offset_angles.
    common.get_current_attitude(offset_angles);
    //Set the target yaw as the current yaw.
    common.get_target_attitude()->yaw() = offset_angles.yaw();

    if (!UAV::fully_actuated){
        offset_angles.roll() = 0.0;
        offset_angles.pitch() = 0.0;
    }
    std::cout << "Offset set to R: " << Attitude::from_rad_to_degrees(offset_angles.roll()) << " P: " << Attitude::from_rad_to_degrees(offset_angles.pitch()) << " Y:" <<  Attitude::from_rad_to_degrees(offset_angles.yaw()) << std::endl;
}

void Controller::set_hover_throttle(){
    hover_throttle = pow(common.get_rc()->get_throttle()+1,2)*factor_thr_fz;
}

void Controller::set_z_terrain(){
    if (common.has_position()){
        z_terrain = common.get_current_position()->z();
        std::cout << "Set Z terrain to: " << z_terrain << std::endl; 
    }
}

void Controller::set_factor_throttle_to_fz(){
    factor_thr_fz = common.get_params()->get_param_value(PARAMS::factor_throttle_to_fz);
}