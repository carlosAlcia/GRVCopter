//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Attitude_Controller.h"
#include "Position_Controller.h"
#include "Logger.h"
#include "Mixer.h"
#include "UAV.h"
#include <iostream>
#include "Logger_Constants.h"

class Controller {
    private:
        AttitudeControl attitude_control;
        PositionControl position_control;
        Mixer mixer;


    public: 
        Controller(){
            mixer.set_roll_factors(UAV::roll_factor);
            mixer.set_pitch_factors(UAV::pitch_factor);
            mixer.set_yaw_factors(UAV::yaw_factor);
            mixer.set_x_factors(UAV::x_factor);
            mixer.set_y_factors(UAV::y_factor);
            mixer.set_z_factors(UAV::z_factor);
        }

        void run();

        void run_stabilize_control(Force& forces, Torques& torques);
        void run_altitude_control(Force& forces, Torques& torques);
        void run_position_control(Force& forces, Torques& torques);

        void compute_targets_from_rc_pos_control();
        void compute_targets_from_rc_alt_control();


        
};