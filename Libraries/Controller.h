//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Attitude_Controller.h"
#include "Position_Controller.h"
#include "Logger.h"
#include "Mixer.h"
#include "UAV.h"
#include <iostream>
#include "Logger_Constants.h"
#include <chrono>
#include "Attitude.h"
#include "Bool_Upgraded.h"
#include "Mixer_Physical.h"

class Controller {
    private:
        AttitudeControl attitude_control;
        PositionControl position_control;
        Mixer mixer;

        Mixer_Physical mixer_physical;

        //Nuevo mixer fisico:


        //Offset angles in Rad. 
        Attitude offset_angles;

        //Ebool to know if the pilot want to move in X.
        Ebool moving_x_pos;

        //Ebool to know if the pilot want to move in Y.
        Ebool moving_y_pos;

        //Ebool to know if the pilot want to move in Z.
        Ebool moving_z_pos;

        //Ebool to know if the previous flight mode was stabilize.
        Ebool from_stabilize;

        //Throttle used in Stabilize before switching to Alt Hold or Pos Hold mode. Give an estimation of the force Z for hover. Used to avoid jumps in Z when switching mode.
        float hover_throttle = 0.0;

        //Altitude Z of the terrain. Set when the UAV arms.
        float z_terrain = 0.0;


    public: 
        Controller(){
            mixer.set_roll_factors(UAV::roll_factor);
            mixer.set_pitch_factors(UAV::pitch_factor);
            mixer.set_yaw_factors(UAV::yaw_factor);
            mixer.set_x_factors(UAV::x_factor);
            mixer.set_y_factors(UAV::y_factor);
            mixer.set_z_factors(UAV::z_factor);

            //Nuevo mixer físico:
            mixer_physical.set_roll_factors(UAV::roll_factor_fis);
            mixer_physical.set_pitch_factors(UAV::pitch_factor_fis);
            mixer_physical.set_yaw_factors(UAV::yaw_factor_fis);
            mixer_physical.set_x_factors(UAV::x_factor_fis);
            mixer_physical.set_y_factors(UAV::y_factor_fis);
            mixer_physical.set_z_factors(UAV::z_factor_fis);
        }

        void run();

        void run_stabilize_control(Force& forces, Torques& torques);
        void run_altitude_control(Force& forces, Torques& torques);
        void run_position_control(Force& forces, Torques& torques);

        void compute_targets_from_rc_stabilize_control();
        void compute_targets_from_rc_pos_control();
        void compute_targets_from_rc_alt_control();
        void compute_yaw_target_from_rc();

        void set_offset_angles();
        void set_hover_throttle();

        void set_z_terrain();


        
};