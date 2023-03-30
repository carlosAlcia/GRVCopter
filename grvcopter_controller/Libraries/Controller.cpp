//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"

extern COMMON::Common& common;

void Controller::run(){

    Acceleration accel_xyz_mss;
    if (common.has_position()) {
        accel_xyz_mss = position_control.run(common.get_target_position(), common.get_current_position(), common.get_current_vel());
    } 
    
}