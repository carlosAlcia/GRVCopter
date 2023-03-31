//Created by Carlos Álvarez Cía 2023

#pragma once

#include "PID.h"
#include "Position.h"

class PositionControl {
    private:
        PID pid_x_vel = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_y_vel = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_z_vel = PID(DEFAULT_KP_PID*5, DEFAULT_KI_PID, DEFAULT_KD_PID);

        PID pid_x_pos = PID(DEFAULT_KP_PID, 0, 0);
        PID pid_y_pos = PID(DEFAULT_KP_PID, 0, 0);
        PID pid_z_pos = PID(DEFAULT_KP_PID*5, 0, 0);

    public:
        PositionControl(){}

        //@brief Function to set all the integral terms to 0.
        void reset_pids(){
            pid_x_vel.reset_i();
            pid_y_vel.reset_i();
            pid_z_vel.reset_i();
            pid_x_pos.reset_i();
            pid_y_pos.reset_i();
            pid_z_pos.reset_i();
        }

        //@brief Function to reset the integral term of PID X.
        void reset_pid_x(){
            pid_x_pos.reset_i();
            pid_x_vel.reset_i();
        }

        //@brief Function to reset the integral term of PID Y.
        void reset_pid_y()
        {
            pid_y_pos.reset_i();
            pid_y_vel.reset_i();
        }
        //@brief Function to reset the integral term of PID Z.
        void reset_pid_z()
        {
            pid_z_vel.reset_i();
            pid_z_pos.reset_i();
        }

        //@brief Compute the desired forces in each axis given a target position and the current position.
        //@param Position* target in m.
        //@param Position* current in m.
        //@param Position* current_vel : Current vel in m/s.
        //@returns Acceleration forces = PID(Target-Current). 
        Acceleration run(Position *target, Position *current, Position* current_vel);

};