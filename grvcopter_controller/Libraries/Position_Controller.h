//Created by Carlos Álvarez Cía 2023

#pragma once

#include "PID.h"
#include "Position.h"

class PositionControl {
    private:
        PID pid_x = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_y = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);
        PID pid_z = PID(DEFAULT_KP_PID, DEFAULT_KI_PID, DEFAULT_KD_PID);

    public:
        PositionControl(){}

        //@brief Function to set all the integral terms to 0.
        void reset_pids(){
            pid_x.reset_i();
            pid_y.reset_i();
            pid_z.reset_i();
        }

        //@brief Function to reset the integral term of PID X.
        void reset_pid_x(){
            pid_x.reset_i();
        }

        //@brief Function to reset the integral term of PID Y.
        void reset_pid_y()
        {
            pid_y.reset_i();
        }
        //@brief Function to reset the integral term of PID Z.
        void reset_pid_z()
        {
            pid_z.reset_i();
        }

        //@brief Compute the desired forces in each axis given a target position and the current position.
        //@param Position* target
        //@param Position* current
        //@returns Position forces = PID(Target-Current). 
        Position run(Position *target, Position *current);

};