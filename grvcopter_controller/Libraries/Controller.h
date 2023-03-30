//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Attitude_Controller.h"
#include "Position_Controller.h"
#include <iostream>

class Controller {
    private:
        AttitudeControl attitude_control;
        PositionControl position_control;

    public: 
        Controller(){}

        void run();
};