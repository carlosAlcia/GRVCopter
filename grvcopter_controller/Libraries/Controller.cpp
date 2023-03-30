//Created by Carlos Álvarez Cía 2023

#include "Controller.h"
#include "Common.h"

extern COMMON::Common& common;

void Controller::run(){
    std::cout << common.get_rc()->get_channel(CH_THROTTLE) << std::endl;
}