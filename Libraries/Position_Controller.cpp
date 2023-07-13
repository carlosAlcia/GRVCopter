#include "Position_Controller.h"
#include "UAV.h"
#include "Logger.h"
#include "Logger_Constants.h"

extern LOG::Logger& logger;

Acceleration PositionControl::run(Position *target, Position *current, Position *current_vel){
    
    //Error in position
    Position error_p;
    error_p = Position::error(target, current);
    //Limit max error to avoid aggressive control.
    error_p.apply_limits(0.2, 0.2, 0.2);

    //Compute linear velocities desired:
    Velocity vel_des_from_pid;
    vel_des_from_pid.x() = pid_x_pos.update_pid(error_p.x());
    vel_des_from_pid.y() = pid_y_pos.update_pid(error_p.y());
    vel_des_from_pid.z() = pid_z_pos.update_pid(error_p.z());

    

    //std::cout << "Target Altitude: " << target->z() << "Error: " << error_p.z() << std::endl;

    Acceleration feedforward;
    feedforward = vel_des_from_pid*Vector(ff_pox_xy, ff_pox_xy, ff_pos_z);

    //Units in m/s
    //vel_des_from_pid.apply_limits(10.0, 10.0, 0.8);

    //Compute linear accelerations desired:
    Velocity error_v;

    logger.save_float_data(LOG_C::VEL_DES_X, vel_des_from_pid.x());
    logger.save_float_data(LOG_C::VEL_DES_Y, vel_des_from_pid.y());
    logger.save_float_data(LOG_C::VEL_DES_Z, vel_des_from_pid.z());
    logger.save_float_data(LOG_C::VEL_CUR_X, current_vel->x());
    logger.save_float_data(LOG_C::VEL_CUR_Y, current_vel->y());
    logger.save_float_data(LOG_C::VEL_CUR_Z, current_vel->z());

    error_v = Velocity::error(&vel_des_from_pid, current_vel);
    //error_v.apply_limits(0.5, 0.5, 0.8);
    //std::cout << "Vel Z desired: " << vel_des_from_pid.z() << "Error: " << error_p.z() << std::endl;

    
    Acceleration accel_des_from_pid;
    accel_des_from_pid.x() = pid_x_vel.update_pid(error_v.x());
    accel_des_from_pid.y() = pid_y_vel.update_pid(error_v.y());
    accel_des_from_pid.z() = pid_z_vel.update_pid(error_v.z());

    logger.save_float_data(LOG_C::INT_POS_X, pid_x_vel.get_I());
    logger.save_float_data(LOG_C::INT_POS_Y, pid_y_vel.get_I());
    logger.save_float_data(LOG_C::INT_POS_Z, pid_z_vel.get_I());
    //std::cout << "INTEGRAL: " << pid_z_vel.get_I() << std::endl;


    //Units in cm/s²
    accel_des_from_pid = accel_des_from_pid + feedforward;
    //accel_des_from_pid.apply_limits(20.0, 20.0, GRAVITY*1.5);

    //std::cout << "Accel Z desired: " << accel_des_from_pid.z() << "    Error: " << error_p.z() << std::endl;


    return accel_des_from_pid;

}