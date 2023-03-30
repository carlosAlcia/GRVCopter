#include "Position_Controller.h"

Acceleration PositionControl::run(Position *target, Position *current, Position *current_vel){
    
    //Error in position
    Position error_p;
    error_p = Position::error(target, current);

    //Compute linear velocities desired:
    Velocity vel_des_from_pid;
    vel_des_from_pid.x() = pid_x_pos.update_pid(error_p.x());
    vel_des_from_pid.y() = pid_y_pos.update_pid(error_p.y());
    vel_des_from_pid.z() = pid_z_pos.update_pid(error_p.z());

    //Compute linear accelerations desired:
    Velocity error_v;
    error_v = Velocity::error(&vel_des_from_pid, current_vel);
    
    Acceleration accel_des_from_pid;
    accel_des_from_pid.x() = pid_x_vel.update_pid(error_v.x());
    accel_des_from_pid.y() = pid_y_vel.update_pid(error_v.y());
    accel_des_from_pid.z() = pid_z_vel.update_pid(error_v.z());

    int a = 0;
    return accel_des_from_pid;

}