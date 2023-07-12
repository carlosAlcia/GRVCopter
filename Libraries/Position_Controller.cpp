#include "Position_Controller.h"
#include "UAV.h"

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
    feedforward = vel_des_from_pid*Vector(0.5, 0.5, 4.0);

    //Units in m/s
    vel_des_from_pid.apply_limits(10.0, 10.0, 0.8);

    //Compute linear accelerations desired:
    Velocity error_v;
    error_v = Velocity::error(&vel_des_from_pid, current_vel);
    error_v.apply_limits(0.5, 0.5, 0.8);
    //std::cout << "Vel Z desired: " << vel_des_from_pid.z() << "Error: " << error_p.z() << std::endl;

    
    Acceleration accel_des_from_pid;
    accel_des_from_pid.x() = pid_x_vel.update_pid(error_v.x());
    accel_des_from_pid.y() = pid_y_vel.update_pid(error_v.y());
    accel_des_from_pid.z() = pid_z_vel.update_pid(error_v.z());

    //std::cout << "INTEGRAL: " << pid_z_vel.get_I() << std::endl;


    //Units in cm/s²
    accel_des_from_pid = accel_des_from_pid + feedforward;
    accel_des_from_pid.apply_limits(20.0, 20.0, GRAVITY*1.5);

    //std::cout << "Accel Z desired: " << accel_des_from_pid.z() << "    Error: " << error_p.z() << std::endl;


    return accel_des_from_pid;

}