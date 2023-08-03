#pragma once
#include "Position.h"
#include "Attitude.h"
#include "Common.h"
#include "UAV.h"
#include "Matriz3.h"

extern COMMON::Common& common;

constexpr float KI_lin_esc {5};
constexpr float KI_ang_esc {2};


class Wrench_Estimator {

    private: 

        Vector int_vec_lin_;
        Vector int_vec_ang_;
        Vector ext_linear_accel_;
        Vector ext_angular_accel_;
        


    public: Wrench_Estimator(){};

    public: void initialize(){
        int_vec_lin_.zero();
        int_vec_ang_.zero();
        ext_angular_accel_.zero();
        ext_linear_accel_.zero();
    }

    private: inline Matriz3 to_rotation_matrix(Quaternion quat){

        float yy = quat.qy()*quat.qy();
        float yz = quat.qy()*quat.qz();
        float xx = quat.qx()*quat.qx();
        float xy = quat.qx()*quat.qy();
        float xz = quat.qx()*quat.qz();
        float wx = quat.qw()*quat.qx();
        float wy = quat.qw()*quat.qy();
        float wz = quat.qw()*quat.qz();
        float zz = quat.qz()*quat.qz();

        Matriz3 m;
        m.a()[0] = 1.0-2.0*(yy + zz);
        m.a()[1] =   2.0*(xy - wz);
        m.a()[2] =   2.0*(xz + wy);
        m.b()[0] =   2.0*(xy + wz);
        m.b()[1] = 1.0-2.0*(xx + zz);
        m.b()[2] =   2.0*(yz - wx);
        m.c()[0] =   2.0*(xz - wy);
        m.c()[1] =   2.0*(yz + wx);
        m.c()[2] = 1.0-2.0*(xx + yy);
        return m;
    }

    

    public: bool get_wrench_estimation(Force& _forces_est, Torques& _torques_est){

        
        Force force_command(0,0,0);
        Torques torque_command(0,0,0);

        force_command = common.get_last_force_controller_out();
        torque_command = common.get_last_torques_controller_out();

        //Next, we need odometry:
        Quaternion quat;
        Attitude current_attitude;
        current_attitude = *common.get_current_attitude();
        quat = current_attitude.to_quaternion();

        //NED to NWU:
        quat.qy() = -quat.qy();
        quat.qz() = -quat.qz();

        Matriz3 rotation_mat = to_rotation_matrix(quat); //Está bien, da los giros en XYZ correctamente en NWU.
        
        Vector grav_B = rotation_mat.transposed()*(9.81)*Vector(0,0,1); //Está bien.

        Velocity lin_vel_B;
        if(!common.has_position()) return false;
        //NED to NWU:
        lin_vel_B = *common.get_current_vel();
        lin_vel_B.y() = -lin_vel_B.y();
        lin_vel_B.z() = -lin_vel_B.z(); 

        Rate ang_vel_B;
        ang_vel_B = *common.get_current_rate();
        //NED to NWU:
        ang_vel_B.pitch() = -ang_vel_B.pitch();
        ang_vel_B.yaw() = -ang_vel_B.yaw();

        //Compute non linear terms:
        Vector inertia = UAV::inertia_kg_m2;
        Matriz3 UAV_INERTIA_MAT = Matriz3(Vector(inertia[0] , 0, 0), Vector(0, inertia[1], 0), Vector(0, 0, inertia[2]));
        
        //Voy por aqui
        Vector n_ang = UAV_INERTIA_MAT.inverse_as_diagonal()*((UAV_INERTIA_MAT*(-ang_vel_B)).cross(ang_vel_B));
        Vector n_lin = ang_vel_B.cross(lin_vel_B) + grav_B;

        Vector cmd_linear_accel = force_command/UAV::mass_kg;   //Fuerza/Masa = Aceleracion.
        
        Vector cmd_angular_accel = UAV_INERTIA_MAT.inverse_as_diagonal()*torque_command;
        int_vec_lin_ += (cmd_linear_accel + ext_linear_accel_ - n_lin) * 1/400;
        int_vec_ang_ += (cmd_angular_accel + ext_angular_accel_ - n_ang) * 1/400;

        Vector KI_lin = Vector(1,1,1)*KI_lin_esc;
        Vector KI_ang = Vector(1,1,1)*KI_ang_esc;

        _forces_est.x() = UAV::mass_kg*(KI_lin[0]*(lin_vel_B[0]-int_vec_lin_[0]));
        _forces_est.y() = UAV::mass_kg*(KI_lin[1]*(lin_vel_B[1]-int_vec_lin_[1]));
        _forces_est.z() = UAV::mass_kg*(KI_lin[2]*(lin_vel_B[2]-int_vec_lin_[2]));
        ext_linear_accel_ = _forces_est / UAV::mass_kg;
        _torques_est.roll() = (KI_ang[0]*(ang_vel_B[0]-int_vec_ang_[0]));
        _torques_est.pitch() = (KI_ang[1]*(ang_vel_B[1]-int_vec_ang_[1]));
        _torques_est.yaw() = (KI_ang[2]*(ang_vel_B[2]-int_vec_ang_[2]));
        ext_angular_accel_= _torques_est;
        _torques_est = UAV_INERTIA_MAT*_torques_est;


        return true;

    }

};