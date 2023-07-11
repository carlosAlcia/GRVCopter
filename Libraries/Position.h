//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Vector3.h"
#include "Attitude.h"
#include "Helper.h"

class Position : public Vector{
    public: 
    Position():Vector(){}

    Position(float x, float y, float z):Vector(x, y, z){}

    Position(float* _pos):Vector(_pos){}

    virtual void operator=(Vector _v) override {
        _x = _v[0];
        _y = _v[1];
        _z = _v[2];
    }

    virtual void operator=(float* pos) override {
        _x = pos[0];
        _y = pos[1];
        _z = pos[2];
    }

    float& x(){return _x;}
    float& y(){return _y;}
    float& z(){return _z;}

    //@brief Function to change the stored values in cm to meters. It divides each element by 100.
    void from_cm_to_m(){
        _x /= 100.0;
        _y /= 100.0;
        _z /= 100.0;
    }

    //@brief Function to rotate a vector in XY plane given Yaw.
    //@param yaw: [float] Current yaw in Rad.
    void rotate_to_uav_frame(float yaw) {

        Vector vec;
        vec[0] = this->_x*cos(yaw) + this->_y*sin(yaw); 
        vec[1] = -this->_x*sin(yaw) + this->_y*cos(yaw); 
        vec[2] = this->_z;
        *this = vec;
    }

    //@brief Function to pass from NEU to NED.
    void from_NEU_to_NED() {
        this->_z *= -1; 
    }

    //@brief Function to pass from NED to NEU.
    void from_NED_to_NEU() {
        this->_z *= -1; 
    }

    //@brief Function to saturate each component of vector to each limit.
    void apply_limits(float x_limit, float y_limit, float z_limit){
        saturation_and_check(this->_x, -x_limit, x_limit);
        saturation_and_check(this->_y, -y_limit, y_limit);
        saturation_and_check(this->_z, -z_limit, z_limit);
    }



};

//A 3D vector containing velocities in m/s and NED.
class Velocity : public Position {
    public:
        Velocity():Position(){};
        Velocity(float x, float y, float z):Position(x, y, z){}
        Velocity(float* _vel):Position(_vel){}

    void operator=(Vector _v) override {
        _x = _v[0];
        _y = _v[1];
        _z = _v[2];
        }

    void operator=(float* pos) override {
        _x = pos[0];
        _y = pos[1];
        _z = pos[2];
    }

    
};

//A 3D vector containing accelerations in m/ss
class Acceleration : public Position {
    public:
        Acceleration():Position(){};
        Acceleration(float x, float y, float z):Position(x, y, z){}
        Acceleration(float* _vel):Position(_vel){}

    void operator=(Vector _v) override {
        _x = _v[0];
        _y = _v[1];
        _z = _v[2];
        }

    void operator=(float* pos) override {
        _x = pos[0];
        _y = pos[1];
        _z = pos[2];
    }
};

//A 3D vector containing forces in Newtons.
class Force : public Position {
    public:
        Force():Position(){};
        Force(float x, float y, float z):Position(x, y, z){}
        Force(float* _vel):Position(_vel){}

    void operator=(Vector _v) override {
        _x = _v[0];
        _y = _v[1];
        _z = _v[2];
        }

    void operator=(float* pos) override {
        _x = pos[0];
        _y = pos[1];
        _z = pos[2];
    }

    //@brief Limit the max force in XY required by the position controller.
    //@param Value max in Newtons
    void limit_to_xy(float max_force_xy){
        saturation_and_check(this->x(), -max_force_xy, max_force_xy);
        saturation_and_check(this->y(), -max_force_xy, max_force_xy);
    }

};