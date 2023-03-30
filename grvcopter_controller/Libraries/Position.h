//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Vector3.h"

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

};

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