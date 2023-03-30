//Created by Carlos Álvarez Cía 2023

#include "Vector3.h"

class Position : public Vector3::Vector{
    public: 
    Position(){
        _x = 0.0;
        _y = 0.0;
        _z = 0.0;
    }

    Position(float _x, float _y, float _z){
        _x = _x;
        _y = _y;
        _z = _z;
    }

    Position(float* _pos){
        _x = _pos[0];
        _y = _pos[1];
        _z = _pos[2];
    }

    float& x(){return _x;}
    float& y(){return _y;}
    float& z(){return _z;}

};