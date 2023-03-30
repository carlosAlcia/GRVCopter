//Created by Carlos Álvarez Cía 2023

#pragma once

#include "math.h"

class Position;
class Attitude;


class Vector {

    protected:
        float _x, _y, _z;

    public:
        
        Vector(){
            _x = 0.0;
            _y = 0.0;
            _z = 0.0;
        }

        Vector(float __x, float __y, float __z){
            _x = __x;
            _y = __y;
            _z = __z;
        }

        Vector(float* _pos){
            _x = _pos[0];
            _y = _pos[1];
            _z = _pos[2];
        }

        void operator=(Vector pos){
            _x = pos._x;
            _y = pos._y;
            _z = pos._z;
        }

        void operator=(float* pos){
            _x = pos[0];
            _y = pos[1];
            _z = pos[2];
        }

        float& operator[](int index){
            switch (index)
            {
            case 0:
                return _x;
                break;
            case 1:
                return _y;
                break;
            case 2:
                return _z;
                break;

            default:
                throw("Index out of bounds.");
                break;
            }
        }

        //Compute the error in each axis between two vectors.
        //@param Vector* target
        //@param Vector* current
        //@returns Vector* error = Target-Current. 
        static Vector error(Vector* target, Vector* current){
            Vector error;
            error._x = target->_x - current->_x;
            error._y = target->_y - current->_y;
            error._z = target->_z - current->_z;
            return error;
        }

        operator Position();

        operator Attitude();

};
