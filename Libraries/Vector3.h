//Created by Carlos Álvarez Cía 2023

#pragma once

#include "math.h"


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

        virtual void operator=(Vector pos){
            _x = pos._x;
            _y = pos._y;
            _z = pos._z;
        }

        virtual void operator=(float* pos){
            _x = pos[0];
            _y = pos[1];
            _z = pos[2];
        }

        template <class T>
        Vector operator*(T factor){
            Vector result;
            result[0] = this->_x*factor;
            result[1] = this->_y*factor;
            result[2] = this->_z*factor;
            return result;
        }

        Vector operator+(Vector add){
            Vector result;
            result[0] = this->_x+add[0];
            result[1] = this->_y+add[1];
            result[2] = this->_z+add[2];
            return result;
        }

        //Element wise multiplication.
        Vector operator*(Vector factors){
            Vector result;
            result[0] = this->_x*factors[0];
            result[1] = this->_y*factors[1];
            result[2] = this->_z*factors[2];
            return result;
        }

        template <class T>
        Vector operator/(T factor){
            Vector result;
            result[0] = this->_x/factor;
            result[1] = this->_y/factor;
            result[2] = this->_z/factor;
            return result;
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

        //@brief Normalize vector between -1:1.
        void normalize_sign() {
            float max = _x;
            float min = _x;
            if (_y > max){
                max = _y;
            }
            if (_y < min){
                min = _y;
            }
            if (_z > max){
                max = _z;
            }
            if (_z < min){
                min = _z;
            }
            *this = *this/(max-min);
        
        }

        //@brief Get the euclidean norm of a 3D vector.
        float norm_euclidean(){
            float x_2 = pow(this->_x, 2);
            float y_2 = pow(this->_y, 2);
            float z_2 = pow(this->_z, 2);
            return sqrt(x_2 + y_2 + z_2);
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

};
