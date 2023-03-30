#include "Vector3.h"
#include "Attitude.h"
#include "Position.h"


    Vector::operator Position(){
        Position _pos = Position(this->_x, this->_y, this->_z);
        return _pos;
    }

    Vector::operator Attitude(){
        Attitude _att = Attitude(this->_x, this->_y, this->_z);
        return _att;
    }
