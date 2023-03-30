//Created by Carlos Álvarez Cía 2023

#include "Vector3.h"


class Quaternion {

    private:
        float _qx, _qy, _qz, _qw;

    public:
        Quaternion(){
            _qw = 0;
            _qx = 0;
            _qy = 0;
            _qz = 0;
        };

        Quaternion(float qw, float qx, float qy, float qz){
            _qw = qw;
            _qx = qx;
            _qy = qy;
            _qz = qz;
        };

        float& qx(){return _qx;}
        float& qy(){return _qy;}
        float& qz(){return _qz;}
        float& qw(){return _qw;}

};

class Attitude : public Vector{
    public:         
    
    Attitude():Vector(){}

    Attitude(float roll, float pitch, float yaw):Vector(roll, pitch, yaw){}

    Attitude(float* _pos):Vector(_pos){}

    float& roll(){return _x;}
    float& pitch(){return _y;}
    float& yaw (){return _z;}

    Quaternion to_quaternion(Attitude &att){
        Quaternion quat;
        const float cr2 = cos(att.roll()*0.5);
        const float cp2 = cos(att.pitch()*0.5);
        const float cy2 = cos(att.yaw()*0.5);
        const float sr2 = sin(att.roll()*0.5);
        const float sp2 = sin(att.pitch()*0.5);
        const float sy2 = sin(att.yaw()*0.5);

        quat.qw() = cr2*cp2*cy2 + sr2*sp2*sy2;
        quat.qx() = sr2*cp2*cy2 - cr2*sp2*sy2;
        quat.qy() = cr2*sp2*cy2 + sr2*cp2*sy2;
        quat.qz() = cr2*cp2*sy2 - sr2*sp2*cy2;
        return quat;
    }

    Attitude to_euler(Quaternion &q){
        Attitude att;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.qw() * q.qx() + q.qy() * q.qz());
        double cosr_cosp = 1 - 2 * (q.qx() * q.qx() + q.qy() * q.qy());
        att.roll() = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (q.qw() * q.qy() - q.qx() * q.qz()));
        double cosp = std::sqrt(1 - 2 * (q.qw() * q.qy() - q.qx() * q.qz()));
        att.pitch() = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.qw() * q.qz() + q.qx() * q.qy());
        double cosy_cosp = 1 - 2 * (q.qy() * q.qy() + q.qz() * q.qz());
        att.yaw() = std::atan2(siny_cosp, cosy_cosp);
        return att;
    }



};

class Rate : public Attitude{
    public:
        Rate():Attitude(){}

        Rate(float roll, float pitch, float yaw):Attitude(roll, pitch, yaw){}

        Rate(float* _rate):Attitude(_rate){}
};

