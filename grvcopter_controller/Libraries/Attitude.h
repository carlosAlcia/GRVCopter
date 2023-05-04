//Created by Carlos Álvarez Cía 2023
#pragma once
#include "Vector3.h"
#include "Helper.h"


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

    //@brief Get lean angles given a vector of forces. Used by non fully actuated UAV to move in XY plane.
    //@param A 3D Vector of forces in Newtons and NED.
    //@returns A 3D Vector of Attitude in degrees and NED.
    static Attitude get_angles_from_forces(Vector v){
        Attitude angles;
        const float GRAVITY = 9.81;
        angles.pitch() = from_rad_to_degrees(atanf(-v[0]/GRAVITY));
        float cos_pitch = cosf(from_degrees_to_rad(angles.pitch()));
        angles.roll() = from_rad_to_degrees(atanf(v[1]*cos_pitch/GRAVITY));
        return angles;
    }

    static Vector get_forces_by_angles(Attitude v){
        Vector forces;
        const float GRAVITY = 9.81;
        forces[0] = tanf(from_degrees_to_rad(-v[1]))*GRAVITY;
        forces[1] = tanf(from_degrees_to_rad(-v[0]))*GRAVITY;
        return forces;
    }

    //@brief Limit lean angle to a defined max. Used by non fully actuated UAV to move in XY plane.
    //@param A 3D Vector of lean angles.
    //@param The max value of angle.
    //@returns A 3D Vector of Attitude in degrees and NED.
    static Attitude limit_lean_angle(Vector v, float max_angle){
        Attitude angles;
        angles.roll() = saturation(v[0], -max_angle, max_angle);
        angles.pitch() = saturation(v[1], -max_angle, max_angle);
        return angles;
    }

    void from_degrees_to_rad(){
        *this = *this * (M_PI/180.0);
    }

    void from_rad_to_degrees(){
        *this = *this / (M_PI/180.0);
    }

    static Attitude from_degrees_to_rad(Attitude _degrees){
        Attitude att;
        att = _degrees*(M_PI/180.0);
        return att;
    } 

    static Attitude from_rad_to_degrees(Attitude _degrees){
        Attitude att;
        att = _degrees/(M_PI/180.0);
        return att;
    } 

    static float from_degrees_to_rad(float _degrees){
        return _degrees*M_PI/180.0;
    }

    static float from_rad_to_degrees(float _rad){
        return _rad/M_PI*180.0;
    }

};

class Rate : public Attitude{
    public:
        Rate():Attitude(){}

        Rate(float roll, float pitch, float yaw):Attitude(roll, pitch, yaw){}

        Rate(float* _rate):Attitude(_rate){}

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

class Angular_Acceleration : public Attitude{
    public:
        Angular_Acceleration():Attitude(){}

        Angular_Acceleration(float roll, float pitch, float yaw):Attitude(roll, pitch, yaw){}

        Angular_Acceleration(float* _rate):Attitude(_rate){}

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


class Torques : public Attitude{
    public:
        Torques():Attitude(){}

        Torques(float roll, float pitch, float yaw):Attitude(roll, pitch, yaw){}

        Torques(float* _rate):Attitude(_rate){}

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



