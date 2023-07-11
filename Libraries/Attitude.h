//Created by Carlos Álvarez Cía 2023
#pragma once
#include "Vector3.h"
#include "Helper.h"

class Attitude;

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

        Attitude to_euler();

        Quaternion operator-(Quaternion second){
            Quaternion diff;
            diff.qw() = this->qw() - second.qw();
            diff.qx() = this->qx() - second.qx();
            diff.qy() = this->qy() - second.qy();
            diff.qz() = this->qz() - second.qz();
            return diff;
        }


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

    

    static Attitude compute_error(Attitude ref, Attitude current){
        //TODO voy por aqui
        Attitude error;
        error.roll() = wrap_PI(ref.roll() - current.roll());
        error.pitch() = wrap_PI(ref.pitch() - current.pitch());
        error.yaw() = wrap_PI(ref.yaw() - current.yaw());
        
        return error;
    }

    float& roll(){return _x;}
    float& pitch(){return _y;}
    float& yaw (){return _z;}

    Quaternion to_quaternion(){
        Quaternion quat;
        const float cr2 = cos(this->roll()*0.5);
        const float cp2 = cos(this->pitch()*0.5);
        const float cy2 = cos(this->yaw()*0.5);
        const float sr2 = sin(this->roll()*0.5);
        const float sp2 = sin(this->pitch()*0.5);
        const float sy2 = sin(this->yaw()*0.5);

        quat.qw() = cr2*cp2*cy2 + sr2*sp2*sy2;
        quat.qx() = sr2*cp2*cy2 - cr2*sp2*sy2;
        quat.qy() = cr2*sp2*cy2 + sr2*cp2*sy2;
        quat.qz() = cr2*cp2*sy2 - sr2*sp2*cy2;
        return quat;
    }

    

    //@brief Get lean angles given a vector of forces. Used by non fully actuated UAV to move in XY plane.
    //@param A 3D Vector of forces in Newtons and NED.
    //@returns A 3D Vector of Attitude in Rad and NED.
    static Attitude get_angles_from_forces(Vector v){
        Attitude angles;
        angles.pitch() = atanf(-v[0]/GRAVITY);
        float cos_pitch = cosf(angles.pitch());
        angles.roll() = atanf(v[1]*cos_pitch/GRAVITY);
        return angles;
    }

    //@brief Get the equivalent forces in XY given the target attitude in Rad. Used by fully actuated UAVs to move in XY plane without lean.
    //@param att: [Attitude] Attitude in Rad.
    //@return [Vector] The forces in XY equivalent to a desired attitude. 
    static Vector get_forces_by_angles(Attitude att){
        Vector forces;
        forces[0] = tanf(-att[1])*GRAVITY;
        forces[1] = tanf(-att[0])*GRAVITY;
        return forces;
    }

    //@brief Limit lean angle to a defined max. Used by non fully actuated UAV to move in XY plane.
    //@param att: [Vector] A 3D Vector of lean angles in degrees.
    //@param max_angle_degrees: [float] The max value of angle in degrees.
    //@returns A 3D Vector of Attitude in degrees and NED.
    static Attitude limit_lean_angle(Vector v, float max_angle){
        Attitude angles;
        angles.roll() = saturation(v[0], -max_angle, max_angle);
        angles.pitch() = saturation(v[1], -max_angle, max_angle);
        return angles;
    }

    //@brief Limit lean angle to a defined max. Used by non fully actuated UAV to move in XY plane.
    //@param att: [Vector] A 3D Vector of lean angles in rad.
    //@param max_angle_degrees: [float] The max value of angle in degrees.
    //@returns A 3D Vector of Attitude in rad and NED.
    static Attitude limit_lean_angle_rad(Vector att, float max_angle_degrees){
        Attitude angles;
        float max_angle = Attitude::from_degrees_to_rad(max_angle_degrees);
        angles.roll() = saturation(att[0], -max_angle, max_angle);
        angles.pitch() = saturation(att[1], -max_angle, max_angle);
        return angles;
    }

    //@brief Function to convert the attitude from Degrees to Rad.
    void from_degrees_to_rad(){
        *this = *this * (M_PI*M_1_180);
    }

    //@brief Function to convert the attitude from Rad to Degrees.
    void from_rad_to_degrees(){
        *this = *this * (M_1_PI * 180.0);
    }

    //@brief Function to obtain an Attitude in Rad from an Attitude in Degrees.
    static Attitude from_degrees_to_rad(Attitude _degrees){
        Attitude att;
        att = _degrees*(M_PI*M_1_180);
        return att;
    } 

    static Attitude from_rad_to_degrees(Attitude _rad){
        Attitude att;
        att = _rad*180*M_1_PI;
        return att;
    } 

    static float from_degrees_to_rad(float _degrees){
        return _degrees*M_PI*M_1_180;
    }

    static float from_rad_to_degrees(float _rad){
        return _rad*M_1_PI*180.0;
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

