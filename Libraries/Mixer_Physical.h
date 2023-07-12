#pragma once
#include "UAV.h"
#include "Position.h"
#include "Attitude.h"
#include "Helper.h"

class Mixer_Physical
{
private:
    float roll_factors[UAV::num_motors]{0.0};
    float pitch_factors[UAV::num_motors]{0.0};
    float yaw_factors[UAV::num_motors]{0.0};
    float x_factors[UAV::num_motors]{0.0};
    float y_factors[UAV::num_motors]{0.0};
    float z_factors[UAV::num_motors]{0.0};

    float coeff_thrust_to_pwm[3] {-7.0183, 161.25, 1122.2};

public:
    Mixer_Physical(){};

    //@brief Function to set the roll factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_roll_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            roll_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the pitch factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_pitch_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            pitch_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the yaw factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_yaw_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            yaw_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the x factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_x_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            x_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the y factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_y_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            y_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the z factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_z_factors(float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            z_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the roll factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_roll_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            roll_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the pitch factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_pitch_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            pitch_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the yaw factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_yaw_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            yaw_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the x factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_x_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            x_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the y factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_y_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            y_factors[i] = _factors[i];
        }
    }

    //@brief Function to set the z factors in NED.
    //@param _factors: [float *] Array with factors stored in sequence sorted by motor number.
    void set_z_factors(const float *_factors)
    {
        for (int i = 0; i < UAV::num_motors; i++)
        {
            z_factors[i] = _factors[i];
        }
    }


    //@brief Function to get the roll factors in NED.
    //@returns [float*] A pointer to the roll factors.
    float *get_roll_factor()
    {
        return roll_factors;
    }

    //@brief Function to get the pitch factors in NED.
    //@returns [float*] A pointer to the pitch factors.
    float *get_pitch_factor()
    {
        return pitch_factors;
    }

    //@brief Function to get the yaw factors in NED.
    //@returns [float*] A pointer to the yaw factors.
    float *get_yaw_factor()
    {
        return yaw_factors;
    }

    //@brief Function to get the x factors in NED.
    //@returns [float*] A pointer to the x factors.
    float *get_x_factor()
    {
        return x_factors;
    }

    //@brief Function to get the y factors in NED.
    //@returns [float*] A pointer to the y factors.
    float *get_y_factor()
    {
        return y_factors;
    }

    //@brief Function to get the z factors in NED.
    //@returns [float*] A pointer to the z factors.
    float *get_z_factor()
    {
        return z_factors;
    }

    static void get_forces_each_motor(Mixer_Physical* mixer, Force* forces, Torques* torques, float* force_each_motor){
        //Get the mixer:
        float* roll_factor = mixer->get_roll_factor();
        float* pitch_factor = mixer->get_pitch_factor();
        float* yaw_factor = mixer->get_yaw_factor();
        float* x_factor = mixer->get_x_factor();
        float* y_factor = mixer->get_y_factor();
        float* z_factor = mixer->get_z_factor();
        
        for (int i = 0; i < UAV::num_motors; i++)
        {
            force_each_motor[i] += forces->x()*x_factor[i];
            force_each_motor[i] += forces->y()*y_factor[i];
            force_each_motor[i] += forces->z()*z_factor[i];
            force_each_motor[i] += torques->roll()*roll_factor[i];
            force_each_motor[i] += torques->pitch()*pitch_factor[i];
            force_each_motor[i] += torques->yaw()*yaw_factor[i];
        }
    } 

    void force_to_pwm(float *forces_each_motor, float* pwms){
        for (int i = 0; i < UAV::num_motors; i++){
            float force = forces_each_motor[i];
            float pwm_float = coeff_thrust_to_pwm[0]*pow(force, 2) + coeff_thrust_to_pwm[1]*force + coeff_thrust_to_pwm[2]; 
            int pwm = (int) pwm_float;
            saturation_and_check(pwm,1000,2000);
            pwms[i] = pwm;
        }
    }

    //@brief Normalize the forces by mass. A force equivalent of the UAV weight will result in a force in Z equal to 0.5.
    //Divide each force by the same factor (twice the UAV weight).
    //@param force_xyz_n: [Force&] Force variable to normalize. Passed by reference.
    static void normalize_force_by_mass(Force& force_xyz_n){
        float factor = 1/(UAV::mass_kg*GRAVITY*2);
        force_xyz_n = force_xyz_n*abs(factor);
    }

};