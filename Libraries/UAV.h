#pragma once
#include "Vector3.h"

namespace UAV {
    static constexpr float mass_kg = 4.0; //Kg
    static const Vector inertia_kg_m2 = Vector(0.3, 0.3, 0.17);
    
    static constexpr int num_motors = 6;
    static constexpr bool fully_actuated = true;
    static constexpr int lower_pwm = 1000;

    static const float roll_factor[num_motors] {0.1344, 0.2130, 0.0787, -0.1344, -0.2130, -0.0787};
    static const float pitch_factor[num_motors] {0.1684, -0.0321, -0.2006, -0.1684, 0.0321, 0.2006};
    static const float yaw_factor[num_motors] {-0.1841, 0.1841, -0.1841, 0.1841, -0.1841, 0.1841};
    static const float x_factor[num_motors] {0.1073, 0.3420, -0.4493, 0.1073, 0.3420, -0.4493};
    static const float y_factor[num_motors] {-0.4569, 0.3214, 0.1355, -0.4569, 0.3214, 0.1355};
    static const float z_factor[num_motors] {0.883, 0.883, 0.883, 0.883, 0.883, 0.883};

    //Debe ser la pseudoinversa en NED con Z up.
    static const float roll_factor_fis[num_motors] {0.9638, 1.5114, 0.5476, -0.9638, -1.5114, -0.5476};
    static const float pitch_factor_fis[num_motors] {1.1888, -0.2403, -1.4291, -1.1888, 0.2403, 1.4291};
    static const float yaw_factor_fis[num_motors] {-0.9348, 0.9348, -0.9348, 0.9348, -0.9348, 0.9348};
    static const float x_factor_fis[num_motors] {0.2051, 0.4864, -0.6914, 0.2051, 0.4864, -0.6914};
    static const float y_factor_fis[num_motors] {-0.6800, 0.5176, 0.1624, -0.6800, 0.5176, 0.1624};
    static const float z_factor_fis[num_motors] {0.1887, 0.1887, 0.1887, 0.1887, 0.1887, 0.1887};

    static constexpr int MODE_STABILIZE = 1;
    static constexpr int MODE_ALTITUDE = 2;
    static constexpr int MODE_POSITION = 3;

    static constexpr float max_lean_angle = 15.0; //Value in degrees.
    
    static constexpr float dead_band_width_yaw_rc = 0.1; 
    static constexpr float dead_band_rc = 0.15; 



}