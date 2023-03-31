#pragma once
#include "Vector3.h"

namespace UAV {
    static constexpr float mass_kg = 4.0; //Kg
    static const Vector inertia_kg_m2 = Vector(0.3, 0.3, 0.17);
    
    static constexpr int num_motors = 6;
    static constexpr bool fully_actuated = true;
    static constexpr int lower_pwm = 1000;

    static const float roll_factor[num_motors] {0.1344, 0.2130, 0.0787, -0.1344, -0.2130, -0.0787};
    static const float pitch_factor[num_motors] {0.2130, -0.0321, -0.2006, -0.1684, 0.0321, 0.2006};
    static const float yaw_factor[num_motors] {-0.1841, 0.1841, -0.1841, 0.1841, -0.1841, 0.1841};
    static const float x_factor[num_motors] {0.1073, -0.3420, -0.4493, 0.1073, 0.3420, 0.4493};
    static const float y_factor[num_motors] {-0.4569, 0.3214, 0.1355, -0.4569, 0.3214, 0.1355};
    static const float z_factor[num_motors] {-0.883, -0.883, -0.883, -0.883, -0.883, -0.883};

//     ors_6DoF:add_motor(0, 0.1344, 0.1684, -0.1841, 0.883, 0.1073, -0.4569, false, 1)
// Motors_6DoF:add_motor(1, 0.2130, -0.0321, 0.1841, 0.883, 0.3420, 0.3214, false, 2)
// Motors_6DoF:add_motor(2, 0.0787, -0.2006, -0.1841, 0.883, -0.4493, 0.1355, false, 3)
// Motors_6DoF:add_motor(3, -0.1344, -0.1684, 0.1841, 0.883, 0.1073, -0.4569, false, 4)
// Motors_6DoF:add_motor(4, -0.2130, 0.0321, -0.1841, 0.883, 0.3420, 0.3214, false, 5)
// Motors_6DoF:add_motor(5, -0.0787, 0.2006, 0.1841, 0.883, -0.4493, 0.1355, false, 6)
    


}