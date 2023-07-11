#include "Attitude.h"

Attitude Quaternion::to_euler(){
        Attitude att;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (this->qw() * this->qx() + this->qy() * this->qz());
        double cosr_cosp = 1 - 2 * (this->qx() * this->qx() + this->qy() * this->qy());
        att.roll() = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (this->qw() * this->qy() - this->qx() * this->qz()));
        double cosp = std::sqrt(1 - 2 * (this->qw() * this->qy() - this->qx() * this->qz()));
        att.pitch() = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (this->qw() * this->qz() + this->qx() * this->qy());
        double cosy_cosp = 1 - 2 * (this->qy() * this->qy() + this->qz() * this->qz());
        att.yaw() = std::atan2(siny_cosp, cosy_cosp);
        return att;
}