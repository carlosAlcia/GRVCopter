#include "Logger.h"
#include "Common.h"

using namespace LOG;

const Logger& logger = Logger();
extern COMMON::Common& common;


void Logger::save_all_data(){
    save_time();
    save_att();
    file  << "\n";

}

void Logger::save_time(){
    const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    int64_t milliseconds_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now-this->start).count();
    file << milliseconds_elapsed << "\t";
}

void Logger::save_att(){
    // char roll_str[5];
    // strfromf32(roll_str, sizeof(roll_str), "%1.2f", common.get_current_attitude()->roll());
    file << common.get_current_attitude()->roll() << "\t";
    file << common.get_current_attitude()->pitch() << "\t";
    file << common.get_current_attitude()->yaw() << "\t";
}