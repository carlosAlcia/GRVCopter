//Created by Carlos Álvarez Cía 2023


#pragma once
#include <chrono>
#include <string.h>
#include <stdio.h>
// ...

using namespace std::chrono;
using namespace std;


namespace MSG_GRVCOPTER {

    //DATA:
        //Roll
        //Pitch
        //Yaw
    static constexpr unsigned short ATT_MSG_ID = 1;
    //DATA:
        //X
        //Y
        //Z
    static constexpr unsigned short POS_MSG_ID = 2;
    //DATA:
        //RC_1
        //RC_2
        //...
        //RC_MAX_NUM_CHANNEL
    static constexpr unsigned short RC_MSG_ID = 3;
    //DATA:
        //NUM MOTORS
        //PWM_1
        //PWM_2
        //PWM_...
        //PWM_NUM_MOTORS
    static constexpr unsigned short PWM_MSG_ID = 4;
    //DATA:
        //Roll
        //Pitch
        //Yaw
    static constexpr unsigned short RATE_MSG_ID = 5;
    //DATA:
        //X
        //Y
        //Z
    static constexpr unsigned short VEL_MSG_ID = 6;

    //DATA:
        //ID of the constant to change.
        //New Value.
    static constexpr unsigned short PID_CONSTANT_CHANGE_MSG_ID = 7;

    static constexpr unsigned short PARAMS_REQUEST_MSG_ID = 8;

    static constexpr unsigned short NUMBER_PARAMS_MSG_ID = 9;

    static constexpr unsigned short PARAM_INFO_MSG_ID = 10;

    //DATA:
            //X
            //Y
            //Z
            //We have 12 more fields that we can use to specify target position, velocity... for future updates.
    static constexpr unsigned short TARGET_POS_MSG_ID = 11;
    
    
    static constexpr unsigned short BATTERY_VOLTAGE_MSG_ID = 12;

    static constexpr unsigned short MAX_NUM_CHANNEL = 14;
    static constexpr unsigned short MSG_DATA_LENGTH = 15;
    static constexpr unsigned short MSG_SIZE = 70;


    int64_t get_time_us();
    
    union UINT16_BYTES {
        unsigned short value;
        char bytes[sizeof(unsigned short)];
    };

    union INT64_BYTES {
        int64_t value;
        char bytes[sizeof(int64_t)];
    };

    union FLOAT_BYTES {
        float value;
        char bytes[sizeof(float)];
    };

    typedef struct {
        UINT16_BYTES MSG_ID;
        INT64_BYTES TIME;
        FLOAT_BYTES DATA[MSG_DATA_LENGTH];
    } Message_Bytes;

    void pack_rc_message(float* rc_values, Message_Bytes* msg);

    void pack_att_message(float* attitude, Message_Bytes* msg);

    void pack_rate_message(float* rate, Message_Bytes* msg);

    void pack_pos_message(float* position, Message_Bytes* msg);

    void pack_vel_message(float* vel, Message_Bytes* msg);

    void pack_pwm_message(int num_motors, float* pwms, Message_Bytes* msg);

    void pack_param_info_message(int param_number, float value, const char* name_param, Message_Bytes* msg);

    void pack_number_param_message(int number_params, Message_Bytes* msg);

    void get_bytes_msg(Message_Bytes* msg, char *bytes_msg);

    void unpack_message(char* buffer, Message_Bytes* msg);

}