#include "Parameters.h"
#include "Common.h"

extern COMMON::Common& common;

using namespace PARAMS;

void Params::send_all_params(){
    int i = 0;
    MSG_GRVCOPTER::Message_Bytes msg;
    for (const char* param_name : params_names){
        MSG_GRVCOPTER::pack_param_info_message(i, params_values[i], params_names[i], &msg);
        char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};
        MSG_GRVCOPTER::get_bytes_msg(&msg, buffer);
        common.get_socket()->SendReply(buffer);
        i++;
    }
}

void Params::send_number_of_params(){
    MSG_GRVCOPTER::Message_Bytes msg;
    MSG_GRVCOPTER::pack_number_param_message(number_params, &msg);
    char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};
    MSG_GRVCOPTER::get_bytes_msg(&msg, buffer);
    COMMON::Common& common_l = common;

    common.get_socket()->SendReply(buffer);
}

