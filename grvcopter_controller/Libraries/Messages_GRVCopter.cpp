//Created by Carlos Álvarez Cía 2023

#include "Messages_GRVCopter.h"

using namespace MSG_GRVCOPTER;

int64_t MSG_GRVCOPTER::get_time_us()
{
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void MSG_GRVCOPTER::pack_rc_message(float *rc_values, Message_Bytes *msg)
{
    memset(msg, 0, sizeof(Message_Bytes));
    msg->MSG_ID.value = RC_MSG_ID;
    msg->TIME.value = get_time_us();
    for (int i = 0; i < MAX_NUM_CHANNEL; i++)
    {
        msg->DATA[i].value = rc_values[i];
    }
}

void MSG_GRVCOPTER::pack_att_message(float *attitude, Message_Bytes *msg)
{
    memset(msg, 0, sizeof(Message_Bytes));
    msg->MSG_ID.value = ATT_MSG_ID;
    msg->TIME.value = get_time_us();
    for (int i = 0; i < 3; i++)
    {
        msg->DATA[i].value = attitude[i];
    }
}

void MSG_GRVCOPTER::pack_pos_message(float *position, Message_Bytes *msg)
{
    memset(msg, 0, sizeof(Message_Bytes));
    msg->MSG_ID.value = POS_MSG_ID;
    msg->TIME.value = get_time_us();
    for (int i = 0; i < 3; i++)
    {
        msg->DATA[i].value = position[i];
    }
}

void MSG_GRVCOPTER::get_bytes_msg(Message_Bytes *msg, char *bytes_msg)
{
    memset(bytes_msg, 0, MSG_SIZE);
    memcpy(bytes_msg, msg->MSG_ID.bytes, sizeof(unsigned short));
    // Copy the time:
    short offset = sizeof(unsigned short);
    memcpy(bytes_msg + offset, msg->TIME.bytes, sizeof(int64_t));
    // Copy the data
    offset += sizeof(int64_t);
    for (int i = 0; i < MSG_DATA_LENGTH; i++)
    {
        memcpy(bytes_msg + offset, msg->DATA[i].bytes, sizeof(float));
        offset += sizeof(float);
    }
}

void MSG_GRVCOPTER::unpack_message(char *buffer, Message_Bytes *msg)
{
    memset(msg, 0, sizeof(Message_Bytes));
    // Copy the MSG ID:
    short offset = 0;
    memcpy(msg->MSG_ID.bytes, buffer + offset, sizeof(unsigned short));
    // Copy the time:
    offset += sizeof(unsigned short);
    memcpy(msg->TIME.bytes, buffer + offset, sizeof(int64_t));
    // Copy the data
    offset += sizeof(int64_t);
    for (int i = 0; i < MSG_DATA_LENGTH; i++)
    {
        memcpy(msg->DATA[i].bytes, buffer + offset, sizeof(float));
        offset += sizeof(float);
    }
}