//Created by Carlos Álvarez Cía 2023


#include <iostream>
#include "Libraries/UdpSocket.h"
#include <thread>
#include "Libraries/Common.h"
#include "Libraries/Messages_GRVCopter.h"
#include "Libraries/Controller.h"
#include "Libraries/Logger.h"

void thread_reception_f();

UDP::UDP_Socket *sock = nullptr;
COMMON::Common& common = COMMON::get_common();
Controller controller;

void process_msg(char* msg);

void process_att_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_pos_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_rc_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_vel_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_rate_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_target_pos_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_pid_constant_change_msg(MSG_GRVCOPTER::Message_Bytes *msg);
void process_params_request_msg();
void log_status_data();

void run_controller();

static constexpr int64_t INTERVAL_CONTROLLER = 10;

