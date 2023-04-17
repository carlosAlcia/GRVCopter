//Created by Carlos Álvarez Cía 2023


#include "Grvcopter.h"

using namespace std;

void thread_reception_f(){
    char buffer[MSG_GRVCOPTER::MSG_SIZE] {0};
    while(true){
        sock->Receive(buffer);
        process_msg(buffer);
    }
}


int main(){

    cout << "GRVCopter 1.0" << endl;
    cout << "Carlos Álvarez Cía 2023" << endl;
    cout << "\n" << endl;
    cout << "Enter UAV IP" << endl;
    char ip[20] = "192.168.0.105";
    //cin >> ip;

    UDP::UDP_Socket socket_local = UDP::UDP_Socket(ip);

    sock = &socket_local;
    sock->Ping();

    common.set_socket(sock);

    thread thread_reception = thread(&thread_reception_f);

    while (true) {
        auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(INTERVAL_CONTROLLER);
        run_controller();
        std::this_thread::sleep_until(x);
    }

    return 0;
}

void process_msg(char* buf){
    using namespace MSG_GRVCOPTER;

    Message_Bytes msg;
    unpack_message(buf, &msg);

    switch (msg.MSG_ID.value)
    {
    case ATT_MSG_ID:
        process_att_msg(&msg);
        break;
    case POS_MSG_ID:
        process_pos_msg(&msg);
        break;
    case RATE_MSG_ID:
        process_rate_msg(&msg);
        break;
    case VEL_MSG_ID:
        process_vel_msg(&msg);
        break;
    case RC_MSG_ID:
        process_rc_msg(&msg);
        break;
    default:
        break;
    }
}

void process_att_msg(MSG_GRVCOPTER::Message_Bytes *msg){
    common.update_current_attitude(&(msg->DATA[0].value));
}

void process_pos_msg(MSG_GRVCOPTER::Message_Bytes *msg){
    common.update_current_position(&(msg->DATA[0].value));
}

void process_rate_msg(MSG_GRVCOPTER::Message_Bytes *msg){
    common.update_current_rate(&(msg->DATA[0].value));
}

void process_vel_msg(MSG_GRVCOPTER::Message_Bytes *msg){
    common.update_current_vel(&(msg->DATA[0].value));
}

void process_rc_msg(MSG_GRVCOPTER::Message_Bytes *msg){
    common.update_rc(&(msg->DATA[0].value));
};

void run_controller(){
    extern LOG::Logger& logger;
    logger.save_att(common.get_current_attitude());
    logger.save_des_att(common.get_target_attitude());
    logger.save_des_pos(common.get_target_position());
    logger.save_pos(common.get_current_position());
    if (common.grvcopter_running()){
        common.check_mode();
        controller.run();
    }
}