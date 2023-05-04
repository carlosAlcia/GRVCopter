//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Messages_GRVCopter.h"
#include "mutex"
#include "Position.h"
#include "Attitude.h"
#include "RC.h"
#include "Bool_Upgraded.h"
#include "UdpSocket.h"
#include "Parameters.h"
#include "UAV.h"
#include <iostream>

namespace COMMON {

    class Common {

        private:

            Position current_position;
            Position target_position = Position(0.0, 1.0, -1.0);
            Velocity current_vel;
            
            Attitude current_attitude;
            Attitude target_attitude;
            Rate current_rate;

            RC rc;
            mutex mtx;

            Ebool _has_position;
            Ebool _grvcopter_enabled;

            UDP::UDP_Socket* sock;

            int mode = 0;

            PARAMS::Params params;
            


        public: 
            Common(){};
            ~Common(){};

            void set_socket(UDP::UDP_Socket* _socket){
                sock = _socket;
            }

            UDP::UDP_Socket* get_socket(){
                return sock;
            }

            PARAMS::Params* get_params(){
                return &params;
            }

            void update_current_position(float *_pos){
                mtx.lock();
                current_position = _pos;
                //Ardupilot send position in cm, change to m to unify.
                current_position.from_cm_to_m();
                //ArduPilot send velocities in NEU, change to NED to unify.
                current_position.from_NEU_to_NED();
                _has_position = true;
                mtx.unlock();
            }

            void update_current_attitude(float *_att){
                mtx.lock();
                current_attitude = _att;
                _has_position.tick();
                mtx.unlock();
            }

            void update_current_vel(float *_vel){
                mtx.lock();
                current_vel = _vel;
                //Ardupilot send velocities in cm/s, change to m/s to unify.
                current_vel.from_cm_to_m();
                //ArduPilot send velocities in NEU, change to NED to unify.
                current_vel.from_NEU_to_NED();
                mtx.unlock();
            }

            void update_current_rate(float *_rate){
                mtx.lock();
                current_rate = _rate;
                mtx.unlock();
            }

            void update_rc(float *_rc){
                mtx.lock();
                rc = _rc;
                _grvcopter_enabled = rc.grvcopter_enabled();
                mtx.unlock();
            }

            bool start_grvcopter(){
                return _grvcopter_enabled.rising_edge();
            }

            bool grvcopter_running(){
                return _grvcopter_enabled;
            }


            void get_rc(RC &_rc){_rc = rc;};
            RC* get_rc(){return &rc;};

            void get_current_position(Position &pos){pos = current_position;}; 
            Velocity* get_current_vel(){return &current_vel;}; 
            Position* get_current_position(){return &current_position;};

            void get_target_position(Position &pos){pos = target_position;}; 
            Position* get_target_position(){return &target_position;};
            float get_target_z(){return target_position.z();};

            void get_current_attitude(Attitude &att){att = current_attitude;}; 
            Rate* get_current_rate(){return &current_rate;}; 
            Attitude* get_current_attitude(){return &current_attitude;};


            void get_target_attitude(Attitude &att){att = target_attitude;};
            Attitude* get_target_attitude(){return &target_attitude;};

            bool has_position(){
                return _has_position;
            }

            void check_mode(){
                float ch_mode_value = rc.get_channel(CH_MODE);
                if (ch_mode_value > 0.8)
                {
                    mode = UAV::MODE_POSITION;
                }
                else if ((ch_mode_value < 0.8) && (ch_mode_value > -0.8))
                {
                    mode = UAV::MODE_ALTITUDE;
                }
                else if (ch_mode_value < -0.8)
                {
                    mode = UAV::MODE_STABILIZE;
                }
            }

            int get_mode(){
                return mode;
            }

            


 

    };

    Common &get_common();


}