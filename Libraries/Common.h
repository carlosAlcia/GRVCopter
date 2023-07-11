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

            //Current position of the UAV in m and NED.
            Position current_position;

            //Desired position of the UAV in m and NED.
            Position target_position = Position(0.0, 1.0, -1.0);

            //Current velocity of the UAV in m/s and NED.
            Velocity current_vel;
            
            //Current attitude of the UAV in rad.
            Attitude current_attitude;

            //Desired attitude for the UAV in rad.
            Attitude target_attitude;

            //Current rate of the UAV in rad/s.
            Rate current_rate;

            RC rc;
            mutex mtx;

            Ebool _has_position;
            Ebool _grvcopter_enabled;
            Ebool _is_armed;
            bool reset_pids;

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

            bool reset_pid(){
                bool reset_aux = reset_pids;
                reset_pids = false;
                return reset_aux;
            }

            void update_current_position(float *_pos){
                Position received_position = _pos;
                //Ardupilot send position in cm, change to m to unify.
                received_position.from_cm_to_m();
                //ArduPilot send position in NEU, change to NED to unify.
                received_position.from_NEU_to_NED();

                //Check if the position received is possible compared with the current position:
                if (_has_position){
                    if (Position::error(&received_position, &current_position).norm_euclidean() > 1){
                        return;
                    }
                }

                //If there is no current position or the received one is possible, set as current position.
                mtx.lock();
                current_position = received_position;
                _has_position = true;
                mtx.unlock();
            }

            void set_target_position(Position pos){
                mtx.lock();
                target_position = pos;
                mtx.unlock();
            }

            void set_current_position_as_target_position(){
                mtx.lock();
                target_position = current_position;
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
                if (!_grvcopter_enabled){
                    reset_pids = true;
                }
                _is_armed = rc.is_armed();
                mtx.unlock();
            }

            //@Function to check if it is armed. Channel 7 of RC. Returns a Ebool so is possible to check for rising of falling edges.
            Ebool is_armed(){
                return _is_armed;
            }

            //@brief Function to know if GRVCOPTER has just be enabled.
            //Can return false although GRVCOPTER is enabled. To check if it is enabled call grvcopter_running() instead.
            bool start_grvcopter(){
                return _grvcopter_enabled.rising_edge();
            }

            //@brief Function to check if GRVCOPTER is enabled or not.
            //GRVCOPTER is enabled when the CHANNEL 9 of RC is High.
            bool grvcopter_running(){
                return _grvcopter_enabled;
            }


            void get_rc(RC &_rc){_rc = rc;};
            RC* get_rc(){return &rc;};

            //@brief Get the current position in meter and NED.
            void get_current_position(Position &pos){pos = current_position;}; 
            Velocity* get_current_vel(){return &current_vel;}; 
            Position* get_current_position(){return &current_position;};

            //@brief Get the target position in meter and NED.
            void get_target_position(Position &pos){pos = target_position;}; 
            //@brief Get the target position in meter and NED.
            Position* get_target_position(){return &target_position;};
            //@brief Get the target position Z in meter and NED.
            float get_target_z(){return target_position.z();};

            //@brief Get the current attitude in Rad.
            void get_current_attitude(Attitude &att){att = current_attitude;}; 

            //@brief Get the current rate in Rad/s
            Rate* get_current_rate(){return &current_rate;}; 

            //@brief Get the current attitude in Rad.
            Attitude* get_current_attitude(){return &current_attitude;};

            //@brief Get the target attitude in Rad.
            void get_target_attitude(Attitude &att){att = target_attitude;};
            //@brief Get the target attitude in Rad.
            Attitude* get_target_attitude(){return &target_attitude;};

            //@brief Function to check if the position is known or not. It can return false if the position is not being sent from ArdupilotGrvcopter 
            //or if the position is not correct (changing too much in short intervals of time).
            //Be aware that the position will not be sent from Ardupilot if the flight mode in Ardupilot is not Position Hold or Guided.
            //The mode in Ardupilot can be Altitude Hold and the position received will have the altitude component OK but the XY received will be 0.
            //The best practice is to have the same configuration in Ardupilot and Grvcopter: Stabilize, Altitude Hold and Position Hold in the channel 5 of the RC.
            Ebool has_position(){
                return _has_position;
            }

            //@brief Check the current flight mode and store in a private variable. Get the mode calling get_mode() function.
            //Returns true if it changed from last time checked.
            //@returns True if changed from last time.
            bool check_mode(){
                float ch_mode_value = rc.get_channel(CH_MODE);
                int previous_mode = mode;
                if (ch_mode_value < -0.8)
                {
                    if(_has_position) {
                        mode = UAV::MODE_POSITION;
                    } else {
                        std::cout << "NO POSITION MODE: NO POSITION DATA" << std::endl;
                        mode = UAV::MODE_STABILIZE;
                    }
                }
                else if ((ch_mode_value < -0.2 ) && (ch_mode_value > -0.8))
                {
                    if(_has_position) {
                        mode = UAV::MODE_ALTITUDE;
                    } else {
                        std::cout << "NO ALTITUDE MODE: NO POSITION DATA" << std::endl;
                        mode = UAV::MODE_STABILIZE;
                    }
                }
                else if (ch_mode_value > -0.2)
                {
                    mode = UAV::MODE_STABILIZE;
                }
                return mode != previous_mode;
            }

            //@brief Function to get the current Flight Mode.
            //@return The current flight mode.
            int get_mode(){
                return mode;
            }

    };

    Common &get_common();


}