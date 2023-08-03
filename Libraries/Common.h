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

            float battery_voltage = 0.0;

            Force last_force_controller_out;
            Torques last_torques_controller_out;
            


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

            void mutex_lock(){
                mtx.lock();
            }

            void mutex_unlock(){
                mtx.unlock();
            }

            //@brief Thread Safe.
            void update_current_position(float *_pos){
                Position received_position = _pos;
                //Ardupilot send position in cm, change to m to unify.
                received_position.from_cm_to_m();
                //ArduPilot send position in NEU, change to NED to unify.
                received_position.from_NEU_to_NED();

                //Check if the position received is possible compared with the current position:
                if (_has_position){
                    if (Position::error(&received_position, &current_position).norm_euclidean() > 0.05){
                        return;
                    }
                }

                //If there is no current position or the received one is possible, set as current position.
                mtx.lock();
                current_position = received_position;
                _has_position = true;
                mtx.unlock();
            }

            //@brief Function to set a new target position. Might be called when a new target position msg is received.
            //Called also when changing to poshold or althold.
            //@param: pos: [Position] New target position in m and NED.
            void set_target_position(Position pos){
                mtx.lock();
                target_position = pos;
                mtx.unlock();
            }

            //@brief Thread Safe.
            void set_current_position_as_target_position(){
                mtx.lock();
                target_position = current_position;
                mtx.unlock();
            }

            //@brief Thread Safe.
            void update_current_attitude(float *_att){
                mtx.lock();
                current_attitude = _att;
                _has_position.tick();
                mtx.unlock();
            }

            //@brief Thread Safe.
            void update_battery_voltage(float voltage){
                mtx.lock();
                battery_voltage = voltage;
                mtx.unlock();
            }

            //@brief Thread Safe.
            void update_current_vel(float *_vel){
                mtx.lock();
                current_vel = _vel;
                //Ardupilot send velocities in cm/s, change to m/s to unify.
                current_vel.from_cm_to_m();
                //ArduPilot send velocities in NEU, change to NED to unify.
                current_vel.from_NEU_to_NED();
                mtx.unlock();
            }

            //@brief Thread Safe.
            void update_current_rate(float *_rate){
                mtx.lock();
                current_rate = _rate;
                mtx.unlock();
            }

            //@brief Thread Safe.
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

            //@brief Function to check if it is armed. Channel 7 of RC. Returns a Ebool so is possible to check for rising of falling edges. Thread Safe.
            Ebool is_armed(){
                mtx.lock();
                Ebool armed = _is_armed;
                mtx.unlock();
                return armed;
            }

            //@brief Function to know if GRVCOPTER has just be enabled.
            //Can return false although GRVCOPTER is enabled. To check if it is enabled call grvcopter_running() instead. Thread Safe.
            bool start_grvcopter(){
                bool _rising;
                mtx.lock();
                _rising = _grvcopter_enabled.rising_edge();
                mtx.unlock();
                return _rising;
            }

            //@brief Function to check if GRVCOPTER is enabled or not.
            //GRVCOPTER is enabled when the CHANNEL 9 of RC is High. Thread Safe.
            bool grvcopter_running(){
                bool _g_e;
                mtx.lock();
                _g_e = _grvcopter_enabled;
                mtx.unlock();
                return _g_e;
            }

            //@brief Function to get the RC class. Thread safe.
            void get_rc(RC &_rc){
                mtx.lock();
                _rc = rc;
                mtx.unlock();
            };

            RC* get_rc(){return &rc;};

            //@brief Get the current position in meter and NED. Thread safe.
            void get_current_position(Position &pos){
                mtx.lock();
                pos = current_position;
                mtx.unlock();
            };

            Velocity* get_current_vel(){return &current_vel;}; 
            Position* get_current_position(){return &current_position;};

            //@brief Get the target position in meter and NED. Thread Safe.
            void get_target_position(Position &pos){
                mtx.lock();
                pos = target_position;
                mtx.unlock();
            };

            //@brief Get the target position in meter and NED.
            Position* get_target_position(){return &target_position;};
            //@brief Get the target position Z in meter and NED. Thread Safe.
            float get_target_z(){
                float _z;
                mtx.lock();
                _z = target_position.z();
                mtx.unlock();
                return _z;
                };

            //@brief Get the current attitude in Rad. Thread Safe.
            void get_current_attitude(Attitude &att){
                mtx.lock();
                att = current_attitude;
                mtx.unlock();
                }; 

            //@brief Get the current rate in Rad/s
            Rate* get_current_rate(){return &current_rate;}; 

            //@brief Get the current attitude in Rad.
            Attitude* get_current_attitude(){return &current_attitude;};

            //@brief Get the target attitude in Rad. Thread Safe.
            void get_target_attitude(Attitude &att){
                mtx.lock();
                att = target_attitude;
                mtx.unlock();
                };


            //@brief Get the target attitude in Rad.
            Attitude* get_target_attitude(){return &target_attitude;};

            //@brief Function to check if the position is known or not. It can return false if the position is not being sent from ArdupilotGrvcopter 
            //or if the position is not correct (changing too much in short intervals of time).
            //Be aware that the position will not be sent from Ardupilot if the flight mode in Ardupilot is not Position Hold or Guided.
            //The mode in Ardupilot can be Altitude Hold and the position received will have the altitude component OK but the XY received will be 0.
            //The best practice is to have the same configuration in Ardupilot and Grvcopter: Stabilize, Altitude Hold and Position Hold in the channel 5 of the RC.
            //Thread Safe.
            Ebool has_position(){
                mtx.lock();
                Ebool _has_position_l = _has_position;
                mtx.unlock();
                return _has_position_l;
            }

            //@brief Check the current flight mode and store in a private variable. Get the mode calling get_mode() function.
            //Returns true if it changed from last time checked.
            //@returns True if changed from last time. Thread Safe.
            bool check_mode(){
                RC _rc;
                this->get_rc(_rc);
                float ch_mode_value = _rc.get_channel(CH_MODE);

                int previous_mode = mode;
                
                if (ch_mode_value < -0.8)
                {
                    if(this->has_position()) {
                        mode = UAV::MODE_POSITION;
                    } else {
                        std::cout << "NO POSITION MODE: NO POSITION DATA" << std::endl;
                        mode = UAV::MODE_STABILIZE;
                    }
                }
                else if ((ch_mode_value < -0.2 ) && (ch_mode_value > -0.8))
                {
                    if(this->has_position()) {
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

            //@brief Function to get the current Flight Mode. Thread safe.
            //@return The current flight mode. Thread Safe.
            int get_mode(){
                int _mode;
                mtx.lock();
                _mode = mode;
                mtx.unlock();
                return _mode;
            }
            
            //@brief Get the current battery voltage in mV. Thread Safe.
            float get_battery_voltage(){
                float bat;
                mtx.lock();
                bat = battery_voltage;
                mtx.unlock();
                return bat;
            }

            void set_last_force_controller_out(Force _forces){
                last_force_controller_out = _forces;
            }

            Force get_last_force_controller_out(){
                return last_force_controller_out;
            }

            void set_last_torques_controller_out(Torques _torques){
                last_torques_controller_out = _torques;
            }

            Torques get_last_torques_controller_out(){
                return last_torques_controller_out;
            }
    };

    Common &get_common();


}