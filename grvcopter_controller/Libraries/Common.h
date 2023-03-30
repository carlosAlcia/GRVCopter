//Created by Carlos Álvarez Cía 2023

#pragma once
#include "Messages_GRVCopter.h"
#include "mutex"
#include "Position.h"
#include "Attitude.h"
#include "RC.h"
#include "Bool_Upgraded.h"


namespace COMMON {

    class Common {

        private:

            Position current_position;
            Position target_position;
            
            Attitude current_attitude;
            Attitude target_attitude;

            RC rc;
            mutex mtx;

            Ebool _has_position;


        public: 
            Common(){};
            ~Common(){};

            void update_current_position(float *_pos){
                mtx.lock();
                current_position = _pos;
                _has_position = true;
                mtx.unlock();
            }

            void update_current_attitude(float *_att){
                mtx.lock();
                current_attitude = _att;
                _has_position.tick();
                mtx.unlock();
            }

            void update_rc(float *_rc){
                mtx.lock();
                rc = _rc;
                mtx.unlock();
            }


            void get_rc(RC &_rc){_rc = rc;};
            RC* get_rc(){return &rc;};

            void get_current_position(Position &pos){pos = current_position;}; 
            Position* get_current_position(){return &current_position;};

            void get_target_position(Position &pos){pos = target_position;}; 
            Position* get_target_position(){return &target_position;};


            void get_current_attitude(Attitude &att){att = current_attitude;}; 
            Attitude* get_current_attitude(){return &current_attitude;};


            void get_target_attitude(Attitude &att){att = target_attitude;};
            Attitude* get_target_attitude(){return &target_attitude;};

            bool has_position(){return has_position;}
 

    };

    Common &get_common();


}