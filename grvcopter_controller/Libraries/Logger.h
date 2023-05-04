#pragma once

#include <ctime>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream> 
#include <chrono>
#include "Attitude.h"
#include "Position.h"
#include "Logger_Constants.h"

namespace LOG{

    static constexpr int NAME_SIZE = 28;
    static constexpr unsigned short LOG_ITEM_SIZE = 10;


    union UINT16_BYTES {
        unsigned short value;
        char bytes[sizeof(unsigned short)];
    };

    union UINT64_BYTES {
        uint32_t value;
        char bytes[sizeof(uint32_t)];
    };

    union FLOAT_BYTES {
        float value;
        char bytes[sizeof(float)];
    };

    typedef struct {
        UINT16_BYTES LOG_ID;
        UINT64_BYTES TIME;
        FLOAT_BYTES DATA;
    } Log_Item_Bytes;




class Logger {

    private:
        FILE* file;
        const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
            

    public: 
        Logger(){
            create_file();          
        }

        ~Logger(){
            fclose(file);
        }

    private:
        void create_file(){
            using namespace std;

            //Create log file as .mat
            time_t t = time(0);   // get time now
            tm* now = localtime(&t);
            char date[20];
            strftime(date, sizeof(date), "%Y-%m-%d.%X", now);


            char file_name[NAME_SIZE] = "";
            strcat(file_name, "Log_");
            strcat(file_name, date);
            strcat(file_name, ".bin\0");

            file = fopen(file_name, "wb");
        }

        uint32_t get_millis(){
            const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now-this->start).count();
        }

        void pack_log_item(uint8_t ID, float value, uint8_t* bytes){
            Log_Item_Bytes log_item = {ID, get_millis(), value};
            memset(bytes, 0, LOG_ITEM_SIZE);
            memcpy(bytes, log_item.LOG_ID.bytes, sizeof(unsigned short));
            // Copy the time:
            short offset = sizeof(unsigned short);
            memcpy(bytes + offset, log_item.TIME.bytes, sizeof(uint32_t));
            // Copy the data
            offset += sizeof(uint32_t);
            memcpy(bytes + offset, log_item.DATA.bytes, sizeof(float));
        }

    public:


        void save_att(Attitude *att){
            Attitude att_degrees = Attitude::from_rad_to_degrees(*att);
            uint8_t bytes[LOG_ITEM_SIZE] = {0};
            pack_log_item(LOG_C::ROLL_ID, att_degrees.roll(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::PITCH_ID, att_degrees.pitch(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::YAW_ID, att_degrees.yaw(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
        }

        void save_des_att(Attitude *att){
            Attitude att_degrees = Attitude::from_rad_to_degrees(*att);
            uint8_t bytes[LOG_ITEM_SIZE] = {0};
            pack_log_item(LOG_C::DES_ROLL_ID, att_degrees.roll(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::DES_PITCH_ID, att_degrees.pitch(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::DES_YAW_ID, att_degrees.yaw(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
        }

        void save_pos(Position *pos){
            uint8_t bytes[LOG_ITEM_SIZE] = {0};
            pack_log_item(LOG_C::POS_X_ID, pos->x(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::POS_Y_ID, pos->y(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::POS_Z_ID, pos->z(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
        }

        void save_des_pos(Position *pos){
            uint8_t bytes[LOG_ITEM_SIZE] = {0};
            pack_log_item(LOG_C::DES_POS_X_ID, pos->x(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::DES_POS_Y_ID, pos->y(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
            pack_log_item(LOG_C::DES_POS_Z_ID, pos->z(), bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
        }

        void save_float_data(uint8_t ID, float data){
            uint8_t bytes[LOG_ITEM_SIZE] = {0};
            pack_log_item(ID, data, bytes);
            fwrite(bytes, sizeof(uint8_t), LOG_ITEM_SIZE, file);
        }


};

}