#pragma once

#include <ctime>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream> 
#include <chrono>
#include "Attitude.h"
#include "Position.h"

namespace LOG{

    static constexpr int NAME_SIZE = 28;
    static constexpr int NUMBER_OF_DATA = 4;


class Logger {

    private:
        std::ofstream file;
        const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
            

    public: 
        Logger(){
            create_file();          
        }

        ~Logger(){
            file << std::endl;
            file.close();
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
            strcat(file_name, ".txt\0");

            file = ofstream(file_name);
        }

        uint64_t get_millis(){
            const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(now-this->start).count();
        }

    public:
        //Save data as tab separated data.
        //Can be read in Matlab with: tdfread(FILE_NAME)

        void save_att(Attitude *att){
            Attitude att_degrees = Attitude::from_rad_to_degrees(*att);
            file << get_millis() << ":Roll: " << att_degrees.roll() << "\n";
            file << get_millis() << ":Pitch: " << att_degrees.pitch() << "\n";
            file << get_millis() << ":Yaw: " << att_degrees.yaw() << "\n";
        }

        void save_des_att(Attitude *att){
            Attitude att_degrees = Attitude::from_rad_to_degrees(*att);
            file << get_millis() << ":Des_Roll: " << att_degrees.roll() << "\n";
            file << get_millis() << ":Des_Pitch: " << att_degrees.pitch() << "\n";
            file << get_millis() << ":Des_Yaw: " << att_degrees.yaw() << "\n";
        }

        void save_pos(Position *pos){
            file << get_millis() << ":Pos_X: " <<  pos->x() << "\n";
            file << get_millis() << ":Pos_Y: " << pos->y() << "\n";
            file << get_millis() << ":Pos_Z: " <<   pos->z() << "\n";
        }

        void save_des_pos(Position *pos){
            file << get_millis() << ":Des_Pos_X: " <<  pos->x() << "\n";
            file << get_millis() << ":Des_Pos_Y: " << pos->y() << "\n";
            file << get_millis() << ":Des_Pos_Z: " <<   pos->z() << "\n";
        }

        void save_float_data(std::string name, float data){
            file << get_millis() << ":" << name << ": " << data << "\n";
        }


};

}