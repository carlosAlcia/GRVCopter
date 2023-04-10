#pragma once

#include <ctime>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream> 
#include <chrono>

namespace LOG{

    static constexpr int NAME_SIZE = 28;
    static constexpr int NUMBER_OF_DATA = 4;


class Logger {

    private:
        //The controller will create 2 files. One containing the default log commonly used as attitude, position... in .mat file.
        std::ofstream file;
        //The other file will contain user defined information but in txt, so another program will be needed in order to get a .mat file.
        std::ofstream file_custom;
        const char* data_names[NUMBER_OF_DATA] = {"Time", "Roll", "Pitch", "Yaw"};
        const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
            

    public: 
        Logger(){
            create_file();          
        }

        ~Logger(){
            file << std::endl;
            file.close();
            file_custom << std::endl;
            file_custom.close();
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
            strcat(file_name, ".mat\0");

            file = ofstream(file_name);
            for (int i = 0; i < NUMBER_OF_DATA; i++){
                file << data_names[i] << "\t";
            }
            file << std::endl;

            char file_name_c[NAME_SIZE] = "";
            strcat(file_name_c, "Log_");
            strcat(file_name_c, date);
            strcat(file_name_c, ".txt\0");

            file_custom = ofstream(file_name_c);
        }

        uint64_t get_millis(){
            const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(now-this->start).count();
        }

    public:
        //Save data as tab separated data.
        //Can be read in Matlab with: tdfread(FILE_NAME)
        void save_all_data();

        void save_time();

        void save_att();

        void save_float_data(std::string name, float data){
            file_custom << get_millis() << ":" << name << ": " << data << std::endl;
        }


};

}