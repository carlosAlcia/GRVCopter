#pragma once

#include "PID.h"
#include <iostream>
#include <fstream>
#include <string>
#include "Messages_GRVCopter.h"


/*
    How to add a new parameter:
    Add the new parameter ID as a static constexpr short unsigned int after the last one defined.
    Increment the value of the variable number_params.
    Add the name of the new param to the list params_names after the last one.
    In the private variable params_values, add the default param value after the last one.

*/

namespace PARAMS
{

    // Angle controller:
    static constexpr short unsigned int kp_roll_ang_id = 0;
    static constexpr short unsigned int kp_pitch_ang_id = 1;
    static constexpr short unsigned int kp_yaw_ang_id = 2;

    // Rate controller:
    static constexpr short unsigned int kp_roll_rate_id = 3;
    static constexpr short unsigned int kp_pitch_rate_id = 4;
    static constexpr short unsigned int kp_yaw_rate_id = 5;

    static constexpr short unsigned int ki_roll_rate_id = 6;
    static constexpr short unsigned int ki_pitch_rate_id = 7;
    static constexpr short unsigned int ki_yaw_rate_id = 8;

    static constexpr short unsigned int kd_roll_rate_id = 9;
    static constexpr short unsigned int kd_pitch_rate_id = 10;
    static constexpr short unsigned int kd_yaw_rate_id = 11;

    // Position controller:
    static constexpr short unsigned int kp_x_pos_id = 12;
    static constexpr short unsigned int kp_y_pos_id = 13;
    static constexpr short unsigned int kp_z_pos_id = 14;

    // Velocity controller:
    static constexpr short unsigned int kp_x_vel_id = 15;
    static constexpr short unsigned int kp_y_vel_id = 16;
    static constexpr short unsigned int kp_z_vel_id = 17;

    static constexpr short unsigned int ki_x_vel_id = 18;
    static constexpr short unsigned int ki_y_vel_id = 19;
    static constexpr short unsigned int ki_z_vel_id = 20;

    static constexpr short unsigned int kd_x_vel_id = 21;
    static constexpr short unsigned int kd_y_vel_id = 22;
    static constexpr short unsigned int kd_z_vel_id = 23;

    static constexpr short unsigned int number_params = 24;

    static const char *params_names[] = {
        "Roll_ang_P", "Pitch_ang_P", "Yaw_ang_P",
        "Roll_rate_P", "Pitch_rate_P", "Yaw_rate_P",
        "Roll_rate_I", "Pitch_rate_I", "Yaw_rate_I",
        "Roll_rate_D", "Pitch_rate_D", "Yaw_rate_D",
        "X_pos_P", "Y_pos_P", "Z_pos_P",
        "X_vel_P", "Y_vel_P", "Z_vel_P",
        "X_vel_I", "Y_vel_I", "Z_vel_I",
        "X_vel_D", "Y_vel_D", "Z_vel_D"};

    class Params
    {
    public:
        Params() {
            load_params();
        }
        ~Params() {}

    private:
        string filename = "ParametersList.carlos";
        std::ofstream file;
        std::ifstream file_in;

        float params_values[24]{8.0, 8.0, 2.0,
                                 0.15, 0.15, 0.1,
                                 0.001, 0.001, 0.05,
                                 0.01, 0.01, 0.01,
                                 10.0, 10.0, 1.0,
                                 5.0, 5.0, 1.0,
                                 0.05, 0.05, 1.0,
                                 0.05, 0.05, 0.0};

        bool pid_constant_changed = false;

    private:

        void load_params(){
            file_in.open(filename, ios::in);
            if (file_in.is_open()){
                string linea;
                while(getline(file_in, linea)){
                    int index_of_delimiter = linea.find_first_of(":");
                    string param_name = linea.substr(0, index_of_delimiter);
                    string value = linea.substr(index_of_delimiter+1);
                    unsigned short inicio = param_name.find_first_not_of(" ");
                    unsigned short final = param_name.find_last_not_of(" ");
                    if (inicio != string::npos && final != string::npos){
                        param_name = param_name.substr(inicio, final- inicio +1);
                    }
                    int i = 0;
                    for (const char* param : params_names){
                        if (param_name.compare(param) == 0){
                            if (value.find(",") != string::npos){
                                value.replace(value.begin(), value.end(), ",", ".");
                            }
                            params_values[i] = stof(value);
                            break;
                        }
                        i++;
                    }

                }
                pid_constant_changed = true;
            }
            else {
                file.open(filename, ios::app);
                int i = 0;
                for (const char* param_name : params_names){
                    file << param_name << " : " << params_values[i++] << "\n";
                }
                file.close();
            }
        }

        void save_params()
        {
            file.open(filename, ios::out);
            int i = 0;
            for (const char* param_name : params_names){
                file << param_name << " : " << params_values[i++] << "\n";
            }
            file.close();
        }

    public:

        void send_all_params();

        void send_number_of_params();


        float get_param_value(int id)
        {
            return params_values[id];
        }

        void change_param_value(int id, float new_value)
        {
            params_values[id] = new_value;
            save_params();
            pid_constant_changed = true;
        }

        void pid_constants_updated()
        {
            pid_constant_changed = false;
        }

        bool check_constant_pid_changed()
        {
            return pid_constant_changed;
        }

        void update_angle_pid_constants(PID *roll_ang_pid, PID *pitch_ang_pid, PID *yaw_ang_pid)
        {
            roll_ang_pid->kp() = params_values[kp_roll_ang_id];
            pitch_ang_pid->kp() = params_values[kp_pitch_ang_id];
            yaw_ang_pid->kp() = params_values[kp_yaw_ang_id];
        }

        void update_rate_pid_constants(PID *roll_rate_pid, PID *pitch_rate_pid, PID *yaw_rate_pid)
        {
            roll_rate_pid->kp() = params_values[kp_roll_rate_id];
            pitch_rate_pid->kp() = params_values[kp_pitch_rate_id];
            yaw_rate_pid->kp() = params_values[kp_yaw_rate_id];

            roll_rate_pid->ki() = params_values[ki_roll_rate_id];
            pitch_rate_pid->ki() = params_values[ki_pitch_rate_id];
            yaw_rate_pid->ki() = params_values[ki_yaw_rate_id];

            roll_rate_pid->kd() = params_values[kd_roll_rate_id];
            pitch_rate_pid->kd() = params_values[kd_pitch_rate_id];
            yaw_rate_pid->kd() = params_values[kd_yaw_rate_id];
        }

        void update_pos_pid_constants(PID *x_pos_pid, PID *y_pos_pid, PID *z_pos_pid)
        {
            x_pos_pid->kp() = params_values[kp_x_pos_id];
            y_pos_pid->kp() = params_values[kp_y_pos_id];
            z_pos_pid->kp() = params_values[kp_z_pos_id];
        }

        void update_vel_pid_constants(PID *x_vel_pid, PID *y_vel_pid, PID *z_vel_pid)
        {
            x_vel_pid->kp() = params_values[kp_x_vel_id];
            y_vel_pid->kp() = params_values[kp_y_vel_id];
            z_vel_pid->kp() = params_values[kp_z_vel_id];

            x_vel_pid->ki() = params_values[ki_x_vel_id];
            y_vel_pid->ki() = params_values[ki_y_vel_id];
            z_vel_pid->ki() = params_values[ki_z_vel_id];

            x_vel_pid->kd() = params_values[kd_x_vel_id];
            y_vel_pid->kd() = params_values[kd_y_vel_id];
            z_vel_pid->kd() = params_values[kd_z_vel_id];
        }
    };

}