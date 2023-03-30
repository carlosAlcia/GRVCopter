//Created by Carlos Álvarez Cía 2023
#pragma once

class Ebool {
    private:
        bool _value = false;
        bool rise_edge = false;
        bool fall_edge = false;
        int reset = 20;
        const int RESET_DEFAULT = 20;

    public:
        Ebool(){};
        ~Ebool(){};

        void operator=(bool _new_value){
            //Is a change: rise or fall edge
            if (_value != _new_value){
                if (_new_value){
                    rise_edge = true;
                } else {
                    fall_edge = true;
                }
            } else {
                rise_edge = false;
                fall_edge = false;
            }
            if(_new_value){
                reset = RESET_DEFAULT;
            }
            _value = _new_value;
        }

        explicit operator bool() const {
            return _value;
        }

        void tick(){
            if (reset > 0){
                reset--;
            }
            if(reset < 1){
                _value = false;
            }
        }

        bool value(){
            return _value;
        }

        bool operator==(bool _compare){
            return (_value == _compare);
        }

        bool operator==(Ebool _compare){
            return (_value == _compare.value());
        }

};