#pragma once

#include "math.h"

#define M_1_180 0.0055555555555555556
#define GRAVITY 9.80665
//@brief Function to saturate a value between a min and max.
//@param Value: value to saturate passed by reference. The result will be stored here.
//@param min: Lower value.
//@param max: Higher value.
//@return true if reached saturation.
template <typename T, typename P>
static bool saturation_and_check(P& value, T min, T max){
    bool saturated = false;
    if (value > max){
        value = max;
        saturated = true;
    }
    if (value < min){
        value = min;
        saturated = true;
    }
    return saturated;
}

//@brief Function to saturate a value between a min and max.
//@param Value: value to saturate passed by copy.
//@param min: Lower value.
//@param max: Higher value.
//@return Value between min and max.
template <typename T>
static T saturation(T value, T min, T max){
    T value_saturated = value;
    if (value > max){
        value_saturated = max;
    }
    if (value < min){
        value_saturated = min;
    }
    return value_saturated;
}

//@brief Function to normalize a vector between -1:1.
//@param Value: values to normalize passed by pointer. The result will be stored here.
template <typename T>
static void normalize_sign(T* values, int size){
    T max = 0.0;
    T min = 0.0;
    for (int i = 0; i < size; i++)
    {
        if (values[i] > max) {
            max = values[i];
        }

        if (values[i] < min) {
            min = values[i];
        }
    }

    for (int i = 0; i < size; i++)
    {
        values[i] = values[i]/(max-min);
    }
    
}

//@brief Function to wrap an angle from -pi to pi rad.
//@param value: [typename T] Value to wrap between -pi and pi.
//@return value between -pi and pi.
template <typename T>
static T wrap_PI(T value){
    while(value > M_PI_2){
        value -= M_PI*2;
    }
    while(value < -M_PI_2){
        value += M_PI*2;
    }
    if (value > M_PI){
        value -= (2.0*M_PI);
    }
    if (value < -M_PI){
        value += (2.0*M_PI);
    }
    return value;
}

//@brief Function to wrap an angle from -180 to 180 degrees.
//@param value: [typename T] Value to wrap between -180 and 180.
//@return value between -180 and 180.
template <typename T>
static T wrap_180(T value){
    while(value > 360){
        value -= 360;
    } 
    while(value < -360) {
        value += 360;
    }
    if (value > 180){
        value -= 360;
    }
    if (value < -180){
        value += 360;
    }
    return value;
}

//@brief Function to get the sign of a value. Returns 1 or -1.
//@param Value: [typename T] Value to get the sign.
//@return [typename T] 1 or -1.
template <typename T>
static T sign(T value){
    return value/abs(value);
}

//@brief Function to apply a dead band to a value. If the value pass is not greater than the margins value (in absolute value), returns a 0.0.
//If it is greater, return the value.
//@param Value: Value to apply the dead band.
//@param Margins: Value to determine the dead band width.
//@returns The value after applied the dead band.
template<typename T>
static T dead_band(T value, T margins){
    T result = value;
    if (abs(value) < margins){
        result = 0.0;
    }
    return result;
}


//@brief Function to apply a dead band to a value. If the value pass is between the margins value around the offset, returns the offset.
//If not, return the value.
//@param Value: Value to apply the dead band.
//@param Margins: Value to determine the dead band width.
//@returns The value after applied the dead band.
template<typename T>
static T dead_band(T value, T margins, T offset){
    T result = value;
    if ((abs(value-offset)) < margins){
        result = offset;
    }
    return result;
}
