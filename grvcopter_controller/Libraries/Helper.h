#pragma once

#include "math.h"
//@brief Function to saturate a value between a min and max.
//@param Value: value to saturate passed by reference. The result will be stored here.
//@param min: Lower value.
//@param max: Higher value.
//@return true if reached saturation.
template <typename T, typename P>
static bool saturation(P& value, T min, T max){
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

template <typename T>
static T sign(T value){
    return value/abs(value);
}


