#!/usr/bin/env python3
import struct
from tkinter import filedialog as fd
filename = fd.askopenfilename()

names = {
    "ROLL_ID ": 0,
    "PITCH_ID ": 1,
    "YAW_ID ": 2,
    "DES_ROLL_ID ": 3,
    "DES_PITCH_ID ": 4,
    "DES_YAW_ID ": 5,
    "POS_X_ID ": 6,
    "POS_Y_ID ": 7,
    "POS_Z_ID ": 8,
    "DES_POS_X_ID ": 9,
    "DES_POS_Y_ID ": 10,
    "DES_POS_Z_ID ": 11,
    "FORCE_X_DES_ID ": 12,
    "FORCE_Y_DES_ID ": 13,
    "FORCE_Z_DES_ID ": 14,
    "TORQUE_ROLL_DES_ID ": 15,
    "TORQUE_PITCH_DES_ID ": 16,
    "TORQUE_YAW_DES_ID ": 17,
    "PWM1_ID ": 18,
    "PWM2_ID ": 19,
    "PWM3_ID ": 20,
    "PWM4_ID ": 21,
    "PWM5_ID ": 22,
    "PWM6_ID ": 23,
    "PWM7_ID ": 24,
    "PWM8_ID ": 25,
    "PWM9_ID ": 26,
    "PWM10_ID ": 27,
    "PWM11_ID ": 28,
    "PWM12_ID ": 29,
    "LAST" : 30,
}


data_values = []
for i in range(len(names)):
    data_values.append(i)

new_filename = filename.replace(".bin", "Matlab.Mat")

with  open(new_filename, "w") as to_file:
    to_file.write("Time \t")
    for data_name in names.keys():
        to_file.write(data_name + "\t")
    to_file.write("\n")
    with open(filename, 'rb') as f:
        time_first = 0
        Time = 0
        EndOfFile = False
        while(True):
            while(Time-time_first < 8):
                try:
                    byteID = f.read(2)
                    byteTime = f.read(4)
                    byteValue = f.read(4)
                except EOFError:
                    EndOfFile = True
                    break
                if (len(byteValue)*len(byteTime)*len(byteValue) == 0):
                    EndOfFile = True
                    break
                Id = struct.unpack('H', byteID)[0]
                Time = struct.unpack('<L', byteTime)[0]
                Value = struct.unpack('f', byteValue)[0]
                index = 0
                for IDs in names.values():
                    index = index+1
                    if Id == IDs:
                        break
                #Nuevo el if. La siguiente línea estaba alineada con la siguiente.
                if (index >= 0 and index < len(names)):
                    print(str(index) + "\n")
                    data_values[index] = Value
                data_values[0] = time_first
            #Mas de 8 milisegundos, otro bloque de datos
            time_first = Time
            text = ""
            for value in data_values:
                text = text + str(value) + "\t"
            text = text + "\n"
            to_file.write(text)

            if EndOfFile:
                break

