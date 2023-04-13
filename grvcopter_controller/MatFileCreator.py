#!/usr/bin/env python3
from tkinter import filedialog as fd
filename = fd.askopenfilename()

data_names = ["Time"]

#primero abro el archivo
with  open(filename, "r") as file:
    #Luego miro cuantos nombres de datos hay diferentes:
    line = file.readline()
    while(len(line.strip()) > 1):
        #Separo la linea en los distintos elementos: Tiempo, Nombre, Valor
        items = line.split(":")
        #Anadir el nombre solo si no estaba ya presente
        if (data_names.count(items[1]) < 1):
            if (items[1] != ''):
                data_names.append(items[1])
        line = file.readline()


#Ya tengo todos los nombres de datos.
#Dimensiono una lista de valores del tamano del num de datos, aqui se iran escribiendo los datos antes de copiarse al archivo.
data_values = []
for i in range(len(data_names)-1):
    data_values.append(i)
print(data_names)

#Crear el .mat, mismo nombre pero con Matlab.mat al final
new_filename = filename.replace(".txt", "Matlab.Mat")

with  open(filename, "r") as from_file:
    with  open(new_filename, "w") as to_file:
        text = ""
        for name in data_names:
            text = text + name + "\t"
        text = text + "\n"
        to_file.write(text)


        linea = from_file.readline()

        while(len(linea.strip()) > 1):
            #Cojo el tiempo y redondeo cada 8 ms considero que es el mismo instante (es la tasa del controlador):
            items = linea.split(":")
            time_first = int(items[0].strip())
            
            while((int(items[0].strip())-time_first) < 8):
                try:
                    ind = data_names.index(items[1]) - 1 
                except:
                    break
                #print(items[1])
                #if (items[1] == "Roll"):
                #   print(ind)
                data_values[ind] = items[2].strip()
                linea = from_file.readline()
                items = linea.split(":")
                if (len(items[0].strip()) < 3):
                    break
            
            text = str(time_first)
            for value in data_values:
                text = text + "\t" + str(value)
            text = text + "\n"
            to_file.write(text)
            if ((int(items[0].strip())-time_first) > 8):
                try:
                    ind = data_names.index(items[1]) - 1 
                except:
                    break
                #print(items[1])
                #if (items[1] == "Roll"):
                #   print(ind)
                data_values[ind] = items[2].strip()
            linea = from_file.readline()

print("Data Set saved: ")
print(data_names)