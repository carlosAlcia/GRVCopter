#pragma once
#include "Vector3.h"

class Matriz3 {
    //Structure:
    /*
        [a1 b1 c1
        a2 b2 c2
        a3 b3 c3]
    */

    private:

        float a1 = 0.0;
        float b1 = 0.0;
        float c1 = 0.0;
        float a2 = 0.0;
        float b2 = 0.0;
        float c2 = 0.0;
        float a3 = 0.0;
        float b3 = 0.0;
        float c3 = 0.0;


    public: 
        Matriz3(){};

        //Construct a new matriz with 3 vectors as columns.
        Matriz3(Vector a, Vector b, Vector c){
            a1 = a[0];
            a2 = a[1];
            a3 = a[2];
            
            b1 = b[0];
            b2 = b[1];
            b3 = b[2];
            
            c1 = c[0];
            c2 = c[1];
            c3 = c[2];
        }

        //Upper-right term is the index 2 (3 if beginning count with 1).
        float& operator[](int index){
            switch (index)
            {
            case 0:
                return a1;
                break;
            case 1:
                return b1;
                break;
            case 2:
                return c1;
                break;
            case 3:
                return a2;
                break;
            case 4:
                return b2;
                break;
            case 5:
                return c2;
                break;
            case 6:
                return a3;
                break;
            case 7:
                return b3;
                break;
            case 8:
                return c3;
                break;
            default:
                return a1;
                break;
            }
        }

        Vector a(){
            Vector column(a1, a2, a3);
            return column;
        }

        Vector b(){
            Vector column(b1, b2, b3);
            return column;
        }

        Vector c(){
            Vector column(c1, c2, c3);
            return column;
        }

        Vector _1(){
            Vector row(a1, b1, c1);
            return row;
        }

        Vector _2(){
            Vector row(a2, b2, c2);
            return row;
        }

        Vector _3(){
            Vector row(a3, b3, c3);
            return row;
        }

        void transpose(){
            Matriz3 aux(this->a(), this->b(), this->c());
            a1 = aux[0];
            b1 = (aux.a())[1];
            c1 = (aux.a())[2];
            a2 = (aux.b())[0];
            b2 = aux[4];
            c2 = (aux.b())[2];
            a3 = (aux.c())[0];
            b3 = (aux.c())[1];
            c3 = aux[8];
        }

        Matriz3 transposed(){
            Matriz3 aux(this->a(), this->b(), this->c());
            Matriz3 transposed_m;
            transposed_m[0] = aux[0];
            transposed_m[1] = (aux.a())[1];
            transposed_m[2] = (aux.a())[2];
            transposed_m[3] = (aux.b())[0];
            transposed_m[4] = aux[4];
            transposed_m[5] = (aux.b())[2];
            transposed_m[6] = (aux.c())[0];
            transposed_m[7] = (aux.c())[1];
            transposed_m[8] = aux[8];
            return transposed_m;
        }

        Matriz3 operator*(float scalar){
            Matriz3 aux(this->a(), this->b(), this->c());
            aux.a() = aux.a()*scalar;
            aux.b() = aux.b()*scalar;
            aux.c() = aux.c()*scalar;
            return aux;
        }

        Vector operator*(Vector vector){
            Vector aux;
            aux[0] = this->_1().dot(vector);
            aux[1] = this->_2().dot(vector);
            aux[2] = this->_3().dot(vector);
            return aux;
        }

        Matriz3 inverse_as_diagonal(){
            Matriz3 aux(this->a(), this->b(), this->c());
            aux[0] = 1/aux[0];
            aux[4] = 1/aux[4];
            aux[8] = 1/aux[8];
            return aux; 
        }

};