#include "Features.h"

double Features::standardDev(double data[], int size){
    double avg = mean(data, size);
    double sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += pow(data[i]-avg, 2);
    }
    return sqrt(sum/size);
}

double Features::avgDev(double data[], int size){
    double avg = mean(data, size);
    double sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += abs(data[i]-avg);
    }
    return sum/size;
}

double Features::mean(double data[], int size){
    double sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += data[i];
    }
    return sum/size;
}
