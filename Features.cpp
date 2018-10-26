#include "Features.h"

float Features::standardDev(float data[], int size){
    float avg = mean(data, size);
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += pow(data[i]-avg, 2);
    }
    return sqrt(sum/size);
}

float Features::avgDev(float data[], int size){
    float avg = mean(data, size);
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += abs(data[i]-avg);
    }
    return sqrt(sum/size);
}

float Features::mean(float data[], int size){
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += data[i];
    }
    return sum/size;
}
