//features.h

#ifndef FEATURES_H
#define FEATURES_H
#include <cmath>
#include <iostream>
using std::abs;
class Features
{ 
    public:
        Features(){};
        float standardDev(float data[], int size);
        float avgDev(float data[], int size);
        float mean(float data[], int size);
};


#endif 