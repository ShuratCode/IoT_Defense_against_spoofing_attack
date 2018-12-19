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
        double standardDev(double data[], int size);
        double avgDev(double data[], int size);
        double mean(double data[], int size);
};


#endif 