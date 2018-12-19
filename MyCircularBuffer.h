//mycircularBuffer.h

#ifndef MYCIRCULARBUFFER_H
#define MYCIRCULARBUFFER_H
#include "CircularBuffer.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_assert.h"
#include "Features.h"

class CircularBuffer;
namespace mbed {
template<typename T, uint32_t BufferSize, typename CounterType = uint32_t>
class MyCircularBuffer: public CircularBuffer<T, BufferSize, uint32_t>
{ 
    public:
    T& operator[] (long i){
        return CircularBuffer<T, BufferSize, uint32_t>::_pool[CircularBuffer<T, BufferSize, uint32_t>::_head + i];
    }
    ~MyCircularBuffer(){}

    double standardDev(){
        Features f;
        return f.standardDev(CircularBuffer<T, BufferSize, uint32_t>::_pool, BufferSize);
    }
    double avgDev(){
        Features f;
        return f.avgDev(CircularBuffer<T, BufferSize, uint32_t>::_pool, BufferSize);
    }
    double mean(){
        Features f;
        return f.mean(CircularBuffer<T, BufferSize, uint32_t>::_pool, BufferSize);
    }

    double max(){
        float max = CircularBuffer<T, BufferSize, uint32_t>::_pool[0];
        for(int i = 1; i < BufferSize; i++)
        {
            if(CircularBuffer<T, BufferSize, uint32_t>::_pool[i] > max)
            {
                max = CircularBuffer<T, BufferSize, uint32_t>::_pool[i];
            }
        }
        return max;
    }

    double min(){
        float min = CircularBuffer<T, BufferSize, uint32_t>::_pool[0];
        for(int i = 1; i < BufferSize; i++)
        {
            if(CircularBuffer<T, BufferSize, uint32_t>::_pool[i] < min)
            {
                min = CircularBuffer<T, BufferSize, uint32_t>::_pool[i];
            }
        }
        return min;
    }
    
};

}

#endif 