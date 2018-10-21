#include "mycircularBuffer.h"

/*template <class T>
T& m_circularBuffer::operator[] (long i){
    T data;
    core_util_critical_section_enter();
    if (!empty()) {
        data = _pool[_head + i];
    }
    core_util_critical_section_exit();
    return data;
}
template<typename T, uint32_t BufferSize, typename CounterType = uint32_t>
void MyCircularBuffer<T, BufferSize, uint32_t>::print_array(){
    for (int j=0; j < BufferSize; j++)
        std::cout << "buffer[" << j << "] = " << _pool[BufferSize] << "\n";
}*/