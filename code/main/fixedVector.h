/* * A minimalist, custom fixed-size list implementation designed to avoid dynamic memory 
 * allocation (heap fragmentation) on the Arduino.
 * * LIMITATIONS:
 * 1. Fixed capacity: defined by maxSize template argument.
 * 2. Unsafe Access: operator[] does NOT perform bounds checking. 
 * 3. Inefficient Removal: remove() is O(N) complexity (shifts elements).
 * * Use with caution. For more complex needs, consider an Arduino-compatible Vector library.
 */
 
#ifndef FIXED_VECTOR_H
#define FIXED_VECTOR_H

#include <Arduino.h>

template<int maxSize, typename T>
class FixedVector
{
private:
    int length {0};
    T arr[maxSize];

public:
    FixedVector()
    {
    }

    void add(T item)
    {
        if (length >= maxSize)
            return;

        arr[length] = item;
        ++length;
    }

    void remove(int index)
    {
        if (index >= length || index < 0) return;

        for (int i {index}; i < length - 1; ++i)
            arr[i] = arr[i + 1];

        --length;
    }

    T& operator[](int index)
    {
        return arr[index];
    }

    int len()
    {
        return length;
    }

    void clear()
    {
        length = 0;
    }
};

#endif