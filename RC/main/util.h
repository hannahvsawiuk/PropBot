#ifndef UTIL_H
#define UTIL_H

#include <math.h>

#define LN_SCALE_FACTOR -3.0
#define MAX_DUTY        0.95

/**
 * @brief Returns the input linearly mapped to [0, 1] based on the provided I/O minima and maxima.
 * 
 * @param x         input value
 * @param in_min    input minimum
 * @param in_max    input maximum
 * @param out_min   output minimum
 * @param out_max   output maximum
 * @return float 
 */
float linMapToFloat(float x, float in_min, float in_max, float out_min, float out_max);

template <class T, size_t N>
struct Array {
    // Storage
    T data[N];

    static size_t length() { return N; }
    using type = T;

    // Item access
    T &operator[](size_t index) { return data[index]; }
    const T &operator[](size_t index) const { return data[index]; }

    // Iterators
    T *begin() { return &data[0]; }
    const T *begin() const { return &data[0]; }
    T *end() { return &data[N]; }
    const T *end() const { return &data[N]; }

    // Comparisons
    bool operator==(const Array<T, N> &rhs) const {
        if (this == &rhs)
            return true;
        for (size_t i = 0; i < N; i++)
            if ((*this)[i] != rhs[i])
                return false;
        return true;
    }
    bool operator!=(const Array<T, N> &rhs) const {
        return !(*this == rhs);
    }
};

#endif // UTIL_H