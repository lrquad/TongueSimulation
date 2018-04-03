#ifndef __MATH_FUNCTIONS_H__
#define __MATH_FUNCTIONS_H__

#include "Vector.h"

/*!
* source: https://github.com/ChaliZhg/cfdlab
* \author ChaliZhg

* \date August 2016
*/

// Dot product of two vectors
template <class T>
T dot(const cfdlab::Vector<T>& a, const cfdlab::Vector<T>& b)
{
    assert (a.size() == b.size());
    unsigned int n = a.size ();
    T result = 0;
    for(unsigned int i=0; i<n; ++i)
        result += a(i) * b(i);
        
    return result;
}

#endif
