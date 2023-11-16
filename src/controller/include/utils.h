#pragma once
#include <cmath>

inline double truncate(double value, double precision = 2)
{
  return (floor((value * pow(10, precision) + 0.5)) / pow(10, precision));
}

template<typename VectorT>
VectorT truncate(const VectorT & value, double precision = 2)
{
  VectorT r;
  for(unsigned i = 0; i < value.size(); ++i)
  {
    r[i] = truncate(value[i], precision);
  }
  return r;
}
