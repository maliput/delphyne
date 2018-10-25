// Copyright 2018 Toyota Research Institute
#include "visualization/car_vis.h"

namespace delphyne {

template <typename T>
int CarVis<T>::num_poses() const {
  return static_cast<int>(GetVisElements().size());
}

// These instantiations must match the API documentation in
// car_vis.h.
template class CarVis<double>;

}  // namespace delphyne
