#ifndef VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_
#define VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_


#include <vector>

namespace volumetric_mapping {

// A base class for weighing functions: each weighing function should take in
// either a point in 3D (PCL), UVD (disparity), or an Eigen point (or
// preferably all 3) and return a weight between 0.0 and 1.0 as confidence
// value for the measurement.
// For example, default behavior would happen with a value of 1.0 for all points
// (and this is the base class implementation).
// In the case of raycasting-built maps, such as octomap, all probabilities in
// the ray of the point will be scaled by this amount.
class WeighingFunction {
 public:
  WeighingFunction() {}
  virtual ~WeighingFunction() {}

  virtual double weighPoint(double x, double y, double z) { return 1.0; }
  virtual double weighDisparity(unsigned int u, unsigned int v, double d) {
    return 1.0;
  }
};

}  // namespace volumetric_mapping

#endif  // VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_
