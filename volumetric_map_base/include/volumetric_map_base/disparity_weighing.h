#ifndef VOLUMETRIC_MAP_BASE_DISPARITY_WEIGHING_H_
#define VOLUMETRIC_MAP_BASE_DISPARITY_WEIGHING_H_

#include "volumetric_map_base/point_weighing.h"

namespace volumetric_mapping {

class DisparityWeighing : public PointWeighing {
 public:
  DisparityWeighing(double b, double f, double c_x, double c_y, double delta_d)
      : b_(b), f_(f), c_x_(c_x), c_y_(c_y), delta_d_(delta_d) {}
  virtual ~DisparityWeighing() {}

  virtual double computeWeightForDisparity(unsigned int u, unsigned int v,
                                           double d) const {
    double z = b_ * f_ / d;
    double x = (u - c_x_) / f_;
    double y = (v - c_y_) / f_;

    return computeWeightForPoint(x, y, z);
  }

  virtual double computeWeightForPoint(double x, double y, double z) const {
    // From "Autonomous Visual Mapping and Exploration With a Micro Aerial
    // Vehicle"
    // Lionel Heng, Dominik Honegger, Gim Hee Lee, Lorenz Meier,
    // Petri Tanskanen, Friedrich Fraundorfer, Marc Pollefeys
    Eigen::Vector3d point(x, y, z);
    double r_p = point.norm();
    double sigma = r_p * r_p / (b_ * f_) * delta_d_;
    double w = 1.0 / (sigma + 1.0);
    return w;
  }

 private:
  // Camera baseline in meters:
  double b_;
  // Camera focal length in pixels:
  double f_;
  // Camera center, x, in pixels:
  double c_x_;
  // Camera center, y, in pixels:
  double c_y_;
  // Delta disparity weight, in pixels, 0.5 is a reasonable choice:
  double delta_d_;
};

}  // namespace volumetric_mapping

#endif  // VOLUMETRIC_MAP_BASE_DISPARITY_WEIGHING_H_
