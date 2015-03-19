#ifndef VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
#define VOLUMETRIC_MAP_BASE_WORLD_BASE_H_

#include <kindr/minimal/quat-transformation.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

namespace volumetric_mapping {

typedef kindr::minimal::QuatTransformation Transformation;

// Minimum number of parameters that need to be known about the stereo camera
// pair to project the disparity image into 3D.
struct StereoCameraParameters {
  StereoCameraParameters()
      : baseline(0), focal_length(0), left_cx(0), left_cy(0), right_cx(0),
        right_cy(0), image_width(0), image_height(0) {}
  double baseline;
  double focal_length;
  double left_cx;
  double left_cy;
  double right_cx;
  double right_cy;
  int image_width;
  int image_height;
};

// Base class for all 3D volumetric representations of the environment.
// By default, implements a valid completely empty world.
class WorldBase {
 public:
  typedef std::shared_ptr<WorldBase> Ptr;

  enum CellStatus {
    kFree = 0,
    kOccupied = 1,
    kUnknown = 2
  };

  WorldBase() {}
  virtual ~WorldBase() {}

  // Data insertion functions.
  // TODO(helenol): Figure out which stereo parameters we need that aren't
  // contained in the stereo_msgs::DisparityImage.
  virtual void insertDisparityImage(
      const Transformation& sensor_to_world,
      const stereo_msgs::DisparityImageConstPtr& disparity) {}

  virtual void insertPointcloud(
      const Transformation& sensor_to_world,
      const sensor_msgs::PointCloud2::ConstPtr& cloud) {}

  // Methods to query the current map state.
  virtual CellStatus getCellStatusBoundingBox(
      const Eigen::Vector3d& point,
      const Eigen::Vector3d& bounding_box_size) const {
    return CellStatus::kFree;
  }
  virtual CellStatus getCellStatusPoint(const Eigen::Vector3d& point) const {
    return CellStatus::kFree;
  }

  virtual CellStatus getLineStatus(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end) const {
    return CellStatus::kFree;
  }
  virtual CellStatus getLineStatusBoundingBox(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& bounding_box) const {
    return CellStatus::kFree;
  }

  virtual Eigen::Vector3d getMapCenter() const {
    return Eigen::Vector3d::Zero();
  }
  virtual Eigen::Vector3d getMapSize() const {
    return Eigen::Vector3d(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
  }
};

}  // namespace volumetric_mapping
#endif  // VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
