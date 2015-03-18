#ifndef VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
#define VOLUMETRIC_MAP_BASE_WORLD_BASE_H_

#include <kindr/minimal/quat-transformation.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

namespace volumetric_mapping {

typedef kindr::minimal::QuatTransformation Transformation;

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
                                   const Eigen::Vector3d& end);
  virtual CellStatus getLineStatusBoundingBox(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& bounding_box);

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
