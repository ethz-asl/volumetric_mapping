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
  virtual void insertDisparityMap(
      const Transformation& sensor_to_world,
      const stereo_msgs::DisparityImageConstPtr& disparity) {}

  virtual void insertPointcloud(
      const Transformation& sensor_to_world,
      const sensor_msgs::PointCloud2::ConstPtr& cloud) {}

  // Methods to query the current map state.
  virtual CellStatus getCellStatus(const Eigen::Vector3d& point,
                                   const Eigen::Vector3d& bounding_box) {
    return CellStatus::kFree;
  }

  // TODO(helenol): what else do we actually need/use here?
  // Overall map size/resolution/etc.? Ray-casting? Test a whole trajectory in
  // batch?
};

}  // namespace volumetric_mapping
#endif  // VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
