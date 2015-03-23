#ifndef VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
#define VOLUMETRIC_MAP_BASE_WORLD_BASE_H_

#include <kindr/minimal/quat-transformation.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

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
  // Project the given disparity map to 3D and insert it into the map.
  // Q is a 4x4 perspective disparity-to-depth mapping matrix for the full-size
  // image. This takes care of adjusting Q for downsampled disparity maps as
  // well.
  // See: http://docs.opencv.org/modules/calib3d/doc/
  //      camera_calibration_and_3d_reconstruction.html#reprojectimageto3d
  void insertDisparityImage(
      const Transformation& sensor_to_world,
      const stereo_msgs::DisparityImageConstPtr& disparity,
      const Eigen::Matrix4d& Q_full, const Eigen::Vector2d& full_image_size);
  void insertDisparityImage(const Transformation& sensor_to_world,
                            const cv::Mat& disparity,
                            const Eigen::Matrix4d& Q_full,
                            const Eigen::Vector2d& full_image_size);

  // Helper functions to compute the Q matrix for given camera parameters.
  // Assumes UNRECTIFIED camera matrices.
  // Downsampling is handled in insertDisparityImage.
  Eigen::Matrix4d getQForCameras(
    const Transformation& T_C1_C0, const Eigen::Matrix3d& left_cam_matrix,
    const Eigen::Matrix3d& right_cam_matrix, const Eigen::Vector2d& full_image_size) const;
  Eigen::Matrix4d getQForROSCameras(
      const sensor_msgs::CameraInfo& left_camera,
      const sensor_msgs::CameraInfo& right_camera) const;

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

 protected:
  // Called by insertDisparityImage(). Inheriting classes need to implement
  // this.
  // Input is the sensor to world transform and projected points in 3D in
  // the sensor coordinate frame, of type CV_32FC3.
  virtual void insertProjectedDisparityIntoMapImpl(
      const Transformation& sensor_to_world, const cv::Mat& projected_points) {
    LOG(ERROR) << "Calling unimplemented disparity insertion!";
  }

  // Generate Q matrix from parameters.
  Eigen::Matrix4d generateQ(double Tx, double left_cx, double left_cy,
                            double left_fx, double left_fy, double right_cx,
                            double right_cy, double right_fx,
                            double right_fy) const;
};

}  // namespace volumetric_mapping
#endif  // VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
