#include "volumetric_map_base/world_base.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace volumetric_mapping {

void WorldBase::insertDisparityImage(
    const Transformation& sensor_to_world,
    const stereo_msgs::DisparityImageConstPtr& disparity,
    const Eigen::Matrix4d& Q_full, const Eigen::Vector2d& full_image_size) {
  cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(disparity->image, disparity);
  insertDisparityImage(sensor_to_world, cv_img_ptr->image, Q_full, full_image_size);
}

void WorldBase::insertDisparityImage(const Transformation& sensor_to_world,
  const cv::Mat& disparity, const Eigen::Matrix4d& Q_full,
  const Eigen::Vector2d& full_image_size) {
  // Figure out the downsampling of the image.
  double downsampling_factor = full_image_size.x() / disparity.cols;
  Eigen::Matrix4d Q = Q_full;
  if (fabs(downsampling_factor - 1.0) > 1e-6) {
    // c{x,y} and f{x,y} are scaled by the downsampling factor then.
    Q(0, 0) /= downsampling_factor;
    Q(0, 3) /= downsampling_factor*downsampling_factor;
    Q(1, 1) /= downsampling_factor;
    Q(1, 3) /= downsampling_factor*downsampling_factor;
    Q(2, 3) /= downsampling_factor*downsampling_factor;
    Q(3, 2) /= downsampling_factor;
    Q(3, 3) /= downsampling_factor;
  }

  cv::Mat reprojected_disparities(disparity.size(), CV_32FC3);
  cv::Mat Q_cv;
  cv::eigen2cv(Q, Q_cv);

  cv::reprojectImageTo3D(disparity, reprojected_disparities, Q_cv, true);

  // Call the implemnentation function of the inheriting class.
  insertProjectedDisparityIntoMapImpl(sensor_to_world, reprojected_disparities);
}

// Helper functions to compute the Q matrix for given camera parameters.
// Also handles downsampled disparity maps.
Eigen::Matrix4d WorldBase::getQForCameras(double baseline,
    const Eigen::Matrix3d& left_cam_matrix,
    const Eigen::Matrix3d& right_cam_matrix) {
  // TODO(helenol): check if this needs to be negative or positive.
  double Tx = -baseline;
  double left_cx = left_cam_matrix(0, 2);
  double left_cy = left_cam_matrix(1, 2);
  double left_fx = left_cam_matrix(0, 0);
  double left_fy = left_cam_matrix(1, 1);
  double right_cx = right_cam_matrix(0, 2);
  double right_cy = right_cam_matrix(1, 2);
  double right_fx = right_cam_matrix(0, 0);
  double right_fy = right_cam_matrix(1, 1);

  return generateQ(Tx, left_cx, left_cy,  left_fx,  left_fy,  right_cx,
                   right_cy, right_fx, right_fy);
}

Eigen::Matrix4d WorldBase::getQForROSCameras(
    const sensor_msgs::CameraInfo& left_camera,
    const sensor_msgs::CameraInfo& right_camera) {
  // Unfortunately updateQ is protected in StereoCameraModel.
  image_geometry::StereoCameraModel stereo_model;
  stereo_model.fromCameraInfo(left_camera, right_camera);

  double Tx = -stereo_model.baseline();
  double left_cx = stereo_model.left().cx();
  double left_cy = stereo_model.left().cy();
  double left_fx = stereo_model.left().fx();
  double left_fy = stereo_model.left().fy();
  double right_cx = stereo_model.right().cx();
  double right_cy = stereo_model.right().cy();
  double right_fx = stereo_model.right().fx();
  double right_fy = stereo_model.right().fy();

  return generateQ(Tx, left_cx, left_cy,  left_fx,  left_fy,  right_cx,
                   right_cy, right_fx, right_fy);
}

Eigen::Matrix4d WorldBase::generateQ(double Tx,
  double left_cx, double left_cy,  double left_fx,  double left_fy,  double right_cx,
  double right_cy,
  double right_fx,
  double right_fy) {
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

  // Basically do the same that the stereo model does by hand:
  // See: https://github.com/ros-perception/vision_opencv/blob/indigo/
  //      image_geometry/src/stereo_camera_model.cpp#L53
  //
  //  From Springer Handbook of Robotics, p. 524:
  // [x y z 1]^T = Q * [u v d 1]^T
  // Where Q is defined as
  // Q = [ FyTx  0     0   -FyCxTx     ]
  //     [ 0     FxTx  0   -FxCyTx     ]
  //     [ 0     0     0    FxFyTx     ]
  //     [ 0     0     -Fy  Fy(Cx-Cx') ]
  //  where primed parameters are from the left projection matrix,
  //  unprimed from the right.
  // Helen's note: their actual implementation uses the left cam as unprimed,
  // which should make more sense since we always have left disparities anyway.
  //  Disparity = x_left - x_right

  Q(0,0) =  left_fy * Tx;
  Q(0,3) = -left_fy * left_cx * Tx;
  Q(1,1) =  left_fx * Tx;
  Q(1,3) = -left_fx * left_cy * Tx;
  Q(2,3) =  left_fx * left_fy * Tx;
  Q(3,2) = -left_fy;
  // Zero when disparities are pre-adjusted for the difference in projection
  // centers of the 2 cameras.
  Q(3,3) =  left_fy * (left_cx - right_cx);

  return Q;
}

}  // namespace volumetric_mapping
