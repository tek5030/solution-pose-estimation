#pragma once

#include "pose_estimator.h"
#include "Eigen/Core"

/// \brief Augmented Reality visualizer
class ARExample {
public:
  /// \brief Constructs the AR visualizer.
  /// \param axes_length The length of the axes in meters.
  explicit ARExample(double axes_length);

  /// \brief Updates the visualizer with the given data.
  /// \param image Current frame.
  /// \param estimate Current pose estimate.
  /// \param K Current camera calibration matrix.
  /// \param matching_time_ms Time spent matching in milliseconds.
  /// \param pose_est_time_ms Time spent estimating pose in milliseconds.
  void update(const cv::Mat& image,
              const PoseEstimate& estimate,
              const Eigen::Matrix3d& K,
              double matching_time_ms,
              double pose_est_time_ms) const;

private:
  Eigen::Vector3d attitudeFromR(const Eigen::Matrix3d& R) const;

  double axes_length_;
  Eigen::Vector4d origin_;
  Eigen::Vector4d X_;
  Eigen::Vector4d Y_;
  Eigen::Vector4d Z_;
};
