#pragma once

#include "pose_estimator.h"

/// \brief Homography-based pose estimator for a calibrated camera and planar world points.
class HomographyPoseEstimator : public PoseEstimator
{
public:
  /// \brief Constructs the pose estimator.
  /// \param K The camera calibration matrix.
  explicit HomographyPoseEstimator(const Eigen::Matrix3d& K);

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results. Check PoseEstimate::isFound() to check if solution was found.
  PoseEstimate estimate(const std::vector<cv::Point2f>& image_points,
                        const std::vector<cv::Point3f>& world_points) override;

private:
  Eigen::Matrix3d K_;
};
