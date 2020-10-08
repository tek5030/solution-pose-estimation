#pragma once

#include "pose_estimator.h"

/// \brief PnP-based pose estimator for calibrated camera with 3D-2D correspondences.
/// This pose estimator first computes an intial result and extracts an inlier set using PnP,
/// then estimates the pose from the entire inlier set using an iterative method.
class PnPPoseEstimator : public PoseEstimator
{
public:
  /// \brief Constructs the pose estimator.
  /// \param K The camera calibration matrix.
  explicit PnPPoseEstimator(const Eigen::Matrix3d& K, bool do_iterative_also = false);

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results. Check PoseEstimate::isFound() to check if solution was found.
  PoseEstimate estimate(const std::vector<cv::Point2f>& image_points,
                        const std::vector<cv::Point3f>& world_points) override;

private:
  cv::Matx33d K_;
  bool do_iterative_also_;
};
