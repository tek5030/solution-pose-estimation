#pragma once

#include "pose_estimator.h"
#include "plane_world_model.h"
#include "opencv2/viz.hpp"

/// \brief 3D visualizer.
class Scene3D
{
public:
  /// \brief Constructs a 3D representation of the planar world model.
  /// \param world World model.
  explicit Scene3D(const PlaneWorldModel& world);

  /// \brief Updated the visualization.
  /// \param image Current frame.
  /// \param estimate Current pose estimate.
  /// \param K Current camera calibration matrix.
  void update(const cv::Mat& image, const PoseEstimate& estimate, const Eigen::Matrix3d& K);

private:
  cv::viz::Viz3d vis_3d_;
  bool has_camera_;
};
