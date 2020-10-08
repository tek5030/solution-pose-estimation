#include "scene_3d.h"
#include "opencv2/core/eigen.hpp"

Scene3D::Scene3D(const PlaneWorldModel& world)
    : vis_3d_{"3D visualization"}
    , has_camera_{false}
{
  // Show the world axes in 3D.
  vis_3d_.showWidget("World-axes", cv::viz::WCoordinateSystem(world.gridSize()));

  // Visualize the world plane as a 3D image.
  vis_3d_.showWidget("World-plane", cv::viz::WImage3D(world.worldImage(),
                                                      world.worldSize(),
                                                      cv::Vec3d::all(0.0),
                                                      {0.0, 0.0, -1.0},
                                                      {0.0, -1.0, 0.0}));
}

void Scene3D::update(const cv::Mat& image, const PoseEstimate& estimate, const Eigen::Matrix3d& K)
{
  // Extract a reference to the camera pose.
  const auto& pose = estimate.pose_W_C;

  if (estimate.isFound())
  {
    // Convert pose to cv::Mat.
    cv::Mat pose_W_C;
    cv::eigen2cv(pose.matrix(), pose_W_C);

    // Convert calibration matrix to cv::Mat.
    cv::Matx33d K_cv;
    cv::eigen2cv(K, K_cv);

    // Visualize the camera in 3D.
    vis_3d_.showWidget("Camera", cv::viz::WCameraPosition(K_cv, image, 0.05));
    vis_3d_.setWidgetPose("Camera", cv::Affine3d{pose_W_C});
    has_camera_ = true;
  }
  else if (has_camera_)
  {
    vis_3d_.removeWidget("Camera");
    has_camera_ = false;
  }

  vis_3d_.spinOnce();
}
