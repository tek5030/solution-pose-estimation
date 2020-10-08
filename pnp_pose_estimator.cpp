#include "pnp_pose_estimator.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

PnPPoseEstimator::PnPPoseEstimator(const Eigen::Matrix3d& K, bool do_iterative_also)
: do_iterative_also_{do_iterative_also}
{
  // Convert to cv::Mat.
  cv::eigen2cv(K, K_);
}

PoseEstimate PnPPoseEstimator::estimate(const std::vector<cv::Point2f>& image_points,
                                        const std::vector<cv::Point3f>& world_points)
{
  // Set a minimum required number of points,
  // here 3 times the theoretic minimum.
  constexpr size_t min_number_points = 9;

  // Check that we have enough points.
  if (image_points.size() < min_number_points)
  {
    return {};
  }

  // Find inliers and compute initial pose with RANSAC.
  cv::Vec3d r_vec;
  cv::Vec3d t_vec;
  std::vector<int> inliers;
  // Take a look at the OpenCV documentation for "solvePnPRansac". What is r_vec and t_vec?
  // Improve the pose estimation by testing different values for the maximal reprojection error.
  cv::solvePnPRansac(world_points, image_points,
                     K_, {}, r_vec, t_vec, false, 10000, 2.0, 0.99, inliers, cv::SOLVEPNP_AP3P);

  // Check that we have enough inliers.
  if (inliers.size() < min_number_points)
  {
    return {};
  }

  // Extract inliers.
  std::vector<cv::Point2f> inlier_image_points;
  std::vector<cv::Point3f> inlier_world_points;
  for (int inlier : inliers)
  {
    inlier_image_points.push_back(image_points[inlier]);
    inlier_world_points.push_back(world_points[inlier]);
  }

  // Compute the camera pose with an iterative method
  // using the entire inlier set.
  // Use "cv::solvePnP" on inlier points to improve "r_vec" and "t_vec". Use the iterative method with current r_vec and t_vec as initial values.
  if (do_iterative_also_)
  {
    cv::solvePnP(inlier_world_points, inlier_image_points, K_, {}, r_vec, t_vec, true);
  }

  // Convert to rotation matrix.
  cv::Mat R_cv;
  cv::Rodrigues(r_vec, R_cv);

  // Convert to Eigen.
  Eigen::Matrix3d R;
  cv::cv2eigen(R_cv, R);
  Eigen::Vector3d t;
  cv::cv2eigen(t_vec, t);

  // Return camera pose in the world.
  Sophus::SE3d pose_C_W(R, t);
  return {pose_C_W.inverse(), inlier_image_points, inlier_world_points};
}
