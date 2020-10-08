#include "homography_pose_estimator.h"
#include "Eigen/Dense"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

HomographyPoseEstimator::HomographyPoseEstimator(const Eigen::Matrix3d& K)
    : K_{K}
{ }


PoseEstimate HomographyPoseEstimator::estimate(const std::vector<cv::Point2f>& image_points,
                                               const std::vector<cv::Point3f>& world_points)
{
  // Set a minimum required number of points,
  // here 3 times the theoretic minimum.
  constexpr size_t min_number_points = 12;

  // Check that we have enough points.
  if (image_points.size() < min_number_points)
  {
    return {};
  }

  // Compute the homography and extract the inliers.
  std::vector<char> inliers;
  cv::Mat H_cv = cv::findHomography(world_points, image_points, cv::RANSAC, 3, inliers);

  std::vector<cv::Point2f> inlier_image_points;
  std::vector<cv::Point3f> inlier_world_points;
  for (size_t i=0; i<inliers.size(); ++i)
  {
    if (inliers[i] > 0)
    {
      inlier_image_points.push_back(image_points[i]);
      inlier_world_points.push_back(world_points[i]);
    }
  }

  // Check that we have enough inliers.
  if (inlier_image_points.size() < min_number_points)
  {
    return {};
  }

  // Convert homography to Eigen matrix.
  Eigen::Matrix3d H;
  cv::cv2eigen(H_cv, H);

  // Compute the matrix M
  // and extract M_bar (the two first columns of M).
  Eigen::Matrix3d M = K_.inverse() * H;
  Eigen::MatrixXd M_bar = M.leftCols<2>();

  // Perform SVD on M_bar.
  auto svd = M_bar.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute R_bar (the two first columns of R)
  // from the result of the SVD.
  Eigen::Matrix<double, 3, 2> R_bar = svd.matrixU() * svd.matrixV().transpose();

  // Construct R by inserting R_bar and
  // computing the third column of R from the two first.
  // Remember to check det(R)!
  Eigen::Matrix3d R;
  R.leftCols<2>() = R_bar;
  R.col(2) = R_bar.col(0).cross(R_bar.col(1));

  if (R.determinant() < 0)
  {
    R.col(2) *= -1.0;
  }

  // Compute the scale factor lambda.
  double lambda = (R_bar.array() * M_bar.array()).sum() / (M_bar.array() * M_bar.array()).sum();

  // Extract the translation t.
  Eigen::Vector3d t = M.col(2) * lambda;

  // Check that this is the correct solution
  // by testing the last element of t.
  if (t.z() < 0)
  {
    // Switch to other solution.
    t = -t;
    R.topLeftCorner<3,2>() *= -1.0;
  }

  // Return camera pose in the world.
  Sophus::SE3d pose_C_W(R, t);
  return {pose_C_W.inverse(), inlier_image_points, inlier_world_points};
}
