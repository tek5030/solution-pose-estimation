#include "moba_pose_estimator.h"

MobaPoseEstimator::MobaPoseEstimator(PoseEstimator::Ptr initial_pose_estimator,
                                     const Eigen::Vector2d& principal_point,
                                     const Eigen::Vector2d& focal_lengths)
  : initial_pose_estimator_{initial_pose_estimator}
  , principal_point_{principal_point}
  , focal_lengths_{focal_lengths}
{

}

PoseEstimate
MobaPoseEstimator::estimate(const std::vector<cv::Point2f>& image_points, const std::vector<cv::Point3f>& world_points)
{
  // Get initial pose estimate.
  PoseEstimate init_estimate = initial_pose_estimator_->estimate(image_points, world_points);

  if (!init_estimate.isFound())
  {
    return init_estimate;
  }

  // Create measurement set.
  const size_t num_inliers = init_estimate.image_inlier_points.size();
  std::vector<CameraProjectionMeasurement> measurements;
  for (size_t i = 0; i < num_inliers; ++i)
  {
    Eigen::Vector2d image_point{init_estimate.image_inlier_points[i].x,
                                init_estimate.image_inlier_points[i].y};

    Eigen::Vector3d world_point{init_estimate.world_inlier_points[i].x,
                                init_estimate.world_inlier_points[i].y,
                                init_estimate.world_inlier_points[i].z};

    measurements.emplace_back(toNormalized(image_point), world_point);
  }

  // Optimize and update estimate.
  init_estimate.pose_W_C = optimize(measurements, init_estimate.pose_W_C);

  return init_estimate;
}

Eigen::Vector2d MobaPoseEstimator::toNormalized(const Eigen::Vector2d& pixel)
{
  return (pixel - principal_point_).array() / focal_lengths_.array();
}

Sophus::SE3d
MobaPoseEstimator::optimize(const std::vector<CameraProjectionMeasurement>& measurements,
                            const Sophus::SE3d& initial_pose)
{
  const size_t max_iterations = 5;
  const int measure_dim = CameraProjectionMeasurement::measure_dim;
  const int state_dim = CameraProjectionMeasurement::state_dim;

  Eigen::MatrixXd A(measure_dim * measurements.size(), state_dim);
  Eigen::VectorXd b(measure_dim * measurements.size());

  Sophus::SE3d current_state = initial_pose;

  // Comment when done!
  std::cout << "---------" << std::endl;

  size_t iteration = 0;
  double curr_cost = 0.0f;
  while (iteration < max_iterations)
  {
    // Linearize.
    // Build A and b frome each measurement.
    for (size_t j=0; j < measurements.size(); ++j)
    {
      const LinearizedCameraProjectionMeasurement linearization = measurements[j].linearize(current_state);

      A.block(measure_dim*j, 0, measure_dim, state_dim) = linearization.A;
      b.segment(measure_dim*j, measure_dim) = linearization.b;
    }

    // Compute current cost.
    curr_cost = b.squaredNorm();

    // Remove when done!
    std::cout << "Cost before update: " << curr_cost << std::endl;

    // Solve linearized system.
    Sophus::SE3d::Tangent update = A.householderQr().solve(b);

    // Update state.
    current_state = current_state * Sophus::SE3d::exp(update);

    ++iteration;
  }

  return current_state;
}


