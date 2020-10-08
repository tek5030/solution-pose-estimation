#include "camera_projection_measurement.h"

CameraProjectionMeasurement::CameraProjectionMeasurement(
    const Eigen::Vector2d& normalized_plane_point,
    const Eigen::Vector3d& world_point)
    : world_point_{world_point}
    , normalized_plane_point_{normalized_plane_point}
{

}

LinearizedCameraProjectionMeasurement CameraProjectionMeasurement::linearize(const Sophus::SE3d& current_state) const
{
  // Transform world point to camera coordinate frame based on current state estimate.
  Eigen::Vector3d x_c_pred = current_state.inverse() * world_point_;

  // Predict normalized image coordinate based on current state estimate.
  Eigen::Vector2d x_n_pred = x_c_pred.head<2>() / x_c_pred.z();

  // Construct linearization object.
  LinearizedCameraProjectionMeasurement linearization;

  // Compute measurement error.
  linearization.b = normalized_plane_point_ - x_n_pred;

  // Compute Jacobian.
  double d = 1.0 / x_c_pred.z();
  double x_n_y_n = x_n_pred.x()*x_n_pred.y();

  linearization.A <<
    -d, 0.0, d*x_n_pred.x(), x_n_y_n, -1-x_n_pred.x()*x_n_pred.x(), x_n_pred.y(),
    0.0, -d, d*x_n_pred.y(), 1+x_n_pred.y()*x_n_pred.y(), -x_n_y_n, -x_n_pred.x();

  return linearization;
}
