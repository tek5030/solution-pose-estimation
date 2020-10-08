#pragma once

#include "sophus/se3.hpp"

// Forward declaration.
struct LinearizedCameraProjectionMeasurement;

/// \brief Represents a camera projection of a world point, given in normalized image coordinates.
class CameraProjectionMeasurement
{
public:
  /// \brief Constructs the measurement.
  /// \param normalized_plane_point Measured image point as normalized image coordinate.
  /// \param world_point Corresponding world point in world coordinate system.
  CameraProjectionMeasurement(const Eigen::Vector2d& normalized_plane_point, const Eigen::Vector3d& world_point);

  /// \brief Linearized the measurement prediction function at the current state.
  /// \param current_state Current estimate of the camera pose T_w_c.
  LinearizedCameraProjectionMeasurement linearize(const Sophus::SE3d& current_state) const;

  enum {
    measure_dim = 2, /// Dimension of image measurements (normalized image coordinate)
    state_dim = 6    /// Dimension of state (pose)
    };

private:
  Eigen::Vector3d world_point_;
  Eigen::Vector2d normalized_plane_point_;
};

/// \brief Struct for linearization of measurement prediction function.
struct LinearizedCameraProjectionMeasurement
{
  /// Measurement Jacobian
  Eigen::Matrix<double, CameraProjectionMeasurement::measure_dim, CameraProjectionMeasurement::state_dim> A;

  /// Measurement error
  Eigen::Matrix<double, CameraProjectionMeasurement::measure_dim, 1> b;
};
