#include "ar_example.h"
#include "Eigen/Geometry"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iomanip>

ARExample::ARExample(double axes_length)
    : axes_length_{axes_length}
    , origin_{0, 0, 0, 1}
    , X_{axes_length_, 0, 0, 1}
    , Y_{0, axes_length_, 0, 1}
    , Z_{0, 0, axes_length_, 1}
{}

// Define a few BGR-colors for convenience.
namespace color
{
const cv::Scalar blue(255, 0, 0);
const cv::Scalar green(0, 255, 0);
const cv::Scalar red(0, 0, 255);
}

// Define font parameters.
namespace font
{
constexpr auto face = cv::FONT_HERSHEY_PLAIN;
constexpr auto scale_small = 1.0;
constexpr auto scale_large = 2.0;
constexpr auto thickness_bold = 3;
}

void ARExample::update(const cv::Mat& image,
                       const PoseEstimate& estimate,
                       const Eigen::Matrix3d& K,
                       double matching_time_ms,
                       double pose_est_time_ms) const
{
  // Clone image to draw in.
  cv::Mat ar_img = image.clone();

  // Extract reference to pose.
  const auto& pose = estimate.pose_W_C;

  if (estimate.isFound())
  {
    // Compute projection matrix P.
    const Eigen::Matrix<double, 3, 4> P = K * pose.inverse().matrix3x4();

    // Project 3D axis into image.
    Eigen::Vector2d o = (P*origin_).hnormalized();
    Eigen::Vector2d x = (P*X_).hnormalized();
    Eigen::Vector2d y = (P*Y_).hnormalized();
    Eigen::Vector2d z = (P*Z_).hnormalized();

    // Draw axis.
    cv::line(ar_img, cv::Point2d(o[0], o[1]), cv::Point2d(x[0], x[1]), color::red, 4);
    cv::line(ar_img, cv::Point2d(o[0], o[1]), cv::Point2d(y[0], y[1]), color::green, 4);
    cv::line(ar_img, cv::Point2d(o[0], o[1]), cv::Point2d(z[0], z[1]), color::blue, 4);

    // Print processing durations.
    std::stringstream corr_duration_txt;
    corr_duration_txt << std::fixed << std::setprecision(0);
    corr_duration_txt << "Matching: " << matching_time_ms << "ms";
    cv::putText(ar_img, corr_duration_txt.str(), {10, 20}, font::face, font::scale_small, color::red);

    std::stringstream pose_duration_txt;
    pose_duration_txt << std::fixed << std::setprecision(0);
    pose_duration_txt << "Pose est.: " << pose_est_time_ms << "ms";
    cv::putText(ar_img, pose_duration_txt.str(), {10, 40}, font::face, font::scale_small, color::red);

    // Print position.
    const auto& pos = pose.translation() * 100.0; // In cm.
    std::stringstream pos_txt;
    pos_txt << std::fixed << std::setprecision(1);
    pos_txt << "Pos (cm): (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")";
    cv::putText(ar_img, pos_txt.str(), {10, 60}, font::face, font::scale_small, color::green);

    // Print attitude.
    std::stringstream att_txt;
    Eigen::Vector3d att = attitudeFromR(pose.rotationMatrix());
    att_txt << std::fixed << std::setprecision(1);
    att_txt << "Att (deg): (" << att.x() << ", " << att.y() << ", " << att.z() << ")";
    cv::putText(ar_img, att_txt.str(), {10, 80}, font::face, font::scale_small, color::green);

    // Draw keypoints.
    const auto& inliers = estimate.image_inlier_points;
    for (const auto& inlier : inliers)
    {
      cv::drawMarker(ar_img, inlier, color::green, cv::MARKER_CROSS, 5);
    }

  }
  else
  {
    cv::putText(ar_img, "No tracking!", {20, 80}, font::face, font::scale_large, color::red, font::thickness_bold);
  }

  cv::imshow("AR example", ar_img);
}

Eigen::Vector3d ARExample::attitudeFromR(const Eigen::Matrix3d& R) const
{
  Eigen::Vector3d att;

  if (R(2, 0) < 1)
  {
    if (R(2, 0) > -1)
    {
      att.y() = std::asin(-R(2, 0));
      att.x() = std::atan2(R(2, 1) / std::cos(att.y()), R(2, 2) / std::cos(att.y()));
      att.z() = std::atan2(R(1, 0) / std::cos(att.y()), R(0, 0) / std::cos(att.y()));
    }
    else // R(2,0)==-1
    {
      att.x() = std::atan2(-R(1, 2), R(1, 1));
      att.y() = 0.5 * CV_PI;
      att.z() = 0;
    }
  }
  else // R(2,0) == 1
  {
    att.x() = std::atan2(-R(1, 2), R(1, 1));
    att.y() = -0.5 * CV_PI;
    att.z() = 0;
  }

  att.x() *= (180. / CV_PI);
  att.y() *= (180. / CV_PI);
  att.z() *= (180. / CV_PI);

  att.x() = std::fmod(360. + att.x(), 360.) - 180.;

  return att;
}
