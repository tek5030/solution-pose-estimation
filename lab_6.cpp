#include "lab_6.h"
#include "ar_example.h"
#include "homography_pose_estimator.h"
#include "plane_world_model.h"
#include "pnp_pose_estimator.h"
#include "moba_pose_estimator.h"
#include "scene_3d.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <chrono>

// Make shorthand aliases for timing tools.
using Clock = std::chrono::high_resolution_clock;
using DurationInMs = std::chrono::duration<double, std::milli>;

// Convenient shorthand for distortion vectors.
namespace cv
{
using Vec5d = Vec<double, 5>;
}

// Struct for camera model data.
// I have chosen to use both eigen and opencv matrices for K,
// since I frequently will need both.
struct CameraModel
{
  Eigen::Matrix3d K;
  cv::Matx33d K_cv;
  cv::Vec5d dist_coeffs_cv;

  Eigen::Vector2d principalPoint() const { return {K(0,2), K(1,2)}; }
  Eigen::Vector2d focalLengths() const { return {K(0,0), K(1,1)}; }
};


CameraModel setupCameraModel()
{
  // (Using parameters for webcam)
  // Set calibration matrix K.
  Eigen::Matrix3d K;
  K << 6.6051081297156020e+02, 0., 3.1810845757653777e+02,
      0., 6.6051081297156020e+02, 2.3995332228230293e+02,
      0., 0., 1.;

  // Create an OpenCV-version for convenience.
  cv::Matx33d K_cv;
  cv::eigen2cv(K, K_cv);

  // Set distortion coefficients [k1, k2, 0, 0, k3].
  cv::Vec5d dist_coeffs_cv{
      0., 2.2202255011309072e-01, 0., 0., -5.0348071005413975e-01};

  return CameraModel{K, K_cv, dist_coeffs_cv};
}


PlaneWorldModel createWorldModel()
{
  // Read "world" image corresponding to the chosen paper size.
  //std::string image_path = "../world_A4.png";
  std::string image_path = "../world_A3.png";
  cv::Mat world_image = cv::imread(image_path);
  if (world_image.empty())
  {
    throw std::runtime_error{"Could not find: " + image_path};
  }

  // Physical world sizes in meters.
  // Choose the paper size you have used.
  //const cv::Size2d a4_size{0.297, 0.210};
  const cv::Size2d a3_size{0.420, 0.297};

  // Grid size in meters.
  // This will be the physical size of axes in the visualization.
  //const double a4_grid_size{0.025};
  const double a3_grid_size{0.040};

  // Create world model.
  PlaneWorldModel world(world_image, a3_size, a3_grid_size);

  return world;
}


void lab6()
{
  // Get camera model parameters.
  const CameraModel camera_model = setupCameraModel();

  // Construct plane world model.
  const PlaneWorldModel world = createWorldModel();

  // Construct pose estimator.
  auto init_estimator = std::make_shared<HomographyPoseEstimator>(camera_model.K);
  MobaPoseEstimator pose_estimator(init_estimator, camera_model.principalPoint(), camera_model.focalLengths());

  // Construct AR visualizer.
  ARExample ar_example(world.gridSize());

  // Construct 3D visualizer.
  Scene3D scene_3D{world};

  // Setup camera stream.
  const int camera_id = 1; // Should be 0 or 1 on the lab PCs.
  cv::VideoCapture cap(camera_id);

  if (!cap.isOpened())
  {
    throw std::runtime_error("Could not open camera " + std::to_string(camera_id));
  }

  for (;;)
  {
    // Read a frame from the camera.
    cv::Mat frame;
    cap >> frame;

    if (frame.empty())
    {
      throw std::runtime_error("Lost camera feed");
    }

    // Undistort the frame using the camera model.
    cv::Mat undistorted_frame;
    cv::undistort(frame, undistorted_frame, camera_model.K_cv, camera_model.dist_coeffs_cv);
    cv::Mat gray_frame;
    cv::cvtColor(undistorted_frame, gray_frame, cv::COLOR_BGR2GRAY);

    // Find the correspondences between the detected image points and the world points.
    // Measure how long the processing takes.
    auto start = Clock::now();
    std::vector<cv::Point2f> matched_image_points;
    std::vector<cv::Point3f> matched_world_points;
    world.findCorrespondences(gray_frame, matched_image_points, matched_world_points);
    auto end = Clock::now();
    DurationInMs correspondence_matching_duration = end - start;

    // Update the pose estimate.
    // Measure how long the processing takes.
    start = Clock::now();
    PoseEstimate estimate = pose_estimator.estimate(matched_image_points, matched_world_points);
    end = Clock::now();
    DurationInMs pose_estimation_duration = end - start;

    // Update Augmented Reality visualization.
    ar_example.update(undistorted_frame, estimate, camera_model.K,
                      correspondence_matching_duration.count(),
                      pose_estimation_duration.count());

    // Update 3D visualization.
    scene_3D.update(undistorted_frame, estimate, camera_model.K);

    if (cv::waitKey(1) >= 0)
    {
      break;
    }
  }
}
