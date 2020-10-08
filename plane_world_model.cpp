#include "plane_world_model.h"
#include "feature_utils.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

class PlaneReference
{
public:
  PlaneReference(
      cv::Size image_size,
      cv::Size2d scene_size,
      cv::Point3d origin = {0.0, 0.0, 0.0},
      cv::Point3d x_dir = {1.0, 0.0, 0.0},
      cv::Point3d y_dir = {0.0, 1.0, 0.0}
  )
      : origin_{origin}
      , x_dir_{x_dir}, y_dir_{y_dir}
      , units_per_pixel_x_{scene_size.width / image_size.width}
      , units_per_pixel_y_{scene_size.height / image_size.height}
  {}

  cv::Point3f pixelToWorld(cv::Point2f pixel) const
  {
    return origin_ + (pixel.x * units_per_pixel_x_) * x_dir_ + (pixel.y * units_per_pixel_y_) * y_dir_;
  }

private:
  cv::Point3d origin_;
  cv::Point3d x_dir_;
  cv::Point3d y_dir_;
  double units_per_pixel_x_;
  double units_per_pixel_y_;
};


PlaneWorldModel::PlaneWorldModel(const cv::Mat& world_image, const cv::Size2d& world_size, double grid_size)
    : world_image_{world_image}
    , world_size_{world_size}
    , grid_size_{grid_size}
{
  constructWorld();
}


void PlaneWorldModel::constructWorld()
{
  // Convert to gray scale.
  cv::Mat gray_img;
  cv::cvtColor(world_image_, gray_img, cv::COLOR_BGR2GRAY);

  // Set up objects for detection, description and matching.
  detector_ = cv::xfeatures2d::SURF::create();
  desc_extractor_ = detector_;
  matcher_ = cv::BFMatcher::create(desc_extractor_->defaultNorm());

  // Detect keypoints.
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(gray_img, keypoints);
  cv::KeyPointsFilter::retainBest(keypoints, max_num_points_);

  // Compute descriptors for each keypoint.
  cv::Mat new_descriptors;
  desc_extractor_->compute(gray_img, keypoints, new_descriptors);

  // Do matching step and ration test to remove bad points.
  std::vector<std::vector<cv::DMatch>> matches;
  matcher_->knnMatch(new_descriptors, new_descriptors, matches, 2);
  std::vector<cv::DMatch> good_matches{extractGoodRatioMatches(matches, max_ratio_)};

  // Store points and descriptors.
  PlaneReference ref{world_image_.size(),
                     world_size_,
                     {-0.5*world_size_.width, 0.5*world_size_.height, 0.f},
                     {1.f, 0.f, 0.f},
                     {0.f, -1.f, 0.f}};
  for (auto& match : good_matches)
  {
    world_points_.push_back(ref.pixelToWorld(keypoints[match.queryIdx].pt));
    descriptors_.push_back(new_descriptors.row(match.queryIdx));
  }
}


void PlaneWorldModel::findCorrespondences(const cv::Mat& frame,
                                             std::vector<cv::Point2f> &image_points,
                                             std::vector<cv::Point3f> &world_points) const
{
  image_points.clear();
  world_points.clear();

  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(frame, keypoints);

  cv::Mat frame_desc;
  desc_extractor_->compute(frame, keypoints, frame_desc);

  std::vector<std::vector<cv::DMatch>> matches;
  matcher_->knnMatch(frame_desc, descriptors_, matches, 2);

  std::vector<cv::DMatch> good_matches{extractGoodRatioMatches(matches, max_ratio_)};

  image_points.reserve(good_matches.size());
  world_points.reserve(good_matches.size());
  for (auto& match : good_matches)
  {
    image_points.push_back(keypoints[match.queryIdx].pt);
    world_points.push_back(world_points_[match.trainIdx]);
  }
}


cv::Mat PlaneWorldModel::worldImage() const
{ return world_image_.clone(); }


cv::Size2d PlaneWorldModel::worldSize() const
{ return world_size_; }


double PlaneWorldModel::gridSize() const
{ return grid_size_; }
