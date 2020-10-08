#pragma once

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

/// \brief Represents a planar world.
class PlaneWorldModel
{
public:
  /// \brief Constructs the world model.
  /// \param world_image The world map image.
  /// \param world_size The physical size of the world corresponding to the image in meters.
  /// \param grid_size Size of the grid cells in the world image in meters.
  PlaneWorldModel(const cv::Mat& world_image, const cv::Size2d& world_size, double grid_size);

  /// \brief Finds 3D-2D correspondences with keypoint matching.
  /// \param[in] frame Frame to match with the world.
  /// \param[out] image_points 2D image points corresponding to world points.
  /// \param[out] world_points 3D world points corresponding to image points.
  void findCorrespondences(const cv::Mat& frame,
                           std::vector<cv::Point2f>& image_points,
                           std::vector<cv::Point3f>& world_points) const;

  /// \return The world map image.
  cv::Mat worldImage() const;

  /// \return The physical size of the world map in meters.
  cv::Size2d worldSize() const;

  /// \return The physical size of the grid cells in meters.
  double gridSize() const;

private:
  void constructWorld();

  cv::Mat world_image_;
  cv::Size2d world_size_;
  double grid_size_;

  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> desc_extractor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  float max_ratio_ = 0.8;
  int max_num_points_ = 1000;

  std::vector<cv::Point3f> world_points_;
  cv::Mat descriptors_;
};
