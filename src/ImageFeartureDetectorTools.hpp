/*
 * ImageFeartureDetector.h
 *
 *  Created on: Jan 29, 2016
 *      Author: tiagotrocoli
 */

#ifndef DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEFEARTUREDETECTORTOOLS_HPP_
#define DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEFEARTUREDETECTORTOOLS_HPP_

#include <opencv2/features2d/features2d.hpp>

namespace fps_per_velocity {

// group data frame
class ImageFeaturesDescriptor {

 public:
  ImageFeaturesDescriptor();
  ImageFeaturesDescriptor(cv::Mat image, std::vector<cv::KeyPoint> key_points,
                          cv::Mat descriptor);

  ~ImageFeaturesDescriptor();

  cv::Mat image() const;
  std::vector<cv::KeyPoint> key_points() const;
  cv::Mat descriptor() const;

 private:
  cv::Mat image_;
  std::vector<cv::KeyPoint> key_points_;
  cv::Mat descriptor_;

};

// apply methods to detect features, make descriptors and matcher
class ImageDetectorDescriptor {

 public:
  ImageDetectorDescriptor();
  ImageDetectorDescriptor(
      const cv::Ptr<cv::FeatureDetector> &feature_detector,
      const cv::Ptr<cv::DescriptorExtractor> &descriptor_extractor,
      const cv::Ptr<cv::DescriptorMatcher> &descriptor_matcher);

  ~ImageDetectorDescriptor();

  ImageFeaturesDescriptor applyFeatureDetectorAndDescriptorExtractor(
      const cv::Mat &image);
  std::vector<cv::DMatch> applyDescriptorMatcher(cv::Mat descriptor_1,
                                                 cv::Mat descriptor_2);

  std::vector<cv::DMatch> filterMatchersByDistance(
      std::vector<cv::DMatch> matchers, double percerntOfMax = 0.3);

  std::vector<cv::DMatch> filterMatchersByRansacHomografy(
      std::vector<cv::DMatch> matchers,
      std::vector<cv::KeyPoint> previus_keypoints,
      std::vector<cv::KeyPoint> current_keypoints,
      int ransacReprojThreshold = 3, cv::Mat *homografy = 0);

 private:
  cv::Ptr<cv::FeatureDetector> feature_detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;

};

} /* namespace fps_per_velocity */

#endif /* DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEFEARTUREDETECTORTOOLS_HPP_ */
