/*
 * ImageFeartureDetector.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: tiagotrocoli
 */

#include "ImageFeartureDetector.hpp"

#include <opencv2/nonfree/features2d.hpp>

namespace fps_per_velocity {

ImageDetectorDescriptor::ImageDetectorDescriptor(
    const cv::Ptr<cv::FeatureDetector>& feature_detector,
    const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor,
    const cv::Ptr<cv::DescriptorMatcher>& descriptor_matcher) {

  feature_detector_ = feature_detector;
  descriptor_extractor_ = descriptor_extractor;
  descriptor_matcher_ = descriptor_extractor;
}

ImageFeaturesDescriptor ImageDetectorDescriptor::applyFeatureDetectorAndDescriptorExtractor(
    const cv::Mat& image) {

  std::vector<cv::KeyPoint> key_points;
  feature_detector_->detect(image, key_points);

  cv::Mat descriptors;
  descriptor_extractor_->compute(image, key_points, descriptors);

  return ImageFeaturesDescriptor(image, key_points, descriptors);
}

std::vector<cv::DMatch> ImageDetectorDescriptor::applyDescriptorMatcher(
    cv::Mat descriptor_1, cv::Mat descriptor_2) {

  std::vector<cv::DMatch> matchs;
  descriptor_matcher_->match(descriptor_1, descriptor_2, matchs);
  return matchs;
}

ImageFeaturesDescriptor::ImageFeaturesDescriptor(
    cv::Mat image, std::vector<cv::KeyPoint> key_points, cv::Mat descriptor) {
  image_ = image;
  key_points_ = key_points;
  descriptor_ = descriptor;
}

cv::Mat ImageFeaturesDescriptor::image() const {
  return image_;
}

std::vector<cv::KeyPoint> ImageFeaturesDescriptor::key_points() const {
  return key_points_;
}

cv::Mat ImageFeaturesDescriptor::descriptor() const {
  return descriptor_;
}

} /* namespace fps_per_velocity */
