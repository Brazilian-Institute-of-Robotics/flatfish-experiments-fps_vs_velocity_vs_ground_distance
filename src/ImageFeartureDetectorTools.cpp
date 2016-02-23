/*
 * ImageFeartureDetector.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: tiagotrocoli
 */

#include <iostream>

#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ImageFeartureDetectorTools.hpp"

namespace fps_per_velocity {

ImageDetectorDescriptor::ImageDetectorDescriptor(
    const cv::Ptr<cv::FeatureDetector>& feature_detector,
    const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor,
    const cv::Ptr<cv::DescriptorMatcher>& descriptor_matcher) {

  feature_detector_ = feature_detector;
  descriptor_extractor_ = descriptor_extractor;
  descriptor_matcher_ = descriptor_extractor;
}

ImageDetectorDescriptor::ImageDetectorDescriptor() {
  int layers = 5;
  int features = 1500;

  feature_detector_ = new cv::SiftFeatureDetector(features, layers);
  descriptor_extractor_ = new cv::SiftDescriptorExtractor(features, layers);
  descriptor_matcher_ = new cv::BFMatcher(cv::NORM_L1, true);
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

ImageFeaturesDescriptor::ImageFeaturesDescriptor() {
}

cv::Mat ImageFeaturesDescriptor::image() const {
  return image_;
}

std::vector<cv::KeyPoint> ImageFeaturesDescriptor::key_points() const {
  return key_points_;
}

ImageFeaturesDescriptor::~ImageFeaturesDescriptor() {
  image_.release();
}

cv::Mat ImageFeaturesDescriptor::descriptor() const {
  return descriptor_;
}

std::vector<cv::DMatch> ImageDetectorDescriptor::filterMatchersByDistance(
    std::vector<cv::DMatch> matchers, double percernt_Max) {

  double max_dist = 0;
  double min_dist = DBL_MAX;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < matchers.size(); ++i) {
    double dist = matchers[i].distance;
    if (dist > max_dist)
      max_dist = dist;
  }
  std::vector<cv::DMatch> good_matches;

  for (int i = 0; i < matchers.size(); ++i) {
    if (matchers[i].distance <= (max_dist * percernt_Max)) {
      good_matches.push_back(matchers[i]);
    }
  }

  return good_matches;
}

ImageDetectorDescriptor::~ImageDetectorDescriptor() {
  descriptor_extractor_.release();
  descriptor_matcher_.release();
  feature_detector_.release();
}

std::vector<cv::DMatch> ImageDetectorDescriptor::filterMatchersByRansacHomografy(
    std::vector<cv::DMatch> matchers,
    std::vector<cv::KeyPoint> previus_keypoints,
    std::vector<cv::KeyPoint> current_keypoints, int ransacReprojThreshold,
    cv::Mat *homografy) {

  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  for (int i = 0; i < matchers.size(); i++) {
    obj.push_back(previus_keypoints[matchers[i].queryIdx].pt);
    scene.push_back(current_keypoints[matchers[i].trainIdx].pt);
  }

  cv::Mat selectedSet;
  cv::Mat temp_homografy = cv::findHomography(obj, scene, CV_RANSAC,
                                              ransacReprojThreshold,
                                              selectedSet);

  if (homografy) {
    temp_homografy.copyTo((*homografy));
  }

  std::vector<cv::DMatch> final_matchers;
  for (uint i = 0; i < selectedSet.rows; ++i) {
    if (selectedSet.at<int>(i))
      final_matchers.push_back(matchers[i]);
  }
  return final_matchers;
}

} /* namespace fps_per_velocity */
