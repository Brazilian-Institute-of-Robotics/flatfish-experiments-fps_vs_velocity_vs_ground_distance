/*
 * ImageFeartureDetector_test.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: tiagotrocoli
 */

#define BOOST_TEST_MODULE "ImageFeartureDetector_test"
#include <boost/test/unit_test.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <fps_per_velocity/ImageFeartureDetectorTools.hpp>
#include "TestTools.hpp"

using namespace fps_per_velocity;

ImageDetectorDescriptor detectorAndDescriptor;

cv::Mat gt_homografy =
    (cv::Mat_<double>(3, 3) << 0.08699535105686557, -0.0001726008408590446, 454.4593116516771, -0.0006530490389247173, 0.08743479263189738, 605.3735921558917, -9.112779832802366e-07, -2.555594511286005e-07, 1);

BOOST_AUTO_TEST_CASE(ImageFeatureAndDescriptor_testcase) {

  std::string path_image1 = std::string(PATH_RESOURCE) + "/"
      + std::string(IMAGE_01);
  std::string path_image2 = std::string(PATH_RESOURCE) + "/"
      + std::string(IMAGE_02);

  cv::Mat target_image = cv::imread(path_image1);
  cv::Mat query_image = cv::imread(path_image2);

  ImageFeaturesDescriptor target_image_features = detectorAndDescriptor
      .applyFeatureDetectorAndDescriptorExtractor(target_image);
  ImageFeaturesDescriptor query_image_features = detectorAndDescriptor
      .applyFeatureDetectorAndDescriptorExtractor(query_image);

  std::vector<cv::DMatch> matches =
      detectorAndDescriptor.applyDescriptorMatcher(
          target_image_features.descriptor(),
          query_image_features.descriptor());

  matches = detectorAndDescriptor.filterMatchersByDistance(matches, 0.2);

  cv::Mat homografy;
  matches = detectorAndDescriptor.filterMatchersByRansacHomografy(
      matches, target_image_features.key_points(),
      query_image_features.key_points(), 0, &homografy);

  std::cout << "OVERLAPED AREA " << homografy << std::endl;

//  cv::Mat image_matchs;
//  cv::drawMatches(target_image_features.image(),
//                  target_image_features.key_points(),
//                  query_image_features.image(),
//                  query_image_features.key_points(), matches, image_matchs,
//                  cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
//                  std::vector<char>(),
//                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//  cv::resize(image_matchs, image_matchs,
//             cv::Size(image_matchs.rows * 0.25, image_matchs.cols * 0.25));
//  cv::imshow("MATCHERS", image_matchs);
//  cv::waitKey();

  cv::Mat1i temp_gt(gt_homografy * 1000);
  cv::Mat1i temp(homografy * 1000);
  BOOST_CHECK_EQUAL_COLLECTIONS(temp_gt.begin(), temp_gt.end(), temp.begin(),
                                temp.end());

}
