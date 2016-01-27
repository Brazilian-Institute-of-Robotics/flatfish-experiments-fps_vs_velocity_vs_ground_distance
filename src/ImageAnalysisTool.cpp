/*
 * ImageAnalysisTool.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tiagotrocoli
 */

#include "ImageAnalysisTool.hpp"

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace fps_per_velocity {

/* namespace fps_per_velocity */

ImageAnalysisTool::ImageAnalysisTool() {
}

// tenho de melhorar este método para consegui deixá-los mais rápido
double ImageAnalysisTool::siftAnalysis(cv::Mat image_previus,
                                       cv::Mat image_current) {
  double overlap_percent = 1;
  int layers = 5;
  int features = 0;

  cv::SiftFeatureDetector detector(features, layers);
////  cv::FastFeatureDetector detector;
//  cv::DenseFeatureDetector detector;

  std::vector<cv::KeyPoint> key_points_previus, key_points_current;
  detector.detect(image_previus, key_points_previus);
  detector.detect(image_current, key_points_current);

  // to debug
  cv::Mat output_previus, output_current;
  cv::drawKeypoints(image_previus, key_points_previus, output_previus);
  cv::drawKeypoints(image_current, key_points_current, output_current);

  std::cout << "PASSEI AQUI 1" << std::endl;
  cv::SiftDescriptorExtractor extractor(features, layers);
  cv::Mat descriptors_previus, descriptors_current;
  extractor.compute(image_previus, key_points_previus, descriptors_previus);
  extractor.compute(image_current, key_points_current, descriptors_current);

  std::cout << "PASSEI AQUI 2" << std::endl;
  cv::BFMatcher matcher(cv::NORM_L1, true);
//  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors_previus, descriptors_current, matches);

  // filter by distance of points
  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors_previus.rows; ++i) {
    double dist = matches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  std::cout << "-- Max dist : " << max_dist << " Min dist : " << min_dist
            << std::endl;

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector<cv::DMatch> good_matches;

  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i].distance < (max_dist * 0.1)) {
      good_matches.push_back(matches[i]);
    }
  }
  cv::Mat image_matchs;
  std::cout << "PASSEI AQUI 3" << std::endl;
  cv::drawMatches(image_previus, key_points_previus, image_current,
                  key_points_current, good_matches, image_matchs,
                  cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//  ,
//      std::vector<char>(),
//      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
//          || cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

// continuar daqui
  //seguir este tutorial
  // http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
  cv::imshow("PREVIUS", image_previus);
  cv::imshow("PREVIUS SIFT", output_previus);
  cv::imshow("CURRENT SIFT", output_current);
  cv::imshow("CURRENT", image_current);
  cv::imshow("MATCH", image_matchs);
  float fps = 5;
  int perid = (1.0 / fps) * 1000;
//  std::cout << "FPS " << fps << std::endl;
  cv::waitKey(perid);
//  cv::waitKey();

  return overlap_percent;

}

}
