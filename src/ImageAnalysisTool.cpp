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
#include <opencv2/calib3d/calib3d.hpp>

namespace fps_per_velocity {

/* namespace fps_per_velocity */

ImageAnalysisTool::ImageAnalysisTool() {
}

// tenho de melhorar este método para consegui deixá-los mais rápido
double ImageAnalysisTool::siftAnalysis(cv::Mat image_previus,
                                       cv::Mat image_current) {
  double overlap_percent = 1;
  int layers = 5;
  int features = 1500;

  cv::SiftFeatureDetector detector(features, layers);
////  cv::FastFeatureDetector detector;
//  cv::DenseFeatureDetector detector;

  std::vector<cv::KeyPoint> key_points_previus, key_points_current;
  detector.detect(image_previus, key_points_previus);
  detector.detect(image_current, key_points_current);

  // to debug
//  cv::Mat output_previus, output_current;
//  cv::drawKeypoints(image_previus, key_points_previus, output_previus);
//  cv::drawKeypoints(image_current, key_points_current, output_current);

  std::cout << "PASSEI AQUI 1" << std::endl;
  cv::SiftDescriptorExtractor extractor(features, layers);
  cv::Mat descriptors_previus, descriptors_current;
  extractor.compute(image_previus, key_points_previus, descriptors_previus);
  extractor.compute(image_current, key_points_current, descriptors_current);

  std::cout << "PASSEI AQUI 2" << std::endl;
  cv::BFMatcher matcher(cv::NORM_L2, true);
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors_previus, descriptors_current, matches);

  // filter by distance of points
  double max_dist = 0;
  double min_dist = DBL_MAX;

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
  std::vector<cv::DMatch> good_matches, final_matches;

  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i].distance < (max_dist * 0.3)) {
      good_matches.push_back(matches[i]);
    }
  }

//  good_matches = matches;

  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  if (good_matches.size() < 10)
    good_matches = matches;

  std::cout << "MATCHES SIZE " << matches.size() << std::endl;
  std::cout << "SIZE GOOD MATCHERS " << good_matches.size() << std::endl;

  for (int i = 0; i < good_matches.size(); i++) {
    obj.push_back(key_points_previus[good_matches[i].queryIdx].pt);
    scene.push_back(key_points_current[good_matches[i].trainIdx].pt);
  }

  cv::Mat selectedSet;
  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 0, selectedSet);

  for (uint i = 0; i < selectedSet.rows; ++i) {
    if (selectedSet.at<int>(i))
      final_matches.push_back(good_matches[i]);
  }

  std::cout << "SIZE final MATCHERS " << final_matches.size() << std::endl;
  cv::Mat image_matchs;
  std::cout << "PASSEI AQUI 3" << std::endl;
  cv::drawMatches(image_previus, key_points_previus, image_current,
                  key_points_current, good_matches, image_matchs,
                  cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cv::Point2f(0, 0);
  obj_corners[1] = cv::Point2f(image_previus.cols, 0);
  obj_corners[2] = cv::Point2f(image_previus.cols, image_previus.rows);
  obj_corners[3] = cv::Point2f(0, image_previus.rows);
  std::vector<cv::Point2f> scene_corners(4);

  perspectiveTransform(obj_corners, scene_corners, H);

  cv::Mat image_final = image_current.clone();
  for (int i = 0; i < 4; ++i)
    cv::line(image_final, scene_corners[i], scene_corners[(i + 1) % 4],
             cv::Scalar(0, 255, 0), 2);

// continuar daqui
  //seguir este tutorial
  // http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
  // http://stackoverflow.com/questions/29472959/orb-giving-better-feature-matching-than-sift-why
//  cv::imshow("PREVIUS", image_previus);
//  cv::imshow("PREVIUS SIFT", output_previus);
//  cv::imshow("CURRENT SIFT", output_current);
//  cv::imshow("CURRENT", image_current);
  cv::imshow("MATCH", image_matchs);
  cv::imshow("AREA", image_final);
  float fps = 5;
  int perid = (1.0 / fps) * 1000;
//  std::cout << "FPS " << fps << std::endl;
  cv::waitKey(perid);
//  cv::waitKey();

  return overlap_percent;

}

}
