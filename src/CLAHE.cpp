/*
 * CLAHE.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: tiagotrocoli
 */

#include "CLAHE.hpp"
#include <vector>

CLAHE::~CLAHE() {
  clahe.release();
}

CLAHE::CLAHE(double clipLimit, cv::Size grid) {
  clahe = cv::createCLAHE(clipLimit, grid);
}

CLAHE::CLAHE() {
  clahe = cv::createCLAHE(4, cv::Size(4, 4));
}

cv::Mat CLAHE::applyHazeRemoval(cv::Mat image) {

  std::vector<cv::Mat> channels;

  cv::cvtColor(image, image, CV_BGR2HSV);
  cv::split(image, channels);

  //MAIN FUNCTION
  clahe->apply(channels[2], channels[2]);

  cv::merge(channels, image);
  cv::cvtColor(image, image, CV_HSV2BGR);

  return image;
}
