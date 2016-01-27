/*
 * HistogramEqualization.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: tiagotrocoli
 */

#include "HistogramEqualization.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

HistogramEqualization::HistogramEqualization() {

}

HistogramEqualization::~HistogramEqualization() {
}

cv::Mat HistogramEqualization::applyHazeRemoval(cv::Mat image) {

  std::vector<cv::Mat> channels;

  cv::Mat tempImage = image.clone();

  cv::cvtColor(tempImage, tempImage, CV_BGR2HSV);
  cv::split(tempImage, channels);

  //MAIN FUNCTION
  cv::equalizeHist(channels[2], channels[2]);

  cv::merge(channels, tempImage);
  cv::cvtColor(tempImage, tempImage, CV_HSV2BGR);

  return tempImage;
}
