/*
 * HistogramEqualization.h
 *
 *  Created on: Dec 14, 2015
 *      Author: tiagotrocoli
 */

#ifndef SOURCE_DIRECTORY__SRC_HISTOGRAM_HISTOGRAMEQUALIZATION_H_
#define SOURCE_DIRECTORY__SRC_HISTOGRAM_HISTOGRAMEQUALIZATION_H_

#include <string>
#include <opencv2/core/core.hpp>

#include "SingleImageHazeRemoval.hpp"

class HistogramEqualization : public SingleImageHazeRemoval {
 public:
  HistogramEqualization();
  ~HistogramEqualization();

  cv::Mat applyHazeRemoval(cv::Mat image);
  std::string getSelectedMethod() {
    return "HistogramEqualization";
  }
};

#endif /* SOURCE_DIRECTORY__SRC_HISTOGRAM_HISTOGRAMEQUALIZATION_H_ */
