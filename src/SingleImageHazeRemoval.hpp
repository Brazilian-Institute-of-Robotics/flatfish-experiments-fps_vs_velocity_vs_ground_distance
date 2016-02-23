/*
 * SingleImageHazeRemoval.h
 *
 *  Created on: Dec 14, 2015
 *      Author: tiagotrocoli
 */

#ifndef SOURCE_DIRECTORY__SRC_SINGLEIMAGEHAZEREMOVAL_H_
#define SOURCE_DIRECTORY__SRC_SINGLEIMAGEHAZEREMOVAL_H_

#include <string>

#include <opencv2/core/core.hpp>

class SingleImageHazeRemoval {

 public:
  virtual ~SingleImageHazeRemoval() {
  }

  virtual cv::Mat applyHazeRemoval(cv::Mat image)=0;
  virtual std::string getSelectedMethod()=0;

};

#endif /* SOURCE_DIRECTORY__SRC_SINGLEIMAGEHAZEREMOVAL_H_ */
