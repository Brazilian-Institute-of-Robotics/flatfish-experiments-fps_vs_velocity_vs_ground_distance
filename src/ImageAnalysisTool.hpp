/*
 * ImageAnalysisTool.hpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tiagotrocoli
 */

#ifndef DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEANALYSISTOOL_HPP_
#define DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEANALYSISTOOL_HPP_

#include <opencv2/core/core.hpp>

namespace fps_per_velocity {

class ImageAnalysisTool {

 public:

  ImageAnalysisTool();
  // analyse the image overlap based sift
  double siftAnalysis(cv::Mat previous_image, cv::Mat current_image);

};

} /* namespace fps_per_velocity */

#endif /* DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEANALYSISTOOL_HPP_ */
