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
  double calcOverlapPercent(cv::Mat target_image, cv::Mat query_image,
                            cv::Mat homografy, cv::Mat *overlaped_image = 0);

  void writeMatrixRelation(cv::Mat relation, std::string path_name);
};

} /* namespace fps_per_velocity */

#endif /* DRIVERS_FPS_PER_VELOCITY_SRC_IMAGEANALYSISTOOL_HPP_ */
