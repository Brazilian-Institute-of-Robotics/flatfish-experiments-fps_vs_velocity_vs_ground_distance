/*
 * PocologTools.hpp
 *
 *  Created on: Feb 18, 2016
 *      Author: tiagotrocoli
 */

#ifndef DRIVERS_FPS_PER_VELOCITY_SRC_POCOLOGTOOLS_HPP_
#define DRIVERS_FPS_PER_VELOCITY_SRC_POCOLOGTOOLS_HPP_

#include <pocolog_cpp/InputDataStream.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frame_helper/FrameHelper.h>

#include <iostream>

namespace fps_per_velocity {

class PocologTools {

 public:
  static pocolog_cpp::InputDataStream* getDataStreamFromLog(
      std::string path, std::string stream_name);

  template<typename type>
  static type getSampleFromDataStream(pocolog_cpp::InputDataStream* stream,
                                      uint i);

  // return a vector which has frame index as key, and dvl index as a target
  template<typename type_main, typename type_target>
  static std::map<int, int> syncPocologStreamsByTime(
      pocolog_cpp::InputDataStream* main_stream,
      pocolog_cpp::InputDataStream* target_stream, float percente_key_stream =
          0.05);

  template<typename T>
  static cv::Mat getMatFromStream(pocolog_cpp::InputDataStream* video_stream,
                                  int index, float percent_image_size);

};

} /* namespace fps_per_velocity */

template<typename type>
inline type fps_per_velocity::PocologTools::getSampleFromDataStream(
    pocolog_cpp::InputDataStream* stream, uint i) {

  type sample;
  if (!stream->getSample<type>(sample, i)) {
    throw std::runtime_error("Error could not load sample ");
  }
  return sample;

}

template<typename type_main, typename type_target>
inline std::map<int, int> fps_per_velocity::PocologTools::syncPocologStreamsByTime(
    pocolog_cpp::InputDataStream* main_stream,
    pocolog_cpp::InputDataStream* target_stream, float percente_key_stream) {

  std::map<int, int> sync_map;
  const uint total_size = main_stream->getSize() * percente_key_stream;
  uint j = 0, i = 0;

  for (i = 0; i < total_size; ++i) {
    std::cout << "\rSYNC PROCESSED " << i * 100 / (1.0 * total_size)
              << std::endl;
    type_main temp_main = getSampleFromDataStream<type_main>(main_stream, i);
    while (j < target_stream->getSize()) {
      type_target temp_target = getSampleFromDataStream<type_target>(
          target_stream, j);
      if (temp_target.time > temp_main.time) {
        sync_map.insert(std::pair<int, int>(i, j - 1));
        break;
      }
      ++j;
    }
  }
  return sync_map;

}

template<typename T>
inline cv::Mat fps_per_velocity::PocologTools::getMatFromStream(
    pocolog_cpp::InputDataStream* video_stream, int index,
    float percent_image_size) {

  base::samples::frame::Frame frame_rock;
  frame_rock = PocologTools::getSampleFromDataStream<T>(video_stream, index);

  cv::Mat image_cv = frame_helper::FrameHelper::convertToCvMat(frame_rock);
  cv::cvtColor(image_cv, image_cv, CV_BayerGR2RGB);
  cv::resize(
      image_cv,
      image_cv,
      cv::Size(image_cv.cols * percent_image_size,
               image_cv.rows * percent_image_size));
  return image_cv;

}

#endif /* DRIVERS_FPS_PER_VELOCITY_SRC_POCOLOGTOOLS_HPP_ */
