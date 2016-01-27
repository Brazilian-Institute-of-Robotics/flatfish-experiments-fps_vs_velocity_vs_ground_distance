/*
 * TestTools.h
 *
 *  Created on: Jan 26, 2016
 *      Author: tiagotrocoli
 */

#ifndef DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_
#define DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_

#include <pocolog_cpp/InputDataStream.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frame_helper/FrameHelper.h>


#define PATH_LOG_NAV "/home/tiagotrocoli/jarvis/Documents/BIR-dataset/Log/incoming/ff-nav/20151119-0935"
#define PATH_LOG_PAYLOAD "/home/tiagotrocoli/jarvis/Documents/BIR-dataset/Log/incoming/ff-payload/20151119-0935"

#define LOG_CAMERA_DOWN_LEFT "camera_aravis_down_left.2.log"
#define STREAM_CAMERA_DOWN_LEFT "camera_aravis_down_left.frame"

#define LOG_DVL "dvl_seapilot.0.log"
#define STREAM_DVL_VELOCITY "dvl_seapilot.velocity_samples" // /base/samples/RigidBodyState_m
#define STREAM_DVL_TIME_STAMPgo "dvl_seapilot.timestamp_estimator_status" // /aggregator/TimestampEstimatorStatus

class TestTools {

 public:
  static pocolog_cpp::InputDataStream* getDataStreamFromLog(
      std::string path, std::string stream_name);

  template<typename type>
  static type getSampleFromDataStream(pocolog_cpp::InputDataStream* stream,
                                      uint i);

  // return a vector which has frame index as key, and dvl index as a target
  template<typename type_key, typename type_target>
  static std::map<int, int> syncPocologStreamsByTime(
      pocolog_cpp::InputDataStream* key_stream,
      pocolog_cpp::InputDataStream* target_stream, float percente_key_stream =
          0.05);

  template<typename T>
  static cv::Mat getMatFromStream(pocolog_cpp::InputDataStream* video_stream,
                                  int index, float percent_image_size);

};



template<typename T>
inline T TestTools::getSampleFromDataStream(
    pocolog_cpp::InputDataStream* stream, uint i) {
  T sample;
  if (!stream->getSample<T>(sample, i)) {
    throw std::runtime_error("Error could not load sample ");
  }
  return sample;
}



template<typename type_key, typename type_target>
inline std::map<int, int> TestTools::syncPocologStreamsByTime(
    pocolog_cpp::InputDataStream* key_stream,
    pocolog_cpp::InputDataStream* target_stream, float percente_key_stream) {

  std::map<int, int> sync_map;

  type_key temp_key;
  type_target temp_target;
  int j = 0;
  int total_size = key_stream->getSize() * percente_key_stream;

  for (int i = 0; i < total_size; ++i) {
    temp_key = getSampleFromDataStream<type_key>(key_stream, i);

    while (j < target_stream->getSize()) {
      temp_target = getSampleFromDataStream<type_target>(target_stream, j);

      if (temp_target.time > temp_key.time) {
        sync_map.insert(std::pair<int, int>(i, j - 1));
        break;
      }
      ++j;
    }
  }
  return sync_map;
}


template<typename T>
inline cv::Mat TestTools::getMatFromStream(
    pocolog_cpp::InputDataStream* video_stream, int index,
    float percent_image_size) {

  base::samples::frame::Frame frame_rock;
  frame_rock = TestTools::getSampleFromDataStream<T>(video_stream, index);

  cv::Mat image_cv = frame_helper::FrameHelper::convertToCvMat(frame_rock);
  cv::cvtColor(image_cv, image_cv, CV_BayerGR2RGB);
  cv::resize(
      image_cv,
      image_cv,
      cv::Size(image_cv.cols * percent_image_size,
               image_cv.rows * percent_image_size));
  return image_cv;
}

#endif /* DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_ */
