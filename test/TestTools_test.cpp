/*
 * TestTools_test.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tiagotrocoli
 */

#define BOOST_TEST_MODULE "TestTools_test"
#include <boost/test/unit_test.hpp>

#include <iostream>
#include <base/samples/Frame.hpp>
#include <frame_helper/FrameHelper.h>

#include <pocolog_cpp/InputDataStream.hpp>
#include <pocolog_cpp/LogFile.hpp>
#include <pocolog_cpp/Index.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//using namespace fps_per_velocity;

#define PATH_LOG_PAYLOAD "/home/tiagotrocoli/jarvis/Documents/BIR-dataset/Log/incoming/ff-payload/20151119-0935"

#define LOG_CAMERA_DOWN_LEFT "camera_aravis_down_left.2.log"
#define STREAM_CAMERA_DOWN_LEFT "camera_aravis_down_left.frame"

pocolog_cpp::InputDataStream* getDataStreamFromLog(std::string path,
                                                   std::string stream_name) {
  pocolog_cpp::InputDataStream *data_stream = NULL;
  try {
    pocolog_cpp::LogFile log_file(path);
    pocolog_cpp::Stream *stream = &(log_file.getStream(stream_name));
    data_stream = dynamic_cast<pocolog_cpp::InputDataStream *>(stream);

  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  return data_stream;
}

template<class type>
type getSampleFromDataStream(pocolog_cpp::InputDataStream* stream, uint i) {
  type sample;
  if (!stream->getSample<type>(sample, i)) {
    throw std::runtime_error("Error could not load sample ");
  }
  return sample;
}

BOOST_AUTO_TEST_CASE(getDataStreamFromLog_testcase) {
  const std::string temp_path = std::string(PATH_LOG_PAYLOAD) + "/"
      + std::string(LOG_CAMERA_DOWN_LEFT);
  std::string temp_stream = std::string(STREAM_CAMERA_DOWN_LEFT);

  pocolog_cpp::InputDataStream* data_stream = getDataStreamFromLog(temp_path,
                                                                   temp_stream);
  if (!data_stream)
    BOOST_FAIL("Error do not open log file");

  base::samples::frame::Frame frame_rock;
  frame_rock = getSampleFromDataStream<base::samples::frame::Frame>(data_stream,
                                                                    1000);

  cv::Mat frame_cv = frame_helper::FrameHelper::convertToCvMat(frame_rock);
  cv::cvtColor(frame_cv, frame_cv, CV_BayerGR2RGB);

  if (!frame_cv.data)
    BOOST_FAIL("No data in OpenCV image");
//  cv::imshow("TESTE 1", frame_cv);
//  cv::waitKey();
}

