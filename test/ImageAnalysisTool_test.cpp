/*
 * ImageAnalysisTool_test.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tiagotrocoli
 */
#define BOOST_TEST_MODULE "ImageAnalysisTool_test"
#include <boost/test/unit_test.hpp>

#include <iostream>
#include <base/samples/Frame.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <frame_helper/FrameHelper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TestTools.hpp"
#include <fps_per_velocity/ImageAnalysisTool.hpp>
#include <fps_per_velocity/CLAHE.hpp>

using namespace fps_per_velocity;

pocolog_cpp::InputDataStream* video_stream = 0;
pocolog_cpp::InputDataStream* dvl_stream = 0;

float percent_image_size = 0.25;
float data_stream_size = 0.25;

BOOST_AUTO_TEST_CASE(first_testcase) {

  std::string temp_path = std::string(PATH_LOG_PAYLOAD) + "/"
      + std::string(LOG_CAMERA_DOWN_LEFT);
  std::string temp_stream = std::string(STREAM_CAMERA_DOWN_LEFT);

  video_stream = TestTools::getDataStreamFromLog(temp_path, temp_stream);

  cv::Mat image_previus = TestTools::getMatFromStream<
      base::samples::frame::Frame>(video_stream, 0, percent_image_size);

  SingleImageHazeRemoval *hazeRemoval = new CLAHE();
  ImageAnalysisTool image_analysis;
  cv::Mat image_current;
  for (int i = 1; i < video_stream->getSize() * data_stream_size; i += 1) {
    if (i != 1)
      image_current.copyTo(image_previus);
    else
      hazeRemoval->applyHazeRemoval(image_previus);

    image_current = TestTools::getMatFromStream<base::samples::frame::Frame>(
        video_stream, i, percent_image_size);

    hazeRemoval->applyHazeRemoval(image_current);
    image_analysis.siftAnalysis(image_previus, image_current);
  }

}

BOOST_AUTO_TEST_CASE(TestHazeRemovalUnderwaterImages_testcase) {

}
