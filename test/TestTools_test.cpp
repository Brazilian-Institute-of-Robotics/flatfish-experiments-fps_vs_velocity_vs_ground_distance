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
#include <base/samples/RigidBodyState.hpp>
#include <frame_helper/FrameHelper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TestTools.hpp"

pocolog_cpp::InputDataStream* video_stream = 0;
pocolog_cpp::InputDataStream* dvl_stream = 0;

BOOST_AUTO_TEST_CASE(getDataStreamFromLogVideo_testcase) {
  const std::string temp_path = std::string(PATH_LOG_PAYLOAD) + "/"
      + std::string(LOG_CAMERA_DOWN_LEFT);
  std::string temp_stream = std::string(STREAM_CAMERA_DOWN_LEFT);

  video_stream = TestTools::getDataStreamFromLog(temp_path, temp_stream);
  if (!video_stream)
    BOOST_FAIL("Error do not open camera log file");

  base::samples::frame::Frame frame_rock;
  frame_rock = TestTools::getSampleFromDataStream<base::samples::frame::Frame>(
      video_stream, 1000);

  cv::Mat frame_cv = frame_helper::FrameHelper::convertToCvMat(frame_rock);
  cv::cvtColor(frame_cv, frame_cv, CV_BayerGR2RGB);

  if (!frame_cv.data)
    BOOST_FAIL("No data in OpenCV image");
}

BOOST_AUTO_TEST_CASE(getDataStreamFromLogDVL_testcase) {
  const std::string temp_path = std::string(PATH_LOG_NAV) + "/"
      + std::string(LOG_DVL);
  std::string temp_stream = std::string(STREAM_DVL_VELOCITY);

  dvl_stream = TestTools::getDataStreamFromLog(temp_path, temp_stream);
  if (!dvl_stream)
    BOOST_FAIL("Error do not open log file");

  base::samples::RigidBodyState body_state;
  body_state =
      TestTools::getSampleFromDataStream<base::samples::RigidBodyState>(
          dvl_stream, 1000);

  if (!body_state.hasValidVelocity())
    BOOST_FAIL("No data in RigidBodyState");

}

BOOST_AUTO_TEST_CASE(syncPocologStreamsByTime_testcase) {

  std::map<int, int> sync_map = TestTools::syncPocologStreamsByTime<
      base::samples::frame::Frame, base::samples::RigidBodyState>(video_stream,
                                                                  dvl_stream,
                                                                  0.01);
  base::samples::RigidBodyState body_state;
  base::samples::frame::Frame frame_rock;
  for (uint i = 0; i < sync_map.size(); i += 20) {

    frame_rock =
        TestTools::getSampleFromDataStream<base::samples::frame::Frame>(
            video_stream, i);

    body_state = TestTools::getSampleFromDataStream<
        base::samples::RigidBodyState>(dvl_stream, sync_map.find(i)->second);

    BOOST_CHECK_CLOSE(frame_rock.time.microseconds * 1.0,
                      body_state.time.microseconds * 1.0, 0.0000001);

  }
}

