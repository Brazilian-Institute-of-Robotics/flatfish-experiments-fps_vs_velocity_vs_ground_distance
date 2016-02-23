/*
 * TestTools.h
 *
 *  Created on: Feb 17, 2016
 *      Author: tiagotrocoli
 */

#ifndef DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_
#define DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_

#define PATH_RESOURCE "../../test/resource"
#define IMAGE_01 "image01.jpg"
#define IMAGE_02 "image02.jpg"

#define PATH_LOG_NAV "../../../../../../jarvis/Documents/BIR-dataset/Log/incoming/ff-nav/20151119-0935"
#define PATH_LOG_PAYLOAD "../../../../../../jarvis/Documents/BIR-dataset/Log/incoming/ff-payload/20151119-0935"

#define LOG_CAMERA_DOWN_LEFT "camera_aravis_down_left.2.log"
#define STREAM_CAMERA_DOWN_LEFT "camera_aravis_down_left.frame"

#define LOG_DVL "dvl_seapilot.0.log"
#define STREAM_DVL_VELOCITY "dvl_seapilot.velocity_samples" // /base/samples/RigidBodyState
#define STREAM_DVL_GROUND_DISTANCE "dvl_seapilot.ground_distance" // /base/samples/RigidBodyState
#define STREAM_DVL_TIME_STAMPgo "dvl_seapilot.timestamp_estimator_status" // /aggregator/TimestampEstimatorStatus


#endif /* DRIVERS_FPS_PER_VELOCITY_TEST_TESTTOOLS_HPP_ */
