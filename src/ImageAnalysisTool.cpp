/*
 * ImageAnalysisTool.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tiagotrocoli
 */

#include "ImageAnalysisTool.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <string>
#include <vector>

namespace fps_per_velocity {

/* namespace fps_per_velocity */

ImageAnalysisTool::ImageAnalysisTool() {
}

// http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
// http://stackoverflow.com/questions/29472959/orb-giving-better-feature-matching-than-sift-why

double ImageAnalysisTool::calcOverlapPercent(cv::Mat target_image,
                                             cv::Mat query_image,
                                             cv::Mat homografy,
                                             cv::Mat *overlaped_image) {

  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cv::Point2f(0, 0);
  obj_corners[1] = cv::Point2f(target_image.cols, 0);
  obj_corners[2] = cv::Point2f(target_image.cols, target_image.rows);
  obj_corners[3] = cv::Point2f(0, target_image.rows);
  std::vector<cv::Point2f> scene_corners(4);

  cv::perspectiveTransform(obj_corners, scene_corners, homografy);

  cv::Mat1b image_mask = cv::Mat1b::zeros(query_image.size());

  for (int i = 0; i < image_mask.rows; ++i)
    for (int j = 0; j < image_mask.cols; ++j) {
      cv::pointPolygonTest(scene_corners, cv::Point2f(j, i), true) > 0 ?
          image_mask[i][j] = 1 : image_mask[i][j] = 0;
    }

//  for (int i = 0; i < 4; ++i)
//    cv::line(current_image, scene_corners[i], scene_corners[(i + 1) % 4],
//             cv::Scalar(0, 255, 0), 2);

  if (overlaped_image)
    query_image.copyTo((*overlaped_image), image_mask);

  cv::Scalar sum = cv::sum(image_mask);
  double area = sum[0] / (1.0 * image_mask.rows * image_mask.cols);

  return area;
}

void ImageAnalysisTool::writeMatrixRelation(cv::Mat relation,
                                            std::string path_name) {

  cv::FileStorage fs(path_name+".yml", cv::FileStorage::WRITE);
  time_t rawtime;
  time(&rawtime);
  fs << "analysis_time" << asctime(localtime(&rawtime));
  fs << "relationMatrix" << relation;
  fs.release();
}

}

