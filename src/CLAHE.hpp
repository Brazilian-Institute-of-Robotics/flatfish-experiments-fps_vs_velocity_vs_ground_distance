/*
 * CLAHE.h
 *
 *  Created on: Dec 14, 2015
 *      Author: tiagotrocoli
 */

#ifndef SOURCE_DIRECTORY__SRC_HISTOGRAM_CLAHE_H_
#define SOURCE_DIRECTORY__SRC_HISTOGRAM_CLAHE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "SingleImageHazeRemoval.hpp"

class CLAHE: public SingleImageHazeRemoval {
public:
    CLAHE();
    CLAHE(double clipLimit, cv::Size grid);
    ~CLAHE();

    cv::Mat applyHazeRemoval(cv::Mat image);
    std::string getSelectedMethod() {
        return "CLAHE";
    }

private:
    cv::Ptr<cv::CLAHE> clahe;
};

#endif /* SOURCE_DIRECTORY__SRC_HISTOGRAM_CLAHE_H_ */
