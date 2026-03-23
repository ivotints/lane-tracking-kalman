#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include "lane_detector.h"

namespace utils {
    // Draw detected lanes on frame
    void drawLanes(cv::Mat& frame, const LaneLines& lanes);

    // Draw ROI for visualization
    void drawROI(cv::Mat& frame, int roi_top_offset, int roi_height);

    // Draw all detected lines (for debugging)
    void drawAllLines(cv::Mat& frame, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color);

    // Add text information overlay
    void addInfoOverlay(cv::Mat& frame, const LaneLines& lanes);
}

#endif // UTILS_H
