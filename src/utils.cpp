#include "utils.h"

namespace utils {

void drawLanes(cv::Mat& frame, const LaneLines& lanes) {
    // Draw left lane curve in green
    if (lanes.has_left && lanes.left_points.size() > 1) {
        for (size_t i = 0; i < lanes.left_points.size() - 1; i++) {
            cv::line(frame, lanes.left_points[i], lanes.left_points[i + 1],
                     cv::Scalar(0, 255, 0), 8);
        }
    }

    // Draw right lane curve in green
    if (lanes.has_right && lanes.right_points.size() > 1) {
        for (size_t i = 0; i < lanes.right_points.size() - 1; i++) {
            cv::line(frame, lanes.right_points[i], lanes.right_points[i + 1],
                     cv::Scalar(0, 255, 0), 8);
        }
    }
}

void drawROI(cv::Mat& frame, int roi_top_offset, int roi_height) {
    int height = frame.rows;
    int width = frame.cols;

    int roi_top = height - roi_top_offset - roi_height;
    int roi_bottom = height - roi_top_offset;

    std::vector<cv::Point> roi_points;
    roi_points.push_back(cv::Point(width * 0.1, roi_bottom));
    roi_points.push_back(cv::Point(width * 0.4, roi_top));
    roi_points.push_back(cv::Point(width * 0.6, roi_top));
    roi_points.push_back(cv::Point(width * 0.9, roi_bottom));

    // Draw polygon
    for (size_t i = 0; i < roi_points.size(); i++) {
        cv::line(frame, roi_points[i], roi_points[(i + 1) % roi_points.size()],
                 cv::Scalar(0, 255, 255), 2);
    }
}

void drawAllLines(cv::Mat& frame, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color) {
    for (const auto& line : lines) {
        cv::line(frame, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, 2);
    }
}

void addInfoOverlay(cv::Mat& frame, const LaneLines& lanes) {
    std::string left_status = lanes.has_left ? "LEFT: DETECTED" : "LEFT: NOT DETECTED";
    std::string right_status = lanes.has_right ? "RIGHT: DETECTED" : "RIGHT: NOT DETECTED";

    cv::Scalar left_color = lanes.has_left ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::Scalar right_color = lanes.has_right ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    // Add semi-transparent background
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, cv::Point(10, 10), cv::Point(350, 80), cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.5, frame, 0.5, 0, frame);

    // Draw text
    cv::putText(frame, left_status, cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, left_color, 2);
    cv::putText(frame, right_status, cv::Point(20, 65),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, right_color, 2);
}

} // namespace utils
