#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

struct Line {
    cv::Point start;
    cv::Point end;
    double angle;

    Line(cv::Point s, cv::Point e) : start(s), end(e) {
        angle = atan2(e.y - s.y, e.x - s.x) * 180.0 / CV_PI;
    }
};

struct LaneLines {
    bool has_left;
    bool has_right;

    // Polynomial coefficients: x = a*y^2 + b*y + c
    double left_a, left_b, left_c;
    double right_a, right_b, right_c;

    // Points for drawing the curve
    std::vector<cv::Point> left_points;
    std::vector<cv::Point> right_points;
};

class LaneDetector {
public:
    LaneDetector();

    // Main detection function
    LaneLines detectLanes(const cv::Mat& frame);

    // Configure ROI
    void setROI(int roi_top_offset, int roi_height);

    // Configure Canny thresholds
    void setCannyThresholds(int low, int high);

    // Configure Hough parameters
    void setHoughParams(int threshold, int min_line_length, int max_line_gap);

private:
    // Processing steps
    cv::Mat preprocessFrame(const cv::Mat& frame);
    cv::Mat createColorMask(const cv::Mat& frame);
    cv::Mat detectEdges(const cv::Mat& gray);
    cv::Mat applyROI(const cv::Mat& edges);
    std::vector<cv::Vec4i> detectHoughLines(const cv::Mat& roi_edges);
    LaneLines classifyAndAverageLines(const std::vector<cv::Vec4i>& lines, const cv::Size& frame_size);

    // Helper functions
    void separateLeftRight(const std::vector<Line>& lines, std::vector<Line>& left, std::vector<Line>& right, const cv::Size& frame_size);
    bool fitPolynomialLane(const std::vector<Line>& lines, double& a, double& b, double& c,
                           std::vector<cv::Point>& curve_points, int frame_height);

    // Parameters
    int canny_low_;
    int canny_high_;
    int hough_threshold_;
    int hough_min_line_length_;
    int hough_max_line_gap_;
    int roi_top_offset_;
    int roi_height_;
};

#endif // LANE_DETECTOR_H
