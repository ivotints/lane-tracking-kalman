#include "lane_detector.h"
#include <algorithm>
#include <cmath>

LaneDetector::LaneDetector()
    : canny_low_(50),
      canny_high_(150),
      hough_threshold_(50),
      hough_min_line_length_(50),
      hough_max_line_gap_(150),
      roi_top_offset_(100),
      roi_height_(200) {
}

void LaneDetector::setROI(int roi_top_offset, int roi_height) {
    roi_top_offset_ = roi_top_offset;
    roi_height_ = roi_height;
}

void LaneDetector::setCannyThresholds(int low, int high) {
    canny_low_ = low;
    canny_high_ = high;
}

void LaneDetector::setHoughParams(int threshold, int min_line_length, int max_line_gap) {
    hough_threshold_ = threshold;
    hough_min_line_length_ = min_line_length;
    hough_max_line_gap_ = max_line_gap;
}

LaneLines LaneDetector::detectLanes(const cv::Mat& frame) {
    // Create color mask for white and yellow lanes
    cv::Mat color_mask = createColorMask(frame);

    // Preprocessing
    cv::Mat gray = preprocessFrame(frame);

    // Edge detection
    cv::Mat edges = detectEdges(gray);

    // Combine color mask with edges
    cv::Mat combined;
    cv::bitwise_or(edges, color_mask, combined);

    // Apply ROI
    cv::Mat roi_edges = applyROI(combined);

    // Detect lines using Hough transform
    std::vector<cv::Vec4i> lines = detectHoughLines(roi_edges);

    // Classify and average lines
    LaneLines lane_lines = classifyAndAverageLines(lines, frame.size());

    return lane_lines;
}

cv::Mat LaneDetector::preprocessFrame(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    return gray;
}

cv::Mat LaneDetector::createColorMask(const cv::Mat& frame) {
    // Convert to HLS color space (better for yellow detection)
    cv::Mat hls;
    cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);

    // White color mask in HLS
    cv::Mat white_mask;
    cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 255), white_mask);

    // Yellow color mask in HLS
    cv::Mat yellow_mask;
    cv::inRange(hls, cv::Scalar(10, 50, 100), cv::Scalar(40, 255, 255), yellow_mask);

    // Combine white and yellow masks
    cv::Mat combined_mask;
    cv::bitwise_or(white_mask, yellow_mask, combined_mask);

    // Apply Gaussian blur to smooth the mask
    cv::GaussianBlur(combined_mask, combined_mask, cv::Size(5, 5), 0);

    return combined_mask;
}

cv::Mat LaneDetector::detectEdges(const cv::Mat& gray) {
    cv::Mat edges;
    cv::Canny(gray, edges, canny_low_, canny_high_);
    return edges;
}

cv::Mat LaneDetector::applyROI(const cv::Mat& edges) {
    int height = edges.rows;
    int width = edges.cols;

    // Create mask for ROI
    cv::Mat mask = cv::Mat::zeros(edges.size(), CV_8UC1);

    // Define ROI as trapezoid
    int roi_top = height - roi_top_offset_ - roi_height_;
    int roi_bottom = height - roi_top_offset_;

    std::vector<cv::Point> roi_points;
    roi_points.push_back(cv::Point(width * 0.1, roi_bottom));
    roi_points.push_back(cv::Point(width * 0.4, roi_top));
    roi_points.push_back(cv::Point(width * 0.6, roi_top));
    roi_points.push_back(cv::Point(width * 0.9, roi_bottom));

    // Fill polygon
    std::vector<std::vector<cv::Point>> roi_poly;
    roi_poly.push_back(roi_points);
    cv::fillPoly(mask, roi_poly, cv::Scalar(255));

    // Apply mask
    cv::Mat roi_edges;
    cv::bitwise_and(edges, mask, roi_edges);

    return roi_edges;
}

std::vector<cv::Vec4i> LaneDetector::detectHoughLines(const cv::Mat& roi_edges) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roi_edges, lines, 1, CV_PI / 180,
                    hough_threshold_, hough_min_line_length_, hough_max_line_gap_);
    return lines;
}

LaneLines LaneDetector::classifyAndAverageLines(const std::vector<cv::Vec4i>& lines, const cv::Size& frame_size) {
    LaneLines result;
    result.has_left = false;
    result.has_right = false;

    if (lines.empty()) {
        return result;
    }

    // Convert to Line objects
    std::vector<Line> all_lines;
    for (const auto& line : lines) {
        all_lines.push_back(Line(cv::Point(line[0], line[1]), cv::Point(line[2], line[3])));
    }

    // Separate left and right lines
    separateLeftRight(all_lines, result.left_lines, result.right_lines);

    // Compute average lines
    result.has_left = computeAverageLine(result.left_lines, result.left_bottom,
                                         result.left_top, frame_size.width, frame_size.height);
    result.has_right = computeAverageLine(result.right_lines, result.right_bottom,
                                          result.right_top, frame_size.width, frame_size.height);

    return result;
}

void LaneDetector::separateLeftRight(const std::vector<Line>& lines,
                                     std::vector<Line>& left, std::vector<Line>& right) {
    for (const auto& line : lines) {
        // Filter by angle
        if (std::abs(line.angle) < 25) {
            // Line is too horizontal, skip
            continue;
        }

        // Negative angle -> left lane, positive angle -> right lane
        if (line.angle < 0) {
            left.push_back(line);
        } else {
            right.push_back(line);
        }
    }
}

bool LaneDetector::computeAverageLine(const std::vector<Line>& lines, cv::Point& bottom,
                                      cv::Point& top, int frame_width, int frame_height) {
    if (lines.empty()) {
        return false;
    }

    // Collect all points
    std::vector<cv::Point> points;
    for (const auto& line : lines) {
        points.push_back(line.start);
        points.push_back(line.end);
    }

    // Fit line
    cv::Vec4f line_params;
    cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    double vx = line_params[0];
    double vy = line_params[1];
    double x0 = line_params[2];
    double y0 = line_params[3];

    // Calculate line equation: y = mx + b
    if (std::abs(vx) < 1e-6) {
        return false; // Vertical line, skip
    }

    double m = vy / vx;
    double b = y0 - m * x0;

    // Calculate x coordinates for y = bottom and y = top
    int y_bottom = frame_height - roi_top_offset_;
    int y_top = frame_height - roi_top_offset_ - roi_height_;

    int x_bottom = static_cast<int>((y_bottom - b) / m);
    int x_top = static_cast<int>((y_top - b) / m);

    // Check if points are within frame bounds
    if (x_bottom < 0 || x_bottom >= frame_width || x_top < 0 || x_top >= frame_width) {
        return false;
    }

    bottom = cv::Point(x_bottom, y_bottom);
    top = cv::Point(x_top, y_top);

    return true;
}
