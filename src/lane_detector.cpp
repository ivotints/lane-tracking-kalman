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
    std::vector<Line> left_lines, right_lines;
    separateLeftRight(all_lines, left_lines, right_lines, frame_size);

    // Fit polynomial curves
    result.has_left = fitPolynomialLane(left_lines, result.left_a, result.left_b, result.left_c,
                                        result.left_points, frame_size.height);
    result.has_right = fitPolynomialLane(right_lines, result.right_a, result.right_b, result.right_c,
                                         result.right_points, frame_size.height);

    return result;
}

void LaneDetector::separateLeftRight(const std::vector<Line>& lines,
                                     std::vector<Line>& left, std::vector<Line>& right, const cv::Size& frame_size) {
    int frame_center_x = frame_size.width / 2;

    for (const auto& line : lines) {
        // Filter by angle - lanes should not be too horizontal
        if (std::abs(line.angle) < 25) {
            continue;
        }

        // Calculate line center position
        int line_center_x = (line.start.x + line.end.x) / 2;

        // Classify based on position and angle
        if (line_center_x < frame_center_x && line.angle < 0) {
            // Left side of frame with negative angle (left lane)
            left.push_back(line);
        } else if (line_center_x > frame_center_x && line.angle > 0) {
            // Right side of frame with positive angle (right lane)
            right.push_back(line);
        }
    }
}

bool LaneDetector::fitPolynomialLane(const std::vector<Line>& lines, double& a, double& b, double& c,
                                     std::vector<cv::Point>& curve_points, int frame_height) {
    if (lines.empty()) {
        return false;
    }

    // Collect all points from line segments
    std::vector<cv::Point> points;
    for (const auto& line : lines) {
        points.push_back(line.start);
        points.push_back(line.end);
    }

    if (points.size() < 3) {
        return false; // Need at least 3 points for polynomial fit
    }

    // Prepare matrices for polynomial fitting: x = a*y^2 + b*y + c
    cv::Mat A(points.size(), 3, CV_64F);
    cv::Mat B(points.size(), 1, CV_64F);

    for (size_t i = 0; i < points.size(); i++) {
        double y = points[i].y;
        double x = points[i].x;

        A.at<double>(i, 0) = y * y;  // y^2
        A.at<double>(i, 1) = y;       // y
        A.at<double>(i, 2) = 1;       // constant
        B.at<double>(i, 0) = x;
    }

    // Solve using least squares: (A^T * A)^-1 * A^T * B
    cv::Mat coeffs;
    cv::solve(A, B, coeffs, cv::DECOMP_SVD);

    a = coeffs.at<double>(0, 0);
    b = coeffs.at<double>(1, 0);
    c = coeffs.at<double>(2, 0);

    // Generate curve points for drawing
    curve_points.clear();
    int y_bottom = frame_height - roi_top_offset_;
    int y_top = frame_height - roi_top_offset_ - roi_height_;

    for (int y = y_top; y <= y_bottom; y += 3) {
        int x = static_cast<int>(a * y * y + b * y + c);

        // Check if x is within reasonable bounds
        if (x >= -100 && x < 1500) {
            curve_points.push_back(cv::Point(x, y));
        }
    }

    // Validate that we have a reasonable curve
    if (curve_points.size() < 10) {
        return false;
    }

    return true;
}
