#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include "lane_detector.h"
#include "utils.h"

// Helper function to create directory if it doesn't exist
bool createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory doesn't exist, create it
        #ifdef _WIN32
            return mkdir(path.c_str()) == 0;
        #else
            return mkdir(path.c_str(), 0755) == 0;
        #endif
    } else if (info.st_mode & S_IFDIR) {
        // Directory already exists
        return true;
    }
    return false;
}

// Helper function to extract directory from file path
std::string getDirectory(const std::string& filepath) {
    size_t pos = filepath.find_last_of("/\\");
    if (pos != std::string::npos) {
        return filepath.substr(0, pos);
    }
    return ".";
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "Options:\n"
              << "  --video=<path>     Path to video file (default: webcam)\n"
              << "  --output=<path>    Path to save output video (optional)\n"
              << "  --camera=<id>      Camera device ID (default: 0)\n"
              << "  --help             Show this help message\n"
              << "\nControls:\n"
              << "  ESC or 'q'         Quit\n"
              << "  SPACE              Pause/Resume\n"
              << "  'r'                Show/Hide ROI\n"
              << std::endl;
}

int main(int argc, char** argv) {
    // Parse command line arguments
    std::string video_path;
    std::string output_path;
    int camera_id = 0;
    bool use_camera = true;
    bool save_output = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.find("--video=") == 0) {
            video_path = arg.substr(8);
            use_camera = false;
        } else if (arg.find("--output=") == 0) {
            output_path = arg.substr(9);
            save_output = true;
        } else if (arg.find("--camera=") == 0) {
            camera_id = std::stoi(arg.substr(9));
            use_camera = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // Open video source
    cv::VideoCapture cap;
    if (use_camera) {
        cap.open(camera_id);
        std::cout << "Opening camera " << camera_id << "..." << std::endl;
    } else {
        cap.open(video_path);
        std::cout << "Opening video file: " << video_path << "..." << std::endl;
    }

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video source!" << std::endl;
        return -1;
    }

    // Get video properties
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps == 0) fps = 30; // Default for webcam

    std::cout << "Video properties:" << std::endl;
    std::cout << "  Resolution: " << frame_width << "x" << frame_height << std::endl;
    std::cout << "  FPS: " << fps << std::endl;

    // Initialize video writer if output is requested
    cv::VideoWriter writer;
    if (save_output) {
        // Create output directory if it doesn't exist
        std::string output_dir = getDirectory(output_path);
        if (!createDirectory(output_dir)) {
            std::cerr << "Warning: Could not create output directory: " << output_dir << std::endl;
            std::cerr << "Attempting to save anyway..." << std::endl;
        }

        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        writer.open(output_path, fourcc, fps, cv::Size(frame_width, frame_height));
        if (!writer.isOpened()) {
            std::cerr << "Error: Could not open output video file: " << output_path << std::endl;
            std::cerr << "Make sure the output directory exists and is writable." << std::endl;
            return -1;
        }
        std::cout << "Saving output to: " << output_path << std::endl;
    }

    // Initialize lane detector
    LaneDetector detector;
    detector.setROI(50, 300);
    detector.setCannyThresholds(50, 150);
    detector.setHoughParams(50, 50, 150);

    // Create resizable window
    cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Lane Detection", 1200, 800);

    // Processing loop
    cv::Mat frame;
    bool paused = false;
    bool show_roi = false;
    int frame_count = 0;

    std::cout << "\nProcessing video...\n"
              << "Press ESC or 'q' to quit, SPACE to pause, 'r' to toggle ROI\n"
              << "Close window with X button to exit\n"
              << std::endl;

    while (true) {
        if (!paused) {
            cap >> frame;
            if (frame.empty()) {
                std::cout << "End of video or empty frame." << std::endl;
                break;
            }
            frame_count++;
        }

        // Create a copy for drawing
        cv::Mat display_frame = frame.clone();

        // Detect lanes
        LaneLines lanes = detector.detectLanes(frame);

        // Draw lanes
        utils::drawLanes(display_frame, lanes);

        // Draw ROI if enabled
        if (show_roi) {
            utils::drawROI(display_frame, 50, 300);
        }

        // Add info overlay
        utils::addInfoOverlay(display_frame, lanes);

        // Add frame counter
        std::string frame_info = "Frame: " + std::to_string(frame_count);
        cv::putText(display_frame, frame_info, cv::Point(frame_width - 150, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // Display frame
        cv::imshow("Lane Detection", display_frame);

        // Check if window was closed (X button)
        if (cv::getWindowProperty("Lane Detection", cv::WND_PROP_VISIBLE) < 1) {
            std::cout << "Window closed." << std::endl;
            break;
        }

        // Save frame if output is enabled
        if (save_output && !paused) {
            writer.write(display_frame);
        }

        // Handle keyboard input
        int key = cv::waitKey(use_camera ? 1 : 30);
        if (key == 27 || key == 'q' || key == 'Q') {
            // ESC or 'q' to quit
            std::cout << "Quit requested." << std::endl;
            break;
        } else if (key == ' ') {
            // Space to pause/resume
            paused = !paused;
            std::cout << (paused ? "Paused" : "Resumed") << std::endl;
        } else if (key == 'r' || key == 'R') {
            // 'r' to toggle ROI display
            show_roi = !show_roi;
            std::cout << "ROI display: " << (show_roi ? "ON" : "OFF") << std::endl;
        }
    }

    // Cleanup
    cap.release();
    if (save_output) {
        writer.release();
        std::cout << "Output saved to: " << output_path << std::endl;
    }
    cv::destroyAllWindows();

    std::cout << "Processed " << frame_count << " frames." << std::endl;
    std::cout << "Program finished successfully." << std::endl;

    return 0;
}
