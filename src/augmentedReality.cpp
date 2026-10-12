/*
  Furqan Abid
  March 2026

  Augmented reality program that loads camera calibration parameters,
  detects a checkerboard target in a live video feed, estimates the
  board's 3D pose using solvePnP, and overlays either 3D coordinate
  axes or a virtual rocket object onto the scene in real time. The user
  can switch between rendering modes with keyboard controls.
*/

#include <opencv2/opencv.hpp>
#include "virtualObject.h"
#include <iostream>
#include <vector>
#include <iomanip>

static const int    BOARD_COLS = 9;
static const int    BOARD_ROWS = 6;
static const cv::Size BOARD_SIZE(BOARD_COLS, BOARD_ROWS);

/*
  Constructs the fixed set of 3D world coordinates for all internal
  corners of the checkerboard target. Each square is treated as one
  unit, with the origin at the top-left corner. X increases to the
  right and Y decreases downward so that Z points toward the viewer.

  Returns: vector of 3D points in world coordinates
*/
std::vector<cv::Vec3f> buildWorldPoints() {
    std::vector<cv::Vec3f> pts;
    pts.reserve(BOARD_COLS * BOARD_ROWS);
    for (int r = 0; r < BOARD_ROWS; ++r)
        for (int c = 0; c < BOARD_COLS; ++c)
            pts.emplace_back((float)c, -(float)r, 0.0f);
    return pts;
}

/*
  Main function that loads intrinsic calibration parameters from a file,
  opens a live video stream, and runs a loop that detects the checkerboard
  target, estimates its pose, and draws the selected virtual overlay.
  The user presses '2' for 3D axes, '3' for the virtual rocket, or 'q'
  to quit.

  Returns: 0 on success, 1 on error
*/
int main() {
    std::vector<double> dist_coeffs;
    cv::Mat camera_matrix;
    {
        cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "ERROR: intrinsics.yml not found. Run calibration first.\n";
            return 1;
        }
        fs["camera_matrix"] >> camera_matrix;
        fs["dist_coeffs"]   >> dist_coeffs;
        fs.release();
    }
    
    std::cout << "Camera matrix:\n" << camera_matrix << "\n\n";

    const std::vector<cv::Vec3f> world_pts = buildWorldPoints();

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { std::cerr << "ERROR: Cannot open camera.\n"; return 1; }

    int mode = 2;
    std::cout << "Controls: 2=axes  3=rocket  q=quit\n\n";

    cv::Mat frame, gray;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        const char* labels[] = {"", "", "3D Axes", "Virtual Rocket"};
        cv::putText(frame, std::string("Mode: ") + labels[mode],
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);

        // Detect checkerboard corners and refine to sub-pixel accuracy
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                                 30, 0.001));

            // Estimate the board's rotation and translation relative to the camera
            cv::Mat rvec, tvec;
            cv::solvePnP(world_pts, corners, camera_matrix, dist_coeffs,
                         rvec, tvec);

            std::cout << "\rrvec: ["
                      << std::fixed << std::setprecision(3)
                      << rvec.at<double>(0) << ", "
                      << rvec.at<double>(1) << ", "
                      << rvec.at<double>(2) << "]  tvec: ["
                      << tvec.at<double>(0) << ", "
                      << tvec.at<double>(1) << ", "
                      << tvec.at<double>(2) << "]   " << std::flush;

            if (mode == 2)
                drawAxes(frame, rvec, tvec, camera_matrix, dist_coeffs, 3.0f);
            else if (mode == 3)
                drawVirtualObject(frame, rvec, tvec, camera_matrix, dist_coeffs);
        } else {
            cv::putText(frame, "Target not found", {10, 60},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        }

        cv::imshow("AR", frame);
        char key = (char)cv::waitKey(1);
        if (key == 'q') break;
        if (key == '2' || key == '3') mode = key - '0';
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}