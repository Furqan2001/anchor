/*
  Furqan Abid
  March 2026

  Static image augmented reality program that loads camera calibration
  parameters, reads a user-specified image, detects a checkerboard
  target, estimates its pose using solvePnP, and renders a virtual
  rocket onto the scene. The user can save the resulting image to
  disk or quit through keyboard controls.
*/

#include <opencv2/opencv.hpp>
#include "virtualObject.h"
#include <iostream>
#include <vector>
#include <string>

static const int       BOARD_COLS = 9;
static const int       BOARD_ROWS = 6;
static const cv::Size  BOARD_SIZE(BOARD_COLS, BOARD_ROWS);

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
  Main function that takes an image file path as a command-line argument,
  loads camera intrinsics from intrinsics.yml, detects the checkerboard
  in the image, estimates its pose, and draws a virtual rocket centered
  above the board. The result is displayed in a window where the user
  presses 's' to save the output as static_ar_output.png or 'q' to quit.

  Arguments:
  int argc      - argument count (expects at least 2)
  char** argv   - argument vector; argv[1] is the path to the input image

  Returns: 0 on success, 1 on error
*/
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>\n";
        return 1;
    }

    std::string image_path = argv[1];

    // Load camera intrinsics and distortion coefficients from file
    cv::Mat camera_matrix;
    std::vector<double> dist_coeffs;
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
    std::cout << "Loaded camera matrix:\n" << camera_matrix << "\n\n";

    // Load the static image from disk
    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cerr << "ERROR: Could not read image: " << image_path << "\n";
        return 1;
    }
    std::cout << "Image loaded: " << frame.cols << " x " << frame.rows << "\n";

    // Detect checkerboard corners and refine to sub-pixel accuracy
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) {
        std::cerr << "ERROR: Checkerboard target not found in the image.\n";
        cv::imshow("Static AR - target not found", frame);
        cv::waitKey(0);
        return 1;
    }

    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                         30, 0.001));

    std::cout << "Found " << corners.size() << " corners.\n";

    // Estimate the board's rotation and translation relative to the camera
    const std::vector<cv::Vec3f> world_pts = buildWorldPoints();
    cv::Mat rvec, tvec;
    cv::solvePnP(world_pts, corners, camera_matrix, dist_coeffs, rvec, tvec);

    std::cout << "Pose:\n"
              << "  rvec: [" << rvec.at<double>(0) << ", "
              << rvec.at<double>(1) << ", " << rvec.at<double>(2) << "]\n"
              << "  tvec: [" << tvec.at<double>(0) << ", "
              << tvec.at<double>(1) << ", " << tvec.at<double>(2) << "]\n\n";

    // Draw the virtual rocket centered above the board
    float cx = (BOARD_COLS - 1) / 2.0f;
    float cy = -(BOARD_ROWS - 1) / 2.0f;
    drawVirtualObject(frame, rvec, tvec, camera_matrix, dist_coeffs,
                      cx, cy,
                      /*base_z=*/0.5f,
                      /*r=*/0.8f,
                      /*body_h=*/3.0f,
                      /*cone_h=*/1.5f);

    std::cout << "Virtual rocket drawn.\n"
              << "Press 's' to save, 'q' to quit.\n";

    // Display the result and wait for user input
    cv::imshow("Static AR", frame);

    while (true) {
        char key = (char)cv::waitKey(0);

        if (key == 's') {
            std::string out_path = "static_ar_output.png";
            cv::imwrite(out_path, frame);
            std::cout << "Output saved to: " << out_path << "\n";
        } else if (key == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}