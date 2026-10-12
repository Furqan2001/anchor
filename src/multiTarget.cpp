/*
  Furqan Abid
  March 2026

  Multi-target augmented reality program using ArUco markers. Detects
  multiple ArUco markers simultaneously in a live video feed, estimates
  a separate pose for each one using solvePnP, and renders a virtual
  rocket on top of every detected marker. Also provides a utility to
  generate and save printable ArUco marker images to disk.
*/

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "virtualObject.h"
#include <iostream>
#include <vector>
#include <map>

static const float MARKER_SIZE = 0.07f;

static const std::vector<cv::Scalar> MARKER_COLORS = {
    {0,   200, 255},
    {0,   255, 100},
    {0,   100, 255},
    {255, 200, 0  },
    {180, 0,   255},
};

/*
  Builds the four 3D world-space corner positions of a single ArUco
  marker centered at the origin in the Z=0 plane. The corners are
  ordered clockwise starting from the top-left.

  Arguments:
  float size - physical side length of the marker in world units

  Returns: vector of four 3D corner points
*/
std::vector<cv::Point3f> markerWorldCorners(float size) {
    float h = size / 2.0f;
    return {
        {-h,  h, 0},
        { h,  h, 0},
        { h, -h, 0},
        {-h, -h, 0},
    };
}

/*
  Generates three ArUco marker images from the DICT_4X4_50 dictionary,
  adds a white border around each for reliable detection, and saves
  them as marker_0.png, marker_1.png, and marker_2.png. The user
  should print these at a known physical size and set MARKER_SIZE
  in the code to match.
*/
void generateMarkers() {
    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    for (int id = 0; id < 3; ++id) {
        cv::Mat marker_img;
        cv::aruco::generateImageMarker(dict, id, 400, marker_img, 1);

        cv::Mat bordered;
        cv::copyMakeBorder(marker_img, bordered,
                           40, 40, 40, 40,
                           cv::BORDER_CONSTANT, cv::Scalar(255));

        std::string fname = "marker_" + std::to_string(id) + ".png";
        cv::imwrite(fname, bordered);
        std::cout << "Saved " << fname << "\n";
    }
    std::cout << "Print each marker at the same physical size.\n"
              << "Set MARKER_SIZE in code to match the printed side length (meters).\n";
}

/*
  Main function that loads camera intrinsics, initializes the ArUco
  detector, and runs a live video loop. On each frame it detects all
  visible ArUco markers, estimates each marker's pose with solvePnP,
  and draws 3D axes and a virtual rocket on top of each one. The user
  presses 'g' to generate printable marker images or 'q' to quit.

  Returns: 0 on success, 1 on error
*/
int main() {
    cv::Mat K;
    std::vector<double> D;
    {
        cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "ERROR: intrinsics.yml not found. Run calibration first.\n";
            return 1;
        }
        fs["camera_matrix"] >> K;
        fs["dist_coeffs"]   >> D;
        fs.release();
    }
    std::cout << "Intrinsics loaded.\n";

    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::DetectorParameters params;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    cv::aruco::ArucoDetector detector(dict, params);

    const std::vector<cv::Point3f> world_corners =
        markerWorldCorners(MARKER_SIZE);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { std::cerr << "ERROR: Cannot open camera.\n"; return 1; }

    std::cout << "Controls: 'g' generate markers | 'q' quit\n\n";

    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Detect all ArUco markers in the current frame
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids, rejected);

        if (!ids.empty())
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

        // Estimate pose and render a virtual rocket for each detected marker
        for (int i = 0; i < (int)ids.size(); ++i) {
            int id = ids[i];

            cv::Mat rvec, tvec;
            cv::solvePnP(world_corners, corners[i], K, D, rvec, tvec);

            cv::Scalar color = MARKER_COLORS[id % MARKER_COLORS.size()];

            drawAxes(frame, rvec, tvec, K, D, MARKER_SIZE * 0.8f);

            drawVirtualObject(frame, rvec, tvec, K, D,
                              /*cx=*/0.0f, /*cy=*/0.0f,
                              /*base_z=*/0.0f,
                              /*r=*/MARKER_SIZE * 0.4f,
                              /*body_h=*/MARKER_SIZE * 1.5f,
                              /*cone_h=*/MARKER_SIZE * 0.9f);

            cv::Point2f label_pos = corners[i][0] + cv::Point2f(5, -10);
            cv::putText(frame, "ID: " + std::to_string(id), label_pos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        }

        // Display marker count and controls overlay
        std::string hud = ids.empty()
            ? "No markers detected"
            : "Markers detected: " + std::to_string(ids.size());
        cv::putText(frame, hud, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
        cv::putText(frame, "[g] generate markers  [q] quit", {10, 60},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {200, 200, 200}, 1);

        cv::imshow("Multi Target AR", frame);

        char key = (char)cv::waitKey(1);
        if (key == 'q') break;
        if (key == 'g') generateMarkers();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}