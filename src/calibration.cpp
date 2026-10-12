/*
  Furqan Abid
  March 2026

  Camera calibration program that detects checkerboard corners in a live
  video feed, lets the user select calibration frames, runs the OpenCV
  calibrateCamera routine to estimate intrinsic parameters and distortion
  coefficients, and saves the results to a YAML file. The user controls
  frame selection, calibration, and file output through keyboard commands.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <iomanip>

static const int BOARD_COLS = 9;
static const int BOARD_ROWS = 6;
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
  Prints a labeled cv::Mat of doubles to standard output in a formatted
  row-by-row layout for easy inspection of matrices like the camera
  intrinsic matrix.

  Arguments:
  const std::string& label - descriptive label printed before the matrix
  const cv::Mat& m         - matrix of type CV_64F to display

  Returns: nothing (prints to stdout)
*/
void printMat(const std::string& label, const cv::Mat& m) {
	std::cout << label << ":\n";
	for (int r = 0; r < m.rows; ++r) {
		std::cout << "  [";
		for (int c = 0; c < m.cols; ++c) {
			std::cout << std::setw(12) << std::fixed << std::setprecision(4)
								<< m.at<double>(r, c);
			if (c < m.cols - 1) std::cout << ", ";
		}
		std::cout << " ]\n";
	}
}

/*
  Main function that opens a live video stream and runs an interactive
  calibration loop. On each frame it attempts to detect checkerboard
  corners with sub-pixel refinement. The user presses 's' to save a
  frame for calibration, 'c' to run calibrateCamera once at least five
  frames have been collected, 'w' to write the resulting intrinsic
  parameters and distortion coefficients to intrinsics.yml, or 'q'
  to quit.

  Returns: 0 on success, 1 on error
*/
int main() {
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "ERROR: Cannot open camera.\n";
		return 1;
	}

	std::vector<std::vector<cv::Point2f>> corner_list;
	std::vector<std::vector<cv::Vec3f>>   point_list;
	const std::vector<cv::Vec3f> world_pts = buildWorldPoints();

	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1);
	std::vector<double> dist_coeffs;
	double reprojection_error = -1.0;
	bool calibrated = false;

	cv::Mat frame, gray;
	std::vector<cv::Point2f> last_corners;
	bool corners_found = false;

	std::cout << "=== Calibration Program ===\n"
						<< "  's' → save frame   'c' → calibrate   'w' → write file   'q' → quit\n\n";

	while (true) {
		cap >> frame;
		if (frame.empty()) break;

		// Detect checkerboard corners and refine to sub-pixel accuracy
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		std::vector<cv::Point2f> corner_set;
		bool found = cv::findChessboardCorners(
				gray, BOARD_SIZE, corner_set,
				cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
														30, 0.001));

			cv::drawChessboardCorners(frame, BOARD_SIZE, corner_set, found);

			std::cout << "\rCorners: " << corner_set.size()
								<< "  first=(" << std::fixed << std::setprecision(1)
								<< corner_set[0].x << ", " << corner_set[0].y << ")   "
								<< std::flush;

			last_corners  = corner_set;
			corners_found = true;
		} else {
			std::cout << "\r[no target]                                      " << std::flush;
			corners_found = false;
		}

		// Display saved frame count and reprojection error if calibrated
		std::string info = "Saved: " + std::to_string(corner_list.size());
		if (calibrated)
				info += "  RMS: " + std::to_string(reprojection_error).substr(0, 5);
		cv::putText(frame, info, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
								{0, 255, 0}, 2);

		cv::imshow("Calibration", frame);

		char key = (char)cv::waitKey(1);

		// Save the current corner set and corresponding world points
		if (key == 's') {
			if (!corners_found) {
				std::cout << "\n[skip] No corners in last frame.\n";
			} else {
				corner_list.push_back(last_corners);
				point_list.push_back(world_pts);
				std::cout << "\n[saved] Total calibration frames: "
									<< corner_list.size() << "\n";

				std::string fname = "calib_frame_" +
						std::to_string(corner_list.size()) + ".png";
				cv::imwrite(fname, frame);
			}
		}

		// Run calibration using all saved frames
		else if (key == 'c') {
			if ((int)corner_list.size() < 5) {
				std::cout << "\n[error] Need at least 5 frames (have "
									<< corner_list.size() << ").\n";
			} else {
				camera_matrix.at<double>(0, 2) = frame.cols / 2.0;
				camera_matrix.at<double>(1, 2) = frame.rows / 2.0;

				std::cout << "\n--- Camera matrix BEFORE calibration ---\n";
				printMat("camera_matrix", camera_matrix);

				std::vector<cv::Mat> rvecs, tvecs;
				cv::Size img_size = frame.size();

				reprojection_error = cv::calibrateCamera(
						point_list, corner_list, img_size,
						camera_matrix, dist_coeffs,
						rvecs, tvecs,
						cv::CALIB_FIX_ASPECT_RATIO);

				calibrated = true;
				std::cout << "\n--- Camera matrix AFTER calibration ---\n";
				printMat("camera_matrix", camera_matrix);
				std::cout << "\nDistortion coefficients: [";
				for (double d : dist_coeffs) std::cout << d << " ";
				std::cout << "]\n";
				std::cout << "Reprojection error (RMS): "
									<< reprojection_error << " px\n";
			}
		}

		// Write calibration results to a YAML file
		else if (key == 'w') {
			if (!calibrated) {
				std::cout << "\n[error] Run calibration first ('c').\n";
			} else {
				cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
				fs << "camera_matrix"   << camera_matrix;
				fs << "dist_coeffs"     << dist_coeffs;
				fs << "reprojection_error" << reprojection_error;
				fs.release();
				std::cout << "\n[saved] Intrinsics written to intrinsics.yml\n";
			}
		}

		else if (key == 'q') break;
	}

	cap.release();
	cv::destroyAllWindows();
	return 0;
}