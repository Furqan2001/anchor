/*
  Furqan Abid
  March 2026

  Harris corner feature detection program that runs on a live video
  stream. Computes the Harris corner response for each frame, normalizes
  it, and marks pixels above an adjustable threshold as detected corners.
  The user can raise or lower the threshold interactively to explore
  how corner strength varies across different surfaces and patterns.
*/

#include <opencv2/opencv.hpp>
#include <iostream>

/*
  Main function that opens a live video stream and runs a loop that
  computes the Harris corner response on each frame, draws circles at
  detected corner locations, and displays a threshold and corner count
  overlay. The user presses '+' or '-' to adjust the detection threshold
  and 'q' to quit.

  Returns: 0 on success, 1 on error
*/
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { std::cerr << "ERROR: Cannot open camera.\n"; return 1; }

    float thresh = 120.0f;
    const float THRESH_STEP = 10.0f;
    const float THRESH_MIN  = 10.0f;
    const float THRESH_MAX  = 240.0f;

    std::cout << "Controls: '+' raise threshold | '-' lower threshold | 'q' quit\n\n";

    cv::Mat frame, gray, gray_f, harris, harris_norm;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to grayscale and compute the Harris corner response
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        gray.convertTo(gray_f, CV_32F);
        cv::cornerHarris(gray_f, harris, 4, 3, 0.04);

        // Normalize response to [0, 255] for intuitive thresholding
        cv::normalize(harris, harris_norm, 0, 255, cv::NORM_MINMAX, CV_32F);

        // Draw a circle at each pixel whose response exceeds the threshold
        int count = 0;
        for (int r = 0; r < harris_norm.rows; ++r)
            for (int c = 0; c < harris_norm.cols; ++c)
                if (harris_norm.at<float>(r, c) > thresh) {
                    cv::circle(frame, {c, r}, 5, {0, 0, 255}, 1);
                    ++count;
                }

        // Display current threshold and corner count on the frame
        cv::putText(frame, "Harris Corner Detection",
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
        cv::putText(frame, "Threshold: " + std::to_string((int)thresh) +
                    "   Corners: " + std::to_string(count),
                    {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
        cv::putText(frame, "[+/-] adjust threshold",
                    {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {200, 200, 200}, 1);

        cv::imshow("Harris", frame);

        char key = (char)cv::waitKey(1);
        if (key == 'q') break;
        if (key == '+' || key == '=')
            thresh = std::min(thresh + THRESH_STEP, THRESH_MAX);
        if (key == '-' || key == '_')
            thresh = std::max(thresh - THRESH_STEP, THRESH_MIN);
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}