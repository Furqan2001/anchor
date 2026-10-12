/*
  Furqan Abid
  March 2026

  Shared header containing rendering functions for projecting virtual
  3D objects onto a camera frame. Provides a helper to draw 3D line
  segments, a function to draw labeled XYZ axes at the target origin,
  and a function to draw a wireframe rocket composed of a four-sided
  prism body, nose cone, two triangular fins, and an exhaust nozzle.
*/

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

/*
  Projects a set of 3D points onto the image plane and draws line
  segments between specified pairs of projected points. Used internally
  by drawAxes and drawVirtualObject to render wireframe geometry.

  Arguments:
  cv::Mat& frame                              - output image to draw on
  const std::vector<cv::Vec3f>& pts           - 3D points in world coordinates
  const std::vector<std::pair<int,int>>& edges - index pairs defining line segments
  const cv::Mat& rvec                         - rotation vector from solvePnP
  const cv::Mat& tvec                         - translation vector from solvePnP
  const cv::Mat& K                            - camera intrinsic matrix
  const std::vector<double>& D                - distortion coefficients
  cv::Scalar color                            - line color in BGR
  int thickness                               - line thickness in pixels
*/
static void drawLines3D(cv::Mat& frame,
                        const std::vector<cv::Vec3f>& pts,
                        const std::vector<std::pair<int,int>>& edges,
                        const cv::Mat& rvec, const cv::Mat& tvec,
                        const cv::Mat& K,
                        const std::vector<double>& D,
                        cv::Scalar color, int thickness = 2)
{
    std::vector<cv::Point2f> img;
    cv::projectPoints(pts, rvec, tvec, K, D, img);
    for (auto& [a, b] : edges)
        cv::line(frame, img[a], img[b], color, thickness);
}

/*
  Draws labeled 3D coordinate axes at the world origin onto the frame.
  The X axis is drawn in red, Y in green, and Z in blue, each as an
  arrowed line with a text label at the tip.

  Arguments:
  cv::Mat& frame                - output image to draw on
  const cv::Mat& rvec           - rotation vector from solvePnP
  const cv::Mat& tvec           - translation vector from solvePnP
  const cv::Mat& K              - camera intrinsic matrix
  const std::vector<double>& D  - distortion coefficients
  float axis_len                - length of each axis in world units
*/
void drawAxes(cv::Mat& frame,
                     const cv::Mat& rvec, const cv::Mat& tvec,
                     const cv::Mat& K, const std::vector<double>& D,
                     float axis_len = 3.0f)
{
    std::vector<cv::Vec3f> pts = {
        {0,        0,        0},
        {axis_len, 0,        0},
        {0,        axis_len, 0},
        {0,        0,        axis_len}
    };
    std::vector<cv::Point2f> img;
    cv::projectPoints(pts, rvec, tvec, K, D, img);

    cv::arrowedLine(frame, img[0], img[1], {0,   0,   255}, 3, 8, 0, 0.2);
    cv::arrowedLine(frame, img[0], img[2], {0,   255, 0  }, 3, 8, 0, 0.2);
    cv::arrowedLine(frame, img[0], img[3], {255, 100, 0  }, 3, 8, 0, 0.2);

    cv::putText(frame, "X", img[1] + cv::Point2f(5, 0),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,   0,   255}, 2);
    cv::putText(frame, "Y", img[2] + cv::Point2f(5, 0),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,   255, 0  }, 2);
    cv::putText(frame, "Z", img[3] + cv::Point2f(5, 0),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 100, 0  }, 2);
}

/*
  Draws a wireframe rocket floating above the target at a specified
  position in world coordinates. The rocket consists of a four-sided
  prism body drawn in white, a nose cone drawn in cyan, two triangular
  fins drawn in orange on opposite sides of the base, and a narrower
  exhaust nozzle drawn in red below the base.

  Arguments:
  cv::Mat& frame                - output image to draw on
  const cv::Mat& rvec           - rotation vector from solvePnP
  const cv::Mat& tvec           - translation vector from solvePnP
  const cv::Mat& K              - camera intrinsic matrix
  const std::vector<double>& D  - distortion coefficients
  float cx                      - X center of the rocket base in world units
  float cy                      - Y center of the rocket base in world units
  float base_z                  - Z height where the rocket base sits
  float r                       - radius of the rocket body
  float body_h                  - height of the cylindrical body
  float cone_h                  - height of the nose cone above the body
*/
void drawVirtualObject(cv::Mat& frame,
                               const cv::Mat& rvec, const cv::Mat& tvec,
                               const cv::Mat& K, const std::vector<double>& D,
                               float cx,
                               float cy,
                               float base_z,
                               float r,
                               float body_h,
                               float cone_h)
{
    // Body: four-sided prism with bottom ring and top ring
    std::vector<cv::Vec3f> body_pts = {
        {cx - r, cy - r, base_z},
        {cx + r, cy - r, base_z},
        {cx + r, cy + r, base_z},
        {cx - r, cy + r, base_z},
        {cx - r, cy - r, base_z + body_h},
        {cx + r, cy - r, base_z + body_h},
        {cx + r, cy + r, base_z + body_h},
        {cx - r, cy + r, base_z + body_h},
    };
    drawLines3D(frame, body_pts,
                {{0,1},{1,2},{2,3},{3,0},
                 {4,5},{5,6},{6,7},{7,4},
                 {0,4},{1,5},{2,6},{3,7}},
                rvec, tvec, K, D, {255, 255, 255}, 2);

    // Nose cone: four lines from top ring corners to a single apex
    std::vector<cv::Vec3f> cone_pts = {
        {cx - r, cy - r, base_z + body_h},
        {cx + r, cy - r, base_z + body_h},
        {cx + r, cy + r, base_z + body_h},
        {cx - r, cy + r, base_z + body_h},
        {cx,     cy,     base_z + body_h + cone_h},
    };
    drawLines3D(frame, cone_pts,
                {{0,4},{1,4},{2,4},{3,4}},
                rvec, tvec, K, D, {0, 200, 255}, 2);

    // Two triangular fins on opposite sides of the base
    float fin_out = r * 2.2f;
    float fin_mid = base_z + body_h * 0.25f;

    std::vector<cv::Vec3f> fin_front = {
        {cx - r, cy - r, base_z},
        {cx + r, cy - r, base_z},
        {cx,     cy - fin_out, fin_mid},
    };
    drawLines3D(frame, fin_front, {{0,2},{1,2},{0,1}},
                rvec, tvec, K, D, {0, 100, 255}, 2);

    std::vector<cv::Vec3f> fin_back = {
        {cx - r, cy + r, base_z},
        {cx + r, cy + r, base_z},
        {cx,     cy + fin_out, fin_mid},
    };
    drawLines3D(frame, fin_back, {{0,2},{1,2},{0,1}},
                rvec, tvec, K, D, {0, 100, 255}, 2);

    // Exhaust nozzle: narrower prism below the body base
    float nr = r * 0.5f;
    float nozzle_z = base_z - 0.4f;
    std::vector<cv::Vec3f> nozzle_pts = {
        {cx - nr, cy - nr, base_z},
        {cx + nr, cy - nr, base_z},
        {cx + nr, cy + nr, base_z},
        {cx - nr, cy + nr, base_z},
        {cx - nr, cy - nr, nozzle_z},
        {cx + nr, cy - nr, nozzle_z},
        {cx + nr, cy + nr, nozzle_z},
        {cx - nr, cy + nr, nozzle_z},
    };
    drawLines3D(frame, nozzle_pts,
                {{0,1},{1,2},{2,3},{3,0},
                 {4,5},{5,6},{6,7},{7,4},
                 {0,4},{1,5},{2,6},{3,7}},
                rvec, tvec, K, D, {0, 0, 220}, 2);
}