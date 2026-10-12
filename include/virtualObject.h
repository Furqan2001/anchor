/*
  Furqan Abid
  March 2026

  Header file for virtual object rendering functions used in augmented
  reality. Provides interfaces for projecting 3D geometry onto a camera
  frame given a pose estimated by solvePnP.

  Supported operations:
    drawVirtualObject - renders a wireframe rocket above the target
    drawAxes          - draws labeled XYZ coordinate axes at the origin
*/

#ifndef VIRTUAL_OBJECT_H
#define VIRTUAL_OBJECT_H

#include <opencv2/opencv.hpp>

void drawVirtualObject(cv::Mat& frame,
                               const cv::Mat& rvec, const cv::Mat& tvec,
                               const cv::Mat& K, const std::vector<double>& D,
                               float cx     = 4.0f,
                               float cy     = -2.5f,
                               float base_z = 0.0f,
                               float r      = 0.6f,
                               float body_h = 2.5f,
                               float cone_h = 1.5f);

void drawAxes(cv::Mat& frame,
                     const cv::Mat& rvec, const cv::Mat& tvec,
                     const cv::Mat& K, const std::vector<double>& D,
                     float axis_len = 3.0f);

#endif