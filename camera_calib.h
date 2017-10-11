/*======================================================================
*   Copyright (C) 2017 Institute of Cyber-Systems & Control.
*                       All rights reserved.
*
*           File Name: camera_calib.h
*              Author: Dr. Yongsheng Zhao
*               Email: zhaoyongsheng@zju.edu.cn
*       Creating Data: 09 19, 2017
*         Discription: This file defines the camera calibration class
*                      using opencv.
*
======================================================================*/
#ifndef CAMERA_CALIB_H
#define CAMERA_CALIB_H

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include "settings.h"

class Camera_Calib
{
  public:
    Camera_Calib(Settings& _s):RED(0,0,255),GREEN(0,255,0),ESC_KEY(27)
    {
      s = _s;
      mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    }
    ~Camera_Calib(){}

    bool camCalibration();
    bool posEstimation(cv::InputArray objectPoints, cv::InputArray imagePoints, cv::OutputArray rvec, cv::OutputArray tvec);

    // 从配置文件中载入参数
    template <typename T>
    bool loadCameraParams(std::string conf_file_name, std::string param_name, T& params)
    {
      cv::FileStorage fs(conf_file_name, cv::FileStorage::READ);
      if(!fs.isOpened())
      {
        std::cerr << "Could not open camera parameter file: " << conf_file_name << std::endl;
        return false;
      }
      fs[param_name] >> params;
      return true;
    }

  private:

    enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
    std::vector<std::vector<cv::Point2f> > imagePoints;
    std::vector<bool> imagePtFlag;

    cv::Mat cameraMatrix, distCoeffs;
    cv::Size imageSize;
    int mode;
    cv::Mat view, viewGray;
    std::vector<cv::Point2f> pointBuf;
    bool found;
    Settings s;
    const cv::Scalar RED;
    const cv::Scalar GREEN;
    const char ESC_KEY;
    //! [compute_errors]
    double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                             const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                             const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                             const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                             std::vector<float>& perViewErrors, bool fisheye);
    //! [board_corners]
    void calcBoardCornerPositions(cv::Size boardSize, float squareWidth, float suqareHeight, std::vector<cv::Point3f>& corners,
                                         Settings::Pattern patternType /*= Settings::CHESSBOARD*/);
    //! [board_corners]
    bool runCalibration( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                                std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                                std::vector<float>& reprojErrs,  double& totalAvgErr);

    // Print camera parameters to the output file
    void saveCameraParams( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                                  const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                  const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                  double totalAvgErr );

    //! [run_and_save]
    bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                               std::vector<std::vector<cv::Point2f> > imagePoints);
};
#endif //CAMERA_CALIB_H


