/*======================================================================
*   Copyright (C) 2017 Institute of Cyber-Systems & Control.
*                       All rights reserved.
*
*           File Name: camera_calib.cpp
*              Author: Dr. Yongsheng Zhao
*               Email: zhaoyongsheng@zju.edu.cn
*       Creating Data: 09 19, 2017
*         Discription:
*
======================================================================*/
#include "camera_calib.h"

//! [compute_errors]
double Camera_Calib::computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors, bool fisheye)
{
    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]

//! [board_corners]
void Camera_Calib::calcBoardCornerPositions(cv::Size boardSize, float squareWidth, float squareHeight, std::vector<cv::Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(cv::Point3f(j*squareWidth, i*squareHeight, 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(cv::Point3f((2*j + i % 2)*squareWidth, i*squareHeight, 0));
        break;
    default:
        break;
    }
}

//! [board_corners]
bool Camera_Calib::runCalibration( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                            std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                            std::vector<float>& reprojErrs,  double& totalAvgErr)
{
    //! [fixed_aspect]
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if( s.flag & cv::CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    }

    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareWidth, s.squareHeight, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        cv::Mat _rvecs, _tvecs;
        rms = cv::fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                              s.flag);
    }

    std::cout << "Re-projection error reported by calibrateCamera: "<< rms << std::endl;

    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

    std::cout << "Range check is OK."<< std::endl;

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs, s.useFisheye);
    std::cout << "Re-projection error self-computed: " << totalAvgErr << std::endl;

    return ok;
}
// Print camera parameters to the output file
void Camera_Calib::saveCameraParams( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                              const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                              const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
                              double totalAvgErr )
{
    cv::FileStorage fs( s.outputFileName, cv::FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_width" << s.squareWidth;
    fs << "square_height" << s.squareHeight;

    if( s.flag & cv::CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                << (s.flag & cv::fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & cv::fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        cv::Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            cv::Mat r = bigmat(cv::Range(int(i), int(i+1)), cv::Range(0,3));
            cv::Mat t = bigmat(cv::Range(int(i), int(i+1)), cv::Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

//! [run_and_save]
bool Camera_Calib::runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                           std::vector<std::vector<cv::Point2f> > imagePoints)
{
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);
    std::cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr << std::endl;

    if (ok)
    {
      saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);
    }
    return ok;
}
//! [run_and_save]

bool Camera_Calib::camCalibration()
{
  if (!s.goodInput)
  {
    std::cerr<<"Configuration parameters are not loaded correctly."<<std::endl;
    return false;
  }
  clock_t prevTimestamp = 0;
  for (;;)
  {
    view.release();
    viewGray.release();
    pointBuf.clear();

    bool blinkOutput = false;

    if (imagePoints.size() < (size_t)s.nrFrames)
    {
      view = s.nextImage();
      //---------- Parse the new loaded image----------//
      imageSize = view.size();  // Format input image.
      if( s.flipVertical )
        cv::flip( view, view, 0 );

      // Find Chessboard Corners
      //std::vector<cv::Point2f> pointBuf;

      bool found;

      int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

      if(!s.useFisheye) {
          // fast check erroneously fails with high distortions like fisheye
          chessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
      }

      switch( s.calibrationPattern ) // Find feature points on the input format
      {
      case Settings::CHESSBOARD:
          found = findChessboardCorners( view, s.boardSize, pointBuf, chessBoardFlags);
          break;
      case Settings::CIRCLES_GRID:
          found = findCirclesGrid( view, s.boardSize, pointBuf );
          break;
      case Settings::ASYMMETRIC_CIRCLES_GRID:
          found = findCirclesGrid( view, s.boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID );
          break;
      default:
          found = false;
          break;
      }

      //! [pattern_found]
      if ( found)                // If done with success,
      {
        // improve the found corners' coordinate accuracy for chessboard
        if( s.calibrationPattern == Settings::CHESSBOARD)
        {
            cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, cv::Size(11,11),
                cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
        }

        if( mode == CAPTURING &&  // For camera only take new samples after delay time
            (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointBuf);
            prevTimestamp = clock();
            blinkOutput = s.inputCapture.isOpened();
        }

        // Draw the corners.

        drawChessboardCorners( view, s.boardSize, cv::Mat(pointBuf), found );
      }

      //----------------------------- Output Text ------------------------------------------------
      //! [output_text]
      std::string msg = (mode == CAPTURING) ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
      int baseLine = 0;
      cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
      cv::Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

      if( mode == CAPTURING )
      {
          if(s.showUndistorsed)
              msg = cv::format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
          else
              msg = cv::format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
      }

      cv::putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

      if( blinkOutput )
        cv::bitwise_not(view, view);

      //------------------------------ Show image and check for input commands -------------------
      //! [await_input]
      cv::namedWindow("Image View", cv::WINDOW_NORMAL);
      cv::resizeWindow("Image View", 1400, 1050);
      cv::imshow("Image View", view);
      char key = (char)cv::waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

      if( key  == ESC_KEY )
          break;

      if( key == 'u' && mode == CALIBRATED )
         s.showUndistorsed = !s.showUndistorsed;

      if( s.inputCapture.isOpened() && key == 'g' )
      {
          mode = CAPTURING;
          imagePoints.clear();
      }
      //! [await_input]
    }

    //-----  If no more image, or got enough, then do calibration and show result -------------
    if( mode == CAPTURING && imagePoints.size() == (size_t)s.nrFrames )
    {
      if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
      {
        mode = CALIBRATED;
      }
      else
      {
        mode = DETECTION;
      }
    }
    if(view.empty())          // If there are no more images stop the loop
    {
      // if calibration threshold was not reached yet, calibrate now
      if( mode != CALIBRATED && !imagePoints.empty() )
      {
        if(runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
        {
          mode = CALIBRATED;
          break;
        }
        else
          break;
      }
    }


    //------------------------- Video capture  output  undistorted ------------------------------
    //! [output_undistorted]
    if( mode == CALIBRATED && s.showUndistorsed )
    {
        cv::Mat temp = view.clone();
        if (s.useFisheye)
        {
          cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
          break;
        }
        else
        {
          //std::cout << "Test Before." << std::endl;
          cv::undistort(temp, view, cameraMatrix, distCoeffs);
          cv::namedWindow("Undistored", cv::WINDOW_NORMAL);
          cv::resizeWindow("Undistored", 700, 525);
          cv::imshow("Undistored", view);
          cv::namedWindow("Image", cv::WINDOW_NORMAL);
          cv::resizeWindow("Image", 700, 525);
          cv::imshow("Image",temp);
          cv::waitKey(0);
          //std::cout << "Test After." << std::endl;
          break;
        }
    }

  }
  if (mode == CALIBRATED)
    return true;
  else
    return false;
}

bool Camera_Calib::posEstimation(cv::InputArray objectPoints, cv::InputArray imagePoints, cv::OutputArray rvec, cv::OutputArray tvec)
{
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  loadCameraParams<cv::Mat>(s.outputFileName, "camera_matrix", cameraMatrix);
  loadCameraParams<cv::Mat>(s.outputFileName, "distortion_coefficients", distCoeffs);
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  return true;
}
