/*======================================================================
*   Copyright (C) 2017 Institute of Cyber-Systems & Control.
*                       All rights reserved.
*
*           File Name: settings.cpp
*              Author: Dr. Yongsheng Zhao
*               Email: zhaoyongsheng@zju.edu.cn
*       Creating Data: 09 19, 2017
*         Discription: This is a class to read and write data in XML/YAML
*                      format files for camera calibration using OpenCV.
*
======================================================================*/
#include "settings.h"

void Settings::write(cv::FileStorage& fs) const                        //Write serialization for this class
{
    fs << "{"
              << "BoardSize_Width"  << boardSize.width
              << "BoardSize_Height" << boardSize.height
              << "Square_Width"         << squareWidth
              << "Square_Height"  << squareHeight
              << "Calibrate_Pattern" << patternToUse
              << "Calibrate_NrOfFrameToUse" << nrFrames
              << "Calibrate_FixAspectRatio" << aspectRatio
              << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
              << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

              << "Write_DetectedFeaturePoints" << writePoints
              << "Write_extrinsicParameters"   << writeExtrinsics
              << "Write_outputFileName"  << outputFileName

              << "Show_UndistortedImage" << showUndistorsed

              << "Input_FlipAroundHorizontalAxis" << flipVertical
              << "Input_Delay" << delay
              << "Input" << input
       << "}";
}

void Settings::read(const cv::FileNode& node)                          //Read serialization for this class
{
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Width"]  >> squareWidth;
    node["Square_Height"] >> squareHeight;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorsed;
    node["Input"] >> input;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}

void Settings::validate()
{
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0)
    {
        std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
        goodInput = false;
    }
    if (squareWidth <= 10e-6)
    {
        std::cerr << "Invalid square size " << squareWidth << std::endl;
        goodInput = false;
    }
    if (nrFrames <= 0)
    {
        std::cerr << "Invalid number of frames " << nrFrames << std::endl;
        goodInput = false;
    }

    if (input.empty())      // Check for valid input
    {
      std::cerr<<"Input is empty."<< input << std::endl;
      inputType = INVALID;
    }
    else
    {
        if (input[0] >= '0' && input[0] <= '9')
        {
          std::stringstream ss(input);
            ss >> cameraID;
            inputType = CAMERA;
        }
        else
        {
            if (readStringList(input, imageList))
            {
                inputType = IMAGE_LIST;
                nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
            }
            else
            {
              std::cout<<"Could not parse the image list." << input << std::endl;
              inputType = VIDEO_FILE;
            }
        }
        if (inputType == CAMERA)
            inputCapture.open(cameraID);
        if (inputType == VIDEO_FILE)
            inputCapture.open(input);
        if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                inputType = INVALID;
    }
    if (inputType == INVALID)
    {
        std::cerr << " Input does not exist: " << input << std::endl;
        goodInput = false;
    }

    flag = 0;
    if(calibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= cv::CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= cv::CALIB_FIX_K1;
    if(fixK2)                  flag |= cv::CALIB_FIX_K2;
    if(fixK3)                  flag |= cv::CALIB_FIX_K3;
    if(fixK4)                  flag |= cv::CALIB_FIX_K4;
    if(fixK5)                  flag |= cv::CALIB_FIX_K5;

    if (useFisheye) {
        // the fisheye model has its own enum, so overwrite the flags
        flag = cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if(fixK1)                  flag |= cv::fisheye::CALIB_FIX_K1;
        if(fixK2)                  flag |= cv::fisheye::CALIB_FIX_K2;
        if(fixK3)                  flag |= cv::fisheye::CALIB_FIX_K3;
        if(fixK4)                  flag |= cv::fisheye::CALIB_FIX_K4;
    }

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING)
    {
        std::cerr << " Camera calibration mode does not exist: " << patternToUse << std::endl;
        goodInput = false;
    }
    atImageList = 0;
}
cv::Mat Settings::nextImage()
{
    cv::Mat result;
    if( inputCapture.isOpened() )
    {
        cv::Mat view0;
        inputCapture >> view0;
        view0.copyTo(result);
    }
    else if( atImageList < imageList.size() )
    {
        result = cv::imread(imageList[atImageList++]);
    }

    return result;
}
bool Settings::readStringList( const std::string& filename, std::vector<std::string>& l )
{
    l.clear();
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((std::string)*it);
    return true;
}
