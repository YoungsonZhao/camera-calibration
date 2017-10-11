/*======================================================================
*   Copyright (C) 2017 Institute of Cyber-Systems & Control.
*                       All rights reserved.
*
*           File Name: testCamCalibration.cpp
*              Author: Dr. Yongsheng Zhao
*               Email: zhaoyongsheng@zju.edu.cn
*       Creating Data: 09 20, 2017
*         Discription:
*
======================================================================*/
#include "settings.h"
#include "camera_calib.h"

int main(int argc, char** argv)
{
  Settings s;
  const std::string inputSettingFile = argc > 1 ? argv[1]:"../calibConf.xml";
  cv::FileStorage fs(inputSettingFile, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr<<"Could not open the configuration file:"<<inputSettingFile<<std::endl;
    return -1;
  }
  fs["Settings"]>>s;
  fs.release();
  if(!s.goodInput)
  {
    std::cerr<<"Invalid input detected, please check the configuration file:"<<inputSettingFile<<std::endl;
    return -1;
  }

  Camera_Calib camCalib(s);

  camCalib.camCalibration();

 // cv::Mat cameraMatrix;
 // int flags;
 // camCalib.loadCameraParams<int>(s.outputFileName, "flags", flags);
 // camCalib.loadCameraParams<cv::Mat>(s.outputFileName, "camera_matrix", cameraMatrix);
 // std::cout << flags <<std::endl;
 // std::cout << cameraMatrix << std::endl;
  return 0;

}


static inline void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
    {
      std::cout << "Starting parsing the configuration parameters..." << std::endl;
      x.read(node);
      std::cout << "Parsing the configuration parameters is finished." << std::endl;
    }
        x.read(node);
}

static inline void write(cv::FileStorage& fs, const cv::String&, const Settings& s )
{
    s.write(fs);
}
