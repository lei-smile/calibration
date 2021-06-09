#include <iostream>
#include <exception>
#include <cmath>
#include <fstream>
   
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;
//using namespace GeographicLib;

struct CalibData
{
    double u;
    double v;
    double x;
    double y;
    double z;
};

cv::Mat camera_mat;
cv::Mat dist_coeff;
cv::Mat project_mat;
cv::Mat map1, map2;
int image_width;
int image_height;
cv::Size im_size;

void ReadIntrinsicsProcess(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR:Wrong path to settings" << std::endl;
    }
    fsSettings["CameraMat"] >> camera_mat;
    fsSettings["DistCoeff"] >> dist_coeff;
    image_width = fsSettings["image_width"];
    image_height = fsSettings["image_height"];
    im_size = cv::Size(image_width,image_height);
    std::cout << "CameraMat \n" << camera_mat << std::endl;
    std::cout << "dist_coeff \n" << dist_coeff << std::endl;
    std::cout << "image_width : " << image_width << "  image_height : " << image_height <<std::endl;
    project_mat = getOptimalNewCameraMatrix(camera_mat, dist_coeff, im_size, 1, im_size, 0);
    std::cout << "project_mat \n" << project_mat << std::endl;
    // if(fsSettings["DistModel"] == "plumb_bob")
    //     cv::initUndistortRectifyMap(camera_mat, dist_coeff, Mat(), project_mat,im_size, CV_16SC2, map1, map2);
    // else
        cv::fisheye::initUndistortRectifyMap(camera_mat, dist_coeff, Mat(), project_mat,im_size, CV_16SC2, map1, map2);
}

bool ReadCalibData(std::string fileName, std::vector<CalibData>& points_calib, char separator = ',')
{
    points_calib.clear();
    std::ifstream ifile(fileName);
    if (!ifile.is_open())
    {
        return false;
    }
    std::string lineStr;
    bool first_point = true;
    double lat0 = 0;
    double lon0 = 0;
    double height0 = 0;
    while (std::getline(ifile, lineStr))
    {
        std::stringstream ss(lineStr);
        std::string str;
        CalibData dataInfo;
        // 按照逗号分隔
        std::getline(ss, str, separator); dataInfo.u = atof(str.c_str());
        std::getline(ss, str, separator); dataInfo.v = atof(str.c_str());
        std::getline(ss, str, separator); dataInfo.x = atof(str.c_str());
        std::getline(ss, str, separator); dataInfo.y = atof(str.c_str());
        std::getline(ss, str, separator); dataInfo.z = atof(str.c_str());

        points_calib.push_back(dataInfo);
    }
    
    return true;
}

cv::Mat getRelativeTransform(
    std::vector<cv::Point3d > objectPoints,
    std::vector<cv::Point2d > imagePoints,
    double fx, double fy, double cx, double cy)
{
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx,  0, cx,
                           0,  fy, cy,
                           0,   0,  1);
  cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients

  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, 1); //Tcl
  cv::Mat Rcl;
  cv::Rodrigues(rvec, Rcl);
  cv::Mat tcl = tvec;

  cv::Mat Tcl = cv::Mat::zeros(4,4,CV_64FC1);
  Tcl.at<double>(0,0) = Rcl.at<double>(0,0);
  Tcl.at<double>(0,1) = Rcl.at<double>(0,1);
  Tcl.at<double>(0,2) = Rcl.at<double>(0,2);
  Tcl.at<double>(1,0) = Rcl.at<double>(1,0);
  Tcl.at<double>(1,1) = Rcl.at<double>(1,1);
  Tcl.at<double>(1,2) = Rcl.at<double>(1,2);
  Tcl.at<double>(2,0) = Rcl.at<double>(2,0);
  Tcl.at<double>(2,1) = Rcl.at<double>(2,1);
  Tcl.at<double>(2,2) = Rcl.at<double>(2,2);

  Tcl.at<double>(0,3) = tcl.at<double>(0);
  Tcl.at<double>(1,3) = tcl.at<double>(1);
  Tcl.at<double>(2,3) = tcl.at<double>(2);

  Tcl.at<double>(3,3) = 1.0;

  return Tcl;
}

int main(int argc, char *argv[])
{
    std::string config_file = "../config/camera1_calibration.yaml";
    std::string data = "../data/cam1_lidar1.txt";
    if (argc < 3)
    {
        std::cout << "input arg number not enough! using default path" << std::endl;
    }
    else
    {
        config_file = argv[1];
        data = argv[2];
    }
    ReadIntrinsicsProcess(config_file);
    std::vector<CalibData> points_calib;
    bool flag = ReadCalibData(data, points_calib);
    if(!flag)
        return -1;

    std::vector<cv::Point3d > objectPoints;
    std::vector<cv::Point2d > imagePoints;
    for (int i = 0; i < points_calib.size(); i++ )
    {
        cv::Point2d im_point;
        cv::Point3d world_point;
        imagePoints.push_back(cv::Point2d(points_calib[i].u, points_calib[i].v));
        objectPoints.push_back(cv::Point3d(points_calib[i].x, points_calib[i].y, points_calib[i].z));
//        std:: cout << std::setprecision(12) << points_calib[i].u << std::endl;
    }

    cv::Mat Tcl =  getRelativeTransform(objectPoints, imagePoints,2000.0, 2000.0, 2048.0, 1080.0);
//    cv::Mat Tcl =  getRelativeTransform(objectPoints, imagePoints, project_mat.at<double>(0,0), project_mat.at<double>(1,1),
//                                        project_mat.at<double>(0,2), project_mat.at<double>(1,2));
    std::cout <<" Tcl: \n"<< Tcl << std::endl;

    return 0;

}

