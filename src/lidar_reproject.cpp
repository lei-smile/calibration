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

bool lidar2Pixel(const std::vector<cv::Point3f>& incloud, cv::Mat lidar2Camera, cv::Mat cameraInnerParam, cv::Mat distCoffes, std::vector<cv::Point2f>& pointsOut, std::vector<float>& distList)
{
    
    std::vector<cv::Point> drawPoints;
    for (int i = 0; i < incloud.size(); i++)
    {
        cv::Point3f pointCamera, pointLidar = incloud[i];
        pointCamera.x = pointLidar.x * lidar2Camera.at<double>(0, 0) + pointLidar.y * lidar2Camera.at<double>(0, 1) + pointLidar.z * lidar2Camera.at<double>(0, 2) + lidar2Camera.at<double>(0, 3);
        pointCamera.y = pointLidar.x * lidar2Camera.at<double>(1, 0) + pointLidar.y * lidar2Camera.at<double>(1, 1) + pointLidar.z * lidar2Camera.at<double>(1, 2) + lidar2Camera.at<double>(1, 3);
        pointCamera.z = pointLidar.x * lidar2Camera.at<double>(2, 0) + pointLidar.y * lidar2Camera.at<double>(2, 1) + pointLidar.z * lidar2Camera.at<double>(2, 2) + lidar2Camera.at<double>(2, 3);
        std::cout << pointCamera.x << " "<< pointCamera.y <<" "<< pointCamera.z<< std::endl;
        if (pointCamera.z < 0)
        {
            pointsOut.push_back(cv::Vec2f(0,0));
            distList.push_back(0);
            continue;
        }
        double xhomo = pointCamera.x / pointCamera.z;
        double yhomo = pointCamera.y / pointCamera.z;
        double r = sqrt(xhomo * xhomo + yhomo * yhomo);
        double theta = atan(r);
  //      std::cout << cameraInnerParam.at<double>(1, 0)<<" " << std::endl;
        double theta_d = theta * (1 + distCoffes.at<double>(0, 0) * theta * theta
                                + distCoffes.at<double>(1, 0) * theta * theta * theta * theta
                                + distCoffes.at<double>(2, 0) * theta * theta * theta * theta * theta * theta
                                + distCoffes.at<double>(3, 0) * theta * theta * theta * theta * theta * theta * theta * theta);
//std::cout << "@@@" <<  std::endl;

        double x = (theta_d / r) * xhomo;
        double y = (theta_d / r) * yhomo;

        double u = cameraInnerParam.at<double>(0, 0) * x + cameraInnerParam.at<double>(2, 0);
        double v = cameraInnerParam.at<double>(1, 0) * y + cameraInnerParam.at<double>(3, 0);
        pointsOut.push_back(cv::Vec2f(u, v));
        distList.push_back(pointCamera.z);
    }
    
    return true;
    
}

bool readLidarPoints(std::string lidar_file, std::vector<cv::Point3f>& lidar_points, char separator = ' ')
{
    std::fstream fin(lidar_file);
    if(!fin.is_open())
    {
        std::cout<<"fail to read lidar points"<<std::endl;
        return false;
    }
    std::string line_str;
    while(std::getline(fin, line_str))
    {
        std::stringstream ss_line(line_str);
        std::string temp_str;
        double x,y,z;
        std::getline(ss_line, temp_str,separator); x = std::atof(temp_str.c_str()); 
        std::getline(ss_line, temp_str, separator);y = std::atof(temp_str.c_str());
        std::getline(ss_line, temp_str, separator); z =std::atof(temp_str.c_str());


        lidar_points.push_back(cv::Point3f(x, y, z)); 

    }
    return true;

}

int main(int argc, char *argv[])
{

    std::string config_file = "../config/camera1_calibration.yaml";
    std::vector<cv::Point3f> lidar_points;
    std::string lidar_file = "../008/4/lidar/lidar_1_1577839483232118999.txt";
    std::string pic_file = "../008/4/pic/1_1577839476531654000.jpg";

    cv::Mat im = cv::imread(pic_file);


    bool read_lidar = readLidarPoints(lidar_file, lidar_points);
    if(!read_lidar)
    {
        std::cout<<"fail to read lidar file"<<std::endl;
        return -1;
    }
    printf("lidar point size: %d\n", lidar_points.size());
    std::cout<<"x: "<<lidar_points[0].x <<"y: "<<lidar_points[0].y<<std::endl;

 
    cv::Mat lidar2Camera = (cv::Mat_<double>(4,4) << 
    -0.007190032217678222, -0.9999285050857509, -0.009554483433674876, 0.07976314358982979,
 -0.02992454185725696, 0.009765605364441488, -0.9995044545906233, 0.04949983713957493,
 0.999526300420025, -0.006900555690784016, -0.02999261742189829, -0.2516728237413449,
 0, 0, 0, 1);



    cv::Mat cameraInnerParam = (cv::Mat_<double>(4,1) <<2960.02509457, 2958.03391451, 938.12615099, 581.79624112);

    cv::Mat distCoffes = (cv::Mat_<double>(4,1) << -0.24262844, -0.25324402, -2.38648702, 17.54088803);
    std::cout<<"dist::"<<distCoffes;
    // cv::Mat distCoffes = cv::Mat(4,1,CV_32FC1, mat_distCoffes);

    std::vector<cv::Point2f> lidar_pixels;
    std::vector<float> dis_vec;
    lidar2Pixel(lidar_points, lidar2Camera, cameraInnerParam, distCoffes,lidar_pixels, dis_vec);

    cv::Mat dis_img(1, dis_vec.size(),CV_8U, cv::Scalar(0));
    double scale = 3;
    // cv::Mat color_mat;
    for(int i =0 ; i < dis_vec.size(); i++)
    {
        if (dis_vec[i] * scale > 255)
        {
            dis_img.at<uchar>(0, i) = 255;
            continue;
        }
        if (dis_vec[i] * scale < 0)
        {
            dis_img.at<uchar>(0, i) = 0;
            continue;
        }
        dis_img.at<uchar>(0, i) = dis_vec[i]* scale;

    }
    cv::Mat colorMap;
    cv::applyColorMap(dis_img,colorMap, COLORMAP_HSV);
    // imshow("color", color_mat);
    // double max_dis = 0;
    // double min_dis = 1000;
    // for(int i =0 ; i < dis_vec.size(); i++)
    // {
    //     if(max_dis < dis_vec[i])
    //         max_dis = dis_vec[i];

    //     if(min_dis > dis_vec[i])
    //         min_dis = dis_vec[i];


    // }
    int u,v;
    for(int i = 0 ; i < lidar_pixels.size(); i++)
    {
        u = lidar_pixels[i].x;
        v = lidar_pixels[i].y;

            cv::Point center(u, v);
            cv::Vec3b _color = colorMap.at<cv::Vec3b>(0, i);
            cv::circle(im, center, 1, _color, -1);

//        std::cout << lidar_pixels[i]<<std::endl;
//        double kkey = 255*10/(max_dis-min_dis)*dis_vec[i]; 
        // cv::Scalar color_m = color_mat.at<double>(i, 0);
//        cv::circle(im, lidar_pixels[i], 2,cv::Scalar(0,0,kkey), -1);
        // cv::circle(im, lidar_pixels[i], 2,color_m, -1);
    }
    cv::pyrDown(im, im);
    cv::namedWindow("im", 0);
    imshow("im", im);
    waitKey();

    
    return 0;

}

