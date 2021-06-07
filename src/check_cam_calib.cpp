#include <fstream>
#include <opencv2/opencv.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


struct CamCalibData
{
    double u;
    double v;
    double x;
    double y;
    double z;
};
using namespace cv;

bool ReadConfigfile(std::string config_fileName, double& lat0, double& lon0, double& height0,
                                        std::string& calib_path, std::string& cam_file, std::string& image_file,
                                        std::string& result_file, std::string& cam_reproject_file)
{
  cv::FileStorage config_read(config_fileName, cv::FileStorage::READ);
  if(!config_read.isOpened())
  {
    std::cout<<"Read the calib config file failed！"<<std::endl;
    return false;
  }
  config_read["calib_path"] >> calib_path;
  config_read["cam_file"] >> cam_file;
  cam_file = calib_path + cam_file;
  config_read["image_file"] >> image_file;
  image_file = calib_path + image_file;
  config_read["result_file"] >> result_file;
  result_file = calib_path + result_file;
  config_read["cam_reproject_file"] >> cam_reproject_file;
  cam_reproject_file = calib_path + cam_reproject_file;
  config_read["lat0"] >> lat0;
  config_read["lon0"] >> lon0;
  config_read["height0"] >> height0;

  config_read.release();
  return true;
}

bool ReadCalibfile(std::string cam_fileName, std::vector<CamCalibData>& points_calib,
                                       GeographicLib::LocalCartesian& gps_trans)
{
  points_calib.clear();

  std::ifstream cam_ifile(cam_fileName);
  if (!cam_ifile.is_open())
  {
    std::cout<<"Read the cam calib data failed！"<<std::endl;
    return false;
  }
  std::string cam_lineStr;
  while (std::getline(cam_ifile, cam_lineStr))
  {
    CamCalibData dataInfo;
    std::stringstream cam_ss(cam_lineStr);
    std::string cam_str;
    double lat, lon, height;
    std::getline(cam_ss, cam_str, ','); dataInfo.u = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); dataInfo.v = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); lat = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); lon = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); height = atof(cam_str.c_str());

    gps_trans.Forward(lat, lon, height, dataInfo.x, dataInfo.y, dataInfo.z);
    std::cout<<dataInfo.u<<" "<<dataInfo.v<<" "<<dataInfo.x
             <<" "<<dataInfo.y<<" "<<dataInfo.z<<std::endl;
    points_calib.push_back(dataInfo);
  }
  cam_ifile.close();
  return true;
}

std::vector<cv::Mat> GetUtm2Cam(std::vector<CamCalibData>& points_calib)
{
  std::vector<cv::Point2f> objPoints;
  std::vector<cv::Point2f> imgPoints;
  std::vector<cv::Mat> umt_cam;
  imgPoints.clear();
  objPoints.clear();
  // int Id_a = int(candidate_matrix.at<double>(0,0))-1;
  // int Id_b = int(candidate_matrix.at<double>(1,0))-1;
  // int Id_c = int(candidate_matrix.at<double>(2,0))-1;
  // int Id_d = int(candidate_matrix.at<double>(3,0))-1;

  // imgPoints.push_back(cv::Point2f(points_calib[Id_a].u,points_calib[Id_a].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_b].u,points_calib[Id_b].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_c].u,points_calib[Id_c].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_d].u,points_calib[Id_d].v));
  // objPoints.push_back(cv::Point2f(points_calib[Id_a].x,points_calib[Id_a].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_b].x,points_calib[Id_b].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_c].x,points_calib[Id_c].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_d].x,points_calib[Id_d].y));

  for(int i = 0; i < points_calib.size(); i++ )
  {
    imgPoints.emplace_back(cv::Point2f(points_calib[i].u, points_calib[i].v));
    objPoints.emplace_back(cv::Point2f(points_calib[i].x, points_calib[i].y));
  }

  // cv::Mat cam2UTM = cv::getPerspectiveTransform(imgPoints, objPoints);
  cv::Mat cam2UTM = cv::findHomography(imgPoints, objPoints);
  umt_cam.push_back(cam2UTM);
  // cv::Mat UTM2cam = cv::getPerspectiveTransform(objPoints, imgPoints);
  cv::Mat UTM2cam = cv::findHomography(objPoints, imgPoints);
  umt_cam.push_back(UTM2cam);

  std::cout<<"cam2UTM"<<cam2UTM<<std::endl;
  std::cout<<"UTM2cam"<<UTM2cam<<std::endl;

  return umt_cam;
}

cv::Mat DrawUtm2Cam(std::vector<cv::Point2d> objectPoints,
                    std::vector<cv::Point2d> imagePoints, std::vector<CamCalibData> points_calib,
                    std::string image_file, std::string result_file)
{
   cv::FileStorage result_write(result_file, cv::FileStorage::WRITE);
   std::vector<cv::Mat> umt_cam = GetUtm2Cam(points_calib);
   cv::Mat image  = cv::imread(image_file);

   std::vector<cv::Point2f> utmPos, pixelPos;
   for (int i = 0; i < objectPoints.size(); i++)
   {
     utmPos.push_back(cv::Point2f(objectPoints[i].x, objectPoints[i].y));
   }
    
   cv::perspectiveTransform(utmPos, pixelPos, umt_cam[1]);

   double sum_error = 0;

   for (int i = 0; i < objectPoints.size(); ++i)
   {
      cv:Point2d p1,p2;

      p1.x = imagePoints[i].x;
      p1.y = imagePoints[i].y;
      circle(image, p1, 10, Scalar(0, 255, 0),4);
      //std::cout<<"************src:"+to_string(i) +"   ("<<p.x<<","<<p.y<<")"<<std::endl;

      p2.x = pixelPos[i].x;
      p2.y = pixelPos[i].y;
      circle(image, p2, 10, Scalar(0, 0, 255),4);
      //std::cout<<"************obj:"+to_string(i) +"   ("<<p.x<<","<<p.y<<")"<<std::endl;
      sum_error += std::hypot(std::abs(p1.x-p2.x),std::abs(p1.y-p2.y));
   }

   result_write << "cam2UTM" << umt_cam[0] << "UTM2cam" << umt_cam[1];
   result_write << "reprojection_error" << sum_error/objectPoints.size();
   //result_write.release();
   std::cout<<"sum_error:  "<<sum_error<<std::endl;
   std::cout<<"reprojection_error: "<<sum_error/objectPoints.size()<<std::endl;
   return image;
}

bool readGPSPoints(std::string gps_test_file, std::vector<cv::Point2f>& gps_test_points,
                                          GeographicLib::LocalCartesian& gps_trans )
{
    std::ifstream gps_ifile(gps_test_file);
    if (!gps_ifile.is_open())
    {
        std::cout<<"Read the gps data failed！"<<std::endl;
        return false;
    }
    std::string gps_lineStr;
    while (std::getline(gps_ifile, gps_lineStr))
    {
        cv::Point3f gps_points;
        std::stringstream gps_ss(gps_lineStr);
        std::string gps_str;
        double lat, lon, height;
        std::getline(gps_ss, gps_str, ',');
        std::getline(gps_ss, gps_str, ','); 
        std::getline(gps_ss, gps_str, ','); lat = atof(gps_str.c_str());
        std::getline(gps_ss, gps_str, ','); lon = atof(gps_str.c_str());
        std::getline(gps_ss, gps_str, ','); height = atof(gps_str.c_str());
        double x=0, y=0, z=0;
        gps_trans.Forward(lat, lon, height, x, y, z);
        //printf("%.7lf, %.7lf, %lf, %lf, %lf, %lf\n", lat, lon, height, x, y, z);
        gps_test_points.push_back(cv::Point2f(x, y));
    }
    return true;

}

bool readImgPoints(std::string img_points_file, std::vector<cv::Point2f>& image_points )
{
    std::ifstream pixels_ifile(img_points_file);
    if (!pixels_ifile.is_open())
    {
        std::cout<<"Read the img data failed！"<<std::endl;
        return false;
    }
    std::string pixel_lineStr;
    while (std::getline(pixels_ifile, pixel_lineStr))
    {
        cv::Point2f pixel_points;
        std::stringstream pixel_ss(pixel_lineStr);
        std::string pixel_str;
        double u, v;
        std::getline(pixel_ss, pixel_str, ','); u = atof(pixel_str.c_str());
        std::getline(pixel_ss, pixel_str, ','); v = atof(pixel_str.c_str());
        //printf("%lf, %lf\n", u,v);
        image_points.push_back(cv::Point2f(u, v));
    }
    return true;

}

int main(int argc, char *argv[])
{
    
    double lat0 = 39.08245244, lon0 = 117.0434205, height0 = -7.038; //83路口原点
    static const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian gps_trans(lat0, lon0, height0, earth);

    std::string gps_test_file = "../data/gps_test_points.csv";
    std::vector<cv::Point2f> gps_test_points;
    bool read_gps_flag = readGPSPoints(gps_test_file, gps_test_points, gps_trans);

    std::string config_file = "../data/tj_83_2/result.yaml";
    cv::FileStorage fs_config(config_file, cv::FileStorage::READ);
    cv::Mat cam2UTM_2, UTM2cam_2;
    fs_config["cam2UTM"] >> cam2UTM_2;
    fs_config["UTM2cam"] >>UTM2cam_2;
    std::vector<cv::Point2f> gps_reproject_2;
    cv::perspectiveTransform(gps_test_points, gps_reproject_2, UTM2cam_2);
    std::string img2_file = "../data/tj_83_2/img.jpg";
    cv::Mat img_2 = cv::imread(img2_file);

    std::vector<cv::Point2f> img_points_2;
    bool read_img_flag = readImgPoints("../data/tj_83_2/pixels.txt", img_points_2);
    std::vector<cv::Point2f> img_UTM_2;
    cv::perspectiveTransform(img_points_2, img_UTM_2, cam2UTM_2);

    double avg_error_reproject = 0;
    double avg_error_location = 0;
    std::string gps_xy_file = "../data/gps_xy_points.txt";
    std::fstream fout_gps_xy(gps_xy_file, std::fstream::out);

    std::string img2_xy_file = "../data/img2_xy_points.txt";
    std::fstream fout_img2_xy(img2_xy_file, std::fstream::out);

    for(int i = 0 ; i < gps_reproject_2.size(); i++)
    {
        fout_gps_xy << gps_test_points[i].x<<", "<<gps_test_points[i].y<<std::endl;
        fout_img2_xy <<img_UTM_2[i].x<<", "<<img_UTM_2[i].y<<std::endl;
        cv::circle(img_2, gps_reproject_2[i], 4, cv::Scalar(0, 0, 255), -1);
        cv::circle(img_2, img_points_2[i], 4, cv::Scalar(0, 255, 0), -1);
        double error_reproject =sqrt(pow(gps_reproject_2[i].x - img_points_2[i].x, 2)
                                                    +pow(gps_reproject_2[i].y - img_points_2[i].y, 2));
        // printf("error_reproject:%lf\n", error_reproject);
        avg_error_reproject += error_reproject;
        double error_location =  sqrt(pow(gps_test_points[i].x - img_UTM_2[i].x, 2)
                                                    +pow(gps_test_points[i].y - img_UTM_2[i].y, 2));
        printf("num:%d,error_location:%lf\n", i, error_location);
        avg_error_location += error_location;
    }
    avg_error_reproject /= double(gps_reproject_2.size());
    printf("avg_error_reproject:%lf\n", avg_error_reproject);

    avg_error_location /= double(gps_reproject_2.size());
    printf("avg_error_location:%lf\n", avg_error_location);
    cv::namedWindow("img_2",  0);
    cv::imshow("img_2", img_2);
    cv::imwrite("img_2.jpg", img_2);
    cv::waitKey();

    //83_1
    //avg_error_reproject:9.389377
    //avg_error_location:0.564288

    //83_2
    //avg_error_reproject:8.139997
    //avg_error_location:0.901798

    //83_3
    //avg_error_reproject:11.990462
    //avg_error_location:0.655557

    //83_4
    //avg_error_reproject:7.608024
    // avg_error_location:0.575700

    return 0;
}

// int main()
// {
//   double lat0 = 31.373718, lon0 = 120.4203869, height0 = 11.737;
//   static const Geocentric& earth = Geocentric::WGS84();
//   LocalCartesian gps_trans(lat0, lon0, height0, earth);
//   double lat = 31.37447572, lon = 120.4202786, height = 11.916;
//   double x = 0, y = 0, z= 0;
//   gps_trans.Forward(lat, lon, height, x, y, z);
//   std::cout<<"x:  "<<x<<"y:  "<<y<<"z:   "<<z<<std::endl;


// }






