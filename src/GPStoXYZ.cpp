#include <iostream>
#include <exception>
#include <cmath>
#include <fstream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <stdio.h>
#include <iomanip>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>


struct CalibData
{
    double r;
    double theta;
    double lat;     //纬度
    double lon;     //经度
    double height;  //高度
    double x;
    double y;
    double z;
};

bool ReadConfigfile(const std::string& config_fileName,std::string& calib_data_file,
                                            double& lat0, double& lon0, double& height0)
{
  cv::FileStorage config_read(config_fileName, cv::FileStorage::READ);
  if(!config_read.isOpened())
  {
    std::cerr<<"Read the calib config file failed！"<<std::endl;
    return false;
  }

  config_read["lat0"] >> lat0;
  config_read["lon0"] >> lon0;
  config_read["height0"] >> height0;
  config_read["calib_data_file"] >> calib_data_file;

  std::cout<<lat0<<" "<<lon0<<" "<<height0<<std::endl;
  std::cout<<"calib_data_file::"<<calib_data_file<<std::endl;

  return true;
}

void readCalibData(std::string fileName, std::vector<CalibData>& calib_datas, char separator = ',')
{
  std::fstream file_data(fileName);
  std::string line_str;
  while(std::getline(file_data, line_str))
  {
    CalibData data_read;
    std::stringstream ss(line_str);
    std::string factor_str;
    std::getline(ss, factor_str, separator);
    data_read.r = std::stod(factor_str);
    std::getline(ss, factor_str, separator);
    data_read.theta = std::stod(factor_str);

    std::getline(ss, factor_str, separator);
    data_read.lat = std::stod(factor_str);
    std::cout<<std::fixed<<std::setprecision(8);
    std::cout<<"lat::::"<<data_read.lat<<std::endl;
    std::getline(ss, factor_str, separator);
    data_read.lon = std::stod(factor_str);
    std::getline(ss, factor_str, separator);
    data_read.height = std::stod(factor_str);

    calib_datas.emplace_back(data_read);
  }

}


struct RadarGPSFactorYTH
{
  RadarGPSFactorYTH(CalibData data_calib) 
      : data_calib_(data_calib)
  {
      std::cout<<"data_calib:::::"<<data_calib.x<<"    "<<data_calib.y<<"  "
                            <<data_calib.z<<"   "<<data_calib.r<<"   "<<data_calib.theta<<std::endl;

  }
  template <typename T>
  bool operator()(const T *params, T *residual) const
  {

    // T x_w = T(data_calib_.x);//- data_calib_.v_x*params[1];
    // T y_w = T(data_calib_.y);//- data_calib_.v_y*params[1];
    // T x_project = x_w*cos(params[0]) - y_w*sin(params[0]);
    // T y_project = x_w*sin(params[0]) + y_w*cos(params[0]);

    // T theta = T(data_calib_.theta) / T(180.0) * T(3.1415926);
    // // theta = asin(sin(theta) * params[1]);
    // T x_radar = data_calib_.r * sin(theta);
    // // T y_radar = sqrt(pow(data_calib_.r * cos(theta), 2) - 4.0*4.0);    //法一
    // T y_radar = data_calib_.r * cos(theta);                                                       //法二

    // residual[0] = x_radar - x_project;
    // residual[1] = y_radar - y_project;

    //法三
    T range = T(data_calib_.r);
    T angle = T (data_calib_.theta) / 180.0 * 3.1415926;
    T d = sqrt(range*range - 4.0*4.0);
    T x_radar = d*sin(angle);
    T y_radar = d*cos(angle);
    T x_radar_w = x_radar * cos(params[0]) - y_radar * sin(params[0]);
    T y_radar_w = x_radar * sin(params[0]) + y_radar * cos(params[0]);
    T x = T(data_calib_.x);
    T y = T(data_calib_.y);
    residual[0] = x_radar_w - x;
    residual[1] = y_radar_w - y;

    return true;
  }

  static ceres::CostFunction *Create(const CalibData data_calib)
  {
    return (new ceres::AutoDiffCostFunction<RadarGPSFactorYTH, 2, 1>(new RadarGPSFactorYTH(data_calib)));
  }

  CalibData data_calib_;

};

void calibOptimizeRadar(std::vector<CalibData> points_calib)
{
  double radar_params[1] = {0.0};
  
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem::Options problem_options;

  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(radar_params, 1);

  std::cout << "value after optimizationA:"<<std::endl;
  for (int i = 0; i < points_calib.size(); ++i)
  {
    ceres::CostFunction *cost_function = RadarGPSFactorYTH::Create(points_calib[i]);
    problem.AddResidualBlock(cost_function, loss_function, radar_params);
  }
  std::cout << "value after optimizationB:"<<std::endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << "value after optimization:"<<std::endl;
  // std::cout << "params theta " << params[0] <<"  x " << params[1]  << "  y" << params[2]  <<"  z " << params[3]  << "  delt_t " << params[4]  <<std::endl;
  std::cout << "params angle:: " << radar_params[0] / 3.14159 * 180.0<<std::endl;
  std::cout<<"params theta::"<<radar_params[0]<<std::endl;
  // ofstream data_save(result_file);
  // data_save << params[0] << "," << params_yth[1]<<","<<params_yth[2];
}


struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x, double y, double range, double theta):x_(x), y_(y), range_(range), theta_(theta)
  {
    // double alpha = - 2.0 / 180.0 * M_PI;
    // x_ = cos(alpha) * x - sin(alpha) * y;
    // y_ = sin(alpha) * x + cos(alpha) * y;
     x_ = x;
     y_ = y;
    range_ = range;
    theta_ = theta / 180 * M_PI;

  }
  template <typename T>
  bool operator()(const T* const para,T* residual)const        //para[2]={H-h, theta_offset}
  {
    // residual[0] = T(x_) - T(range_) * T(cos(T(theta_)) + para[2]) * para[1] / para[0];
    // residual[0] = T(y_) - sqrt(pow(T(range_ * cos(theta_ + para[1])), 2) - pow((para[0]), 2));
    //residual[0] = T(y_) - T(range_ * cos(theta_ + para[0]));


    T x_project = x_*cos(para[0]) - y_*sin(para[0]);
    T y_project = x_*sin(para[0]) + y_*cos(para[0]);
    residual[0] = T(y_project) - sqrt(T(pow(range_ * cos(theta_), 2) ) - pow(T(6.0), 2)); //- T(1.0);
    residual[1] = T(x_project) - T(range_) * T(sin(T(theta_)));
    return true;
  }
  double x_, y_, range_, theta_;
};

void optimize(std::vector<CalibData> calibDataVec)
{
  double para[1] = {1.3};  // 初始化para[2] = {H-h, theta_offset};
  ceres::Problem problem;
  // for(int i=0;i<calibDataVec.size(); i++)
  for(int i =0 ; i < calibDataVec.size() ; i ++)
  {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,2,1>(
        new CURVE_FITTING_COST(calibDataVec[i].x, calibDataVec[i].y, calibDataVec[i].r, calibDataVec[i].theta)
      ),
      nullptr,
      para
    );
   
  }
  std::cout<<"333333"<<std::endl;
//配置求解器并求解，输出结果
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  std::cout<<"theta= "<<para[0] / M_PI * 180.0<<std::endl;
}

int main(int argc, char *argv[])
{
  std::string config_file = "../data/config_calib.yaml";
  std::string data_path;  //calib_data_file
  double lat0=0, lon0=0, h0=0;
  ReadConfigfile(config_file, data_path, lat0, lon0, h0);

  const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian gps_trans(lat0, lon0, h0, earth); //以原点构建局部坐标系(东北天)

  std::vector<CalibData> calib_datas;
  readCalibData(data_path, calib_datas);
  for(int i = 0; i < calib_datas.size(); i++)
  {
    double lat = calib_datas[i].lat;
    double lon = calib_datas[i].lon;
    double height = calib_datas[i].height;
    double x=0, y=0, z=0;
    gps_trans.Forward(lat, lon, height, x, y, z);
    calib_datas[i].x = x;
    calib_datas[i].y = y;
    calib_datas[i].z = z;
    std::cout<<"x:"<<x<<"      y:"<<y<<"            z"<<z<<std::endl;
  }
  calibOptimizeRadar(calib_datas);
  // optimize(calib_datas);
  return 0;

}


