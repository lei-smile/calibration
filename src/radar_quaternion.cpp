#include <iostream>
#include <exception>
#include <cmath>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <stdio.h>
#include <iomanip>
#include <cstdlib>

using namespace std;
using namespace cv;

// double para_q[4] = {0.707107, 0.0, 0.0, 0.707107};// x y z w
// double para_t[3] = {0.5, -0.1, 1.15};// x y z
double para_q[4] = {0.0, 0.0, 0.0, 1.0};// x y z w
double para_t[3] = {0.0, 0.0, 0.0};// x y z

Eigen::Map<Eigen::Quaterniond> Q_radar_cam(para_q);//x y z w
Eigen::Map<Eigen::Vector3d> T_radar_cam(para_t);

struct CalibData
{
  double x_radar;
  double y_radar;
  double x_lidar;
  double y_lidar;
  double z_lidar;
};

struct RadarCamFactor
{
  RadarCamFactor(Eigen::Vector3d p_cam00, double x_radar, double y_radar) 
      : p_cam_(p_cam00), x_radar_(x_radar), y_radar_(y_radar) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Quaternion<T> q_radar_cam{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_radar_cam{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> p_cam{T(p_cam_.x()), T(p_cam_.y()), T(p_cam_.z())};

    Eigen::Matrix<T, 3, 1> p_radar = q_radar_cam*p_cam + t_radar_cam;

    T r = sqrt(p_radar.x()*p_radar.x() + p_radar.y()*p_radar.y() + p_radar.z()*p_radar.z());

    T d = sqrt(p_radar.x()*p_radar.x() + p_radar.y()*p_radar.y());

    residual[0] =  x_radar_ - p_radar.x()*r/d;
    residual[1] =  y_radar_ - p_radar.y()*r/d;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d p_cam, double x_radar, double y_radar)
  {
    return (new ceres::AutoDiffCostFunction<RadarCamFactor, 2, 4, 3>(new RadarCamFactor(p_cam, x_radar, y_radar)));
  }

  Eigen::Vector3d p_cam_;
  double x_radar_;
  double y_radar_;
};

bool ReadCalibData(std::string fileName, std::vector<CalibData>& points_calib, char separator = ',')
{
  points_calib.clear();
  std::ifstream ifile(fileName);
  if (!ifile.is_open())
  {
    return false;
  }
  std::string lineStr;

  while (std::getline(ifile, lineStr))
  {
    std::stringstream ss(lineStr);
    std::string str;
    CalibData p_cam;
    std::getline(ss, str, separator);

    p_cam.x_lidar = atof(str.c_str());
    std::getline(ss, str, separator);
    p_cam.y_lidar = atof(str.c_str());
    std::getline(ss, str, separator);
    p_cam.z_lidar = atof(str.c_str());
    
    // std::cout << "p_lidar.x " << p_cam.x_lidar << "  p_lidar.y " << p_cam.y_lidar << "  p_lidar.z " << p_cam.z_lidar << std::endl;
    points_calib.push_back(p_cam);

    }
    
    return true;
}


int main(int argc, char *argv[])
{
  std::string data_path = "../data/radar_calib_files/";
  if (argc < 2)
  {
    std::cout << "input arg number not enough! using default path" << std::endl;
  }
  else
  {
    data_path = argv[1];
  }

  Eigen::Matrix<double, 3, 3> Rrl;
  Rrl << 0.,-1.,0.,
         1.,0.,0.,
         0.,0.,1.;
  Eigen::Matrix<double, 3, 1> trl;
  trl << 0.5,-0.1,1.15;
 
  std::cout << Rrl << std::endl;
  Eigen::Quaterniond qrl(Rrl);
 
  std::cout << "Q lidar to radar w " << qrl.w() <<"  x " << qrl.x() << "  y " << qrl.y() <<"  z " << qrl.z() << std::endl;
  std::cout << "T lidar to radar " << "x " << trl[0] << "  y " << trl[1] <<"  z " << trl[2] << std::endl;
  std::cout << "initial value before optimization:"<<std::endl;
  std::cout << "Q_radar_lidar w " << para_q[3] <<"  x " << para_q[0]  << "  y " << para_q[1]  <<"  z " << para_q[2]  << std::endl;
  std::cout << "T_radar_lidar x " << para_t[0]  << "  y " << para_t[1]  <<"  z " << para_t[2]  << std::endl;
  std::vector<CalibData> points_calib;
  std::string data_file = data_path + "sim_data.txt";
  bool flag = ReadCalibData(data_file, points_calib);

  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
  ceres::Problem::Options problem_options;

  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(para_q, 4, q_parameterization);
  problem.AddParameterBlock(para_t, 3);

  for (int i = 0; i < points_calib.size(); ++i)
  {
    Eigen::Vector3d p_lidar(points_calib[i].x_lidar, points_calib[i].y_lidar, points_calib[i].z_lidar);

    Eigen::Vector3d p_radar = Rrl*p_lidar + trl;
    double r = sqrt(pow(p_radar[0], 2) + pow(p_radar[1], 2) + pow(p_radar[2],2));
    double theta = atan2(p_radar[1], p_radar[0]);
    
    int number = rand() % 5 + 1; 
    double delt_r = ((double)number)/10.0 -0.25; // -0.25m ~ 0.25m
    // delt_r = 0;
    double x_radar = (r + delt_r)*cos(theta);
    double y_radar = (r + delt_r)*sin(theta);

    ceres::CostFunction *cost_function = RadarCamFactor::Create(p_lidar, x_radar, y_radar);
    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << "value after optimization:"<<std::endl;
  std::cout << "Q_radar_lidar w " << para_q[3] <<"  x " << para_q[0]  << "  y " << para_q[1]  <<"  z " << para_q[2]  << std::endl;
  std::cout << "T_radar_lidar x " << para_t[0]  << "  y " << para_t[1]  <<"  z " << para_t[2]  << std::endl;
 
  return 0;

}
