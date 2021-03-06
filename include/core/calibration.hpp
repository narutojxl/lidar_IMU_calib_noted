/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Eigen>
#include <memory>
#include <fstream>

namespace licalib {

class CalibParamManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<CalibParamManager> Ptr;

  CalibParamManager() :
          p_LinI(Eigen::Vector3d(0,0,0)) ,
          q_LtoI(Eigen::Quaterniond::Identity()),
          gravity(Eigen::Vector3d(0, 0, -9.8)),
          time_offset(0),
          gyro_bias(Eigen::Vector3d(0,0,0)),
          acce_bias(Eigen::Vector3d(0,0,0)) {

    double gyroscope_noise_density = 1.745e-4;
    double accelerometer_noise_density = 5.88e-4;
    double imu_rate = 400.0;
    double lidar_noise = 0.02;

    double gyro_discrete = gyroscope_noise_density * std::sqrt(imu_rate);
    double acce_discrete = accelerometer_noise_density * std::sqrt(imu_rate);

    global_opt_gyro_weight = 1.0 / std::pow(gyro_discrete, 2);  // 8.21e4
    global_opt_acce_weight = 1.0 / std::pow(acce_discrete, 2);  // 7.23e3
    global_opt_lidar_weight = 1.0 / std::pow(lidar_noise, 2);   // 2.5e3

    // fine-tuned parameter
    global_opt_gyro_weight = 28.0; //TODO: https://github.com/APRIL-ZJU/lidar_IMU_calib/issues/2
    global_opt_acce_weight = 18.5;
    global_opt_lidar_weight = 10.0; 
  }

  void set_q_LtoI(Eigen::Quaterniond q) {
    q_LtoI = q;
  }

  void set_p_LinI(Eigen::Vector3d p) {
    p_LinI = p;
  }

  void set_gravity(Eigen::Vector3d g) {
    gravity = g;
  }

  void set_time_offset(double t) {
    time_offset = t;
  }

  void set_gyro_bias(Eigen::Vector3d gb) {
    gyro_bias = gb;
  }

  void set_acce_bias(Eigen::Vector3d ab) {
    acce_bias = ab;
  }

  void showStates() const {
    Eigen::Vector3d euler_LtoI = q_LtoI.toRotationMatrix().eulerAngles(0,1,2); //TODO：作者的欧拉角是0,1,2，不是2,1,0
    // Eigen::Vector3d euler_LtoI = q_LtoI.toRotationMatrix().eulerAngles(2,1,0); //jxl
    euler_LtoI = euler_LtoI * 180 / M_PI;

    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();
    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);
    Eigen::Vector3d euler_ItoL = q_ItoL.toRotationMatrix().eulerAngles(0,1,2); //同上
    // Eigen::Vector3d euler_ItoL = q_ItoL.toRotationMatrix().eulerAngles(2,1,0);
    euler_ItoL = euler_ItoL * 180 / M_PI;

    std::cout << "P_LinI      : " << p_LinI.transpose() << std::endl;
    std::cout << "euler_LtoI  : " << euler_LtoI.transpose() << std::endl;
    std::cout << "P_IinL      : " << p_IinL.transpose() << std::endl;
    std::cout << "euler_ItoL  : " << euler_ItoL.transpose() << std::endl;
    std::cout << "time offset : " << time_offset << std::endl;
    std::cout << "gravity     : " << gravity.transpose() << std::endl;
    std::cout << "acce bias   : " << acce_bias.transpose() << std::endl;
    std::cout << "gyro bias   : " << gyro_bias.transpose() << std::endl;
  }

  void save_result(const std::string& filename, const std::string& info) const {
    Eigen::Quaterniond q_ItoL = q_LtoI.inverse();
    Eigen::Vector3d p_IinL = q_ItoL * (-p_LinI);

    std::ofstream outfile;
    outfile.open(filename, std::ios::app);
    outfile << info << ","
            << p_IinL(0) << "," << p_IinL(1) << "," << p_IinL(2) << ","
            << q_ItoL.x() << "," << q_ItoL.y() << "," << q_ItoL.z() << "," << q_ItoL.w() << ","
            << time_offset << "," << gravity(0) << "," << gravity(1) << "," << gravity(2) << ","
            << gyro_bias(0) << "," << gyro_bias(1) << "," <<gyro_bias(2) << ","
            << acce_bias(0) << "," << acce_bias(1) << "," <<acce_bias(2) << "\n";
    outfile.close();
  }

public:
  Eigen::Vector3d p_LinI; //laser在imu下的坐标

  Eigen::Quaterniond q_LtoI; //计算的是imu--->laser

  Eigen::Vector3d gravity; //重力在L0下的向量

  double time_offset; //laser和imu的时间offset

  Eigen::Vector3d gyro_bias;

  Eigen::Vector3d acce_bias;

  ///weight
  double global_opt_gyro_weight; //initial, batch, refine阶段都会用到

  double global_opt_acce_weight;

  double global_opt_lidar_weight;
};

}

#endif
