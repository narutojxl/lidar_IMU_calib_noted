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
#include <core/lidar_odometry.h>
#include <utils/pcl_utils.h>
#include <utils/math_utils.h>

namespace licalib {

LiDAROdometry::LiDAROdometry(double ndt_resolution)
        : map_cloud_(new VPointCloud()) {
  ndt_omp_ = ndtInit(ndt_resolution);
}

pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
LiDAROdometry::ndtInit(double ndt_resolution) {
  auto ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr(
          new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
  ndt_omp->setResolution(ndt_resolution);
  ndt_omp->setNumThreads(4);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setTransformationEpsilon(1e-3);
  ndt_omp->setStepSize(0.01);
  ndt_omp->setMaximumIterations(50);
  return ndt_omp;
}

void LiDAROdometry::feedScan(double timestamp,
                             VPointCloud::Ptr cur_scan,
                             Eigen::Matrix4d pose_predict, //default: I
                             const bool update_map) { //default: true
  OdomData odom_cur;
  odom_cur.timestamp = timestamp;
  odom_cur.pose = Eigen::Matrix4d::Identity();

  VPointCloud::Ptr scan_in_target(new VPointCloud());
  if (map_cloud_->empty()) {//第一帧在updateKeyScan()中赋值后，就变为not empty
    scan_in_target = cur_scan;
  } else {
    Eigen::Matrix4d T_LtoM_predict = odom_data_.back().pose * pose_predict;
    registration(cur_scan, T_LtoM_predict, odom_cur.pose, scan_in_target); //map[first scan, curr scan)累积起来
                                                                           //计算map和curr scan的ndt匹配
  }
  odom_data_.push_back(odom_cur); //把map--->curr TF保存到odom_data_

  if (update_map) { 
    updateKeyScan(cur_scan, odom_cur); //构建map
  }
}

void LiDAROdometry::registration(const VPointCloud::Ptr& cur_scan,
                                 const Eigen::Matrix4d& pose_predict,
                                 Eigen::Matrix4d& pose_out,
                                 VPointCloud::Ptr scan_in_target) {
  VPointCloud::Ptr p_filtered_cloud(new VPointCloud());
  downsampleCloud(cur_scan, p_filtered_cloud, 0.5);

  ndt_omp_->setInputSource(p_filtered_cloud);
  ndt_omp_->align(*scan_in_target, pose_predict.cast<float>());

  pose_out = ndt_omp_->getFinalTransformation().cast<double>();
}

void LiDAROdometry::updateKeyScan(const VPointCloud::Ptr& cur_scan,
                                  const OdomData& odom_data) {
  if (checkKeyScan(odom_data)) {//当前帧与上一关键帧的距离或角度超过一定阈值，当前帧为新的关键帧，才对地图做更新

    VPointCloud::Ptr filtered_cloud(new VPointCloud());
    downsampleCloud(cur_scan, filtered_cloud, 0.1); //leaf size

    VPointCloud::Ptr scan_in_target(new VPointCloud());
    pcl::transformPointCloud(*filtered_cloud, *scan_in_target, odom_data.pose); //把当前帧转化到map下

    *map_cloud_ += *scan_in_target;
    ndt_omp_->setInputTarget(map_cloud_); //注意：ndt_omp的target点云是整个map
    key_frame_index_.push_back(odom_data_.size()); //关键帧的index从1开始
  }
}


bool LiDAROdometry::checkKeyScan(const OdomData& odom_data) {
  static Eigen::Vector3d position_last(0,0,0); //静态的局部变量
  static Eigen::Vector3d ypr_last(0,0,0);

  Eigen::Vector3d position_now = odom_data.pose.block<3,1>(0,3);
  double dist = (position_now - position_last).norm();

  const Eigen::Matrix3d rotation (odom_data.pose.block<3,3> (0,0));
  Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
  Eigen::Vector3d delta_angle = ypr - ypr_last;
  for (size_t i = 0; i < 3; i++)
    delta_angle(i) = normalize_angle(delta_angle(i));
  delta_angle = delta_angle.cwiseAbs();

  if (key_frame_index_.size() == 0 || dist > 0.1 //key_frame_index_.size() == 0: 最开始的第一帧也是关键帧  //default: 0.2m
     || delta_angle(0) > 3.0 || delta_angle(1) > 3.0 || delta_angle(2) > 3.0) { //default: 5.0度
    position_last = position_now;
    ypr_last = ypr;
    return true;
  }
  return false;
}


void LiDAROdometry::setTargetMap(VPointCloud::Ptr map_cloud_in) {
  map_cloud_->clear();
  pcl::copyPointCloud(*map_cloud_in, *map_cloud_);
  ndt_omp_->setInputTarget(map_cloud_);
}

void LiDAROdometry::clearOdomData() {
  key_frame_index_.clear();
  odom_data_.clear();
}


}


