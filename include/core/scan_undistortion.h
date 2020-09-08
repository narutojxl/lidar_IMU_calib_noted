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
#pragma once

#include <utils/dataset_reader.h>
#include <core/trajectory_manager.h>
#include <core/lidar_odometry.h>

namespace licalib {

class ScanUndistortion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ScanUndistortion> Ptr;

  explicit ScanUndistortion(TrajectoryManager::Ptr traj_manager,      //因为是值传递(调用的是copy ctor)，traj_manager指向实参指向的地址
                            std::shared_ptr<IO::LioDataset> dataset)  //dataset也指向实参指向的地址
                            : traj_manager_(std::move(traj_manager)), //traj_manager的指向变为nullptr，traj_manager_指向实参指向的地址
                              dataset_reader_(std::move(dataset)) {   //同上
                        //也就是说this->traj_manager_和CalibrHelper::traj_manager_指向相同
                        //this->dataset_reader_也是一样，和CalibrHelper::dataset_reader_指向相同
  }


  void undistortScan(bool correct_position = false) {
    scan_data_.clear();
    for (const TPointCloud& scan_raw: dataset_reader_->get_scan_data()) {
      Eigen::Quaterniond q_L0_to_G;//jxl: q_I0_Lk
      Eigen::Vector3d p_L0_in_G; //jxl: p_I0_Lk
      double scan_timestamp = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
      if (!traj_manager_->evaluateLidarPose(scan_timestamp, q_L0_to_G, p_L0_in_G)) {
        std::cout << "[ScanUndistortion] : pass " << scan_timestamp << std::endl;
        continue;
      }

      VPointCloud::Ptr scan_in_target(new VPointCloud);
      undistort(q_L0_to_G.conjugate(), p_L0_in_G, scan_raw,
                scan_in_target, correct_position); //把每帧laser下的points，去旋转畸变，转换到本帧laser时间戳时的laser frame下

      scan_data_.insert({scan_in_target->header.stamp, scan_in_target}); //保存到scan_data_中
    }
  }


  void undistortScanInMap(bool correct_position = true) {
    scan_data_in_map_.clear();
    map_cloud_ = VPointCloud::Ptr(new VPointCloud);
    Eigen::Quaterniond q_L0_to_G; //jxl: q_I0_L0, G: I0
    Eigen::Vector3d p_L0_in_G; //jxl: p_I0_L0
    double map_start_time = traj_manager_->get_map_time();
    traj_manager_->evaluateLidarPose(map_start_time, q_L0_to_G, p_L0_in_G); 

    for (const TPointCloud& scan_raw: dataset_reader_->get_scan_data()) {
      VPointCloud::Ptr scan_in_target(new VPointCloud);
      undistort(q_L0_to_G.conjugate(), //q_L0_I0
                p_L0_in_G, //p_I0_L0
                scan_raw,
                scan_in_target, correct_position); //scan_in_target: 转换到L0下，即map下的points

      scan_data_in_map_.insert({scan_in_target->header.stamp, scan_in_target});
      *map_cloud_ += *scan_in_target;
    }
  }


  void undistortScanInMap(const Eigen::aligned_vector<LiDAROdometry::OdomData>& odom_data) {
    scan_data_in_map_.clear();
    map_cloud_ = VPointCloud::Ptr(new VPointCloud);

    for (size_t idx = 0; idx < dataset_reader_->get_scan_data().size(); idx++){
      auto scan_raw = dataset_reader_->get_scan_data().at(idx);
      auto iter = scan_data_.find(scan_raw.header.stamp);
      if (iter == scan_data_.end()) {
        continue;
      }
      VPointCloud::Ptr scan_inMap = VPointCloud::Ptr(new VPointCloud);
      pcl::transformPointCloud(*(iter->second), *scan_inMap, odom_data.at(idx).pose);
      scan_data_in_map_.insert({scan_raw.header.stamp, scan_inMap});
      *map_cloud_ += *scan_inMap;
    }
  }

  const std::map<pcl::uint64_t, VPointCloud::Ptr> &get_scan_data() const {
    return scan_data_;
  }

  const std::map<pcl::uint64_t, VPointCloud::Ptr> &get_scan_data_in_map() const {
    return scan_data_in_map_;
  }

  const VPointCloud::Ptr& get_map_cloud() const {
    return map_cloud_;
  }

private:

  
  void undistort(const Eigen::Quaterniond& q_G_to_target, //q_Lk_I0
                 const Eigen::Vector3d& p_target_in_G, //p_I0_Lk
                 const TPointCloud& scan_raw, //Lk's points
                 const VPointCloud::Ptr& scan_in_target,
                 bool correct_position = false) const {
    scan_in_target->header = scan_raw.header;
    scan_in_target->height = scan_raw.height;
    scan_in_target->width  = scan_raw.width;
    scan_in_target->resize(scan_raw.height * scan_raw.width);
    scan_in_target->is_dense = false;

    VPoint NanPoint;
    NanPoint.x = NAN; NanPoint.y = NAN; NanPoint.z = NAN;
    for (int h = 0; h < scan_raw.height; h++) {
      for (int w = 0; w < scan_raw.width; w++) {
        VPoint vpoint;
        if (pcl_isnan(scan_raw.at(w,h).x)) {
          vpoint = NanPoint;
          scan_in_target->at(w,h) = vpoint;
          continue;
        }
        double point_timestamp = scan_raw.at(w,h).timestamp; //每个点的时间戳
        Eigen::Quaterniond q_Lk_to_G; //q_I0_(Lk's points)
        Eigen::Vector3d p_Lk_in_G; //P_I0_(Lk's points)
        if (!traj_manager_->evaluateLidarPose(point_timestamp, q_Lk_to_G, p_Lk_in_G)) {
          continue;
        }
        Eigen::Quaterniond q_LktoL0 = q_G_to_target * q_Lk_to_G; //q_Lk_(Lk's points): laser时间戳时的laser frame--->该point时间戳时的laser frame
        Eigen::Vector3d p_Lk(scan_raw.at(w,h).x, scan_raw.at(w,h).y, scan_raw.at(w,h).z); //每个点的坐标是在该point时间戳时的laser frame下

        Eigen::Vector3d point_out;
        if (!correct_position) {
          point_out = q_LktoL0 * p_Lk; //去旋转畸变，转换到laser时间戳时的laser frame
        } else {//去旋转和平移畸变，转换到laser时间戳时的laser frame
          point_out = q_LktoL0 * p_Lk + q_G_to_target * (p_Lk_in_G - p_target_in_G);
        }

        vpoint.x = point_out(0);
        vpoint.y = point_out(1);
        vpoint.z = point_out(2);
        vpoint.intensity = scan_raw.at(w,h).intensity;
        scan_in_target->at(w,h) = vpoint;
      }
    }
  }


  TrajectoryManager::Ptr traj_manager_;
  std::shared_ptr<IO::LioDataset> dataset_reader_;

  std::map<pcl::uint64_t, VPointCloud::Ptr> scan_data_; //<每帧scan时间戳，去畸变后的scan在各帧laser frame下>
  std::map<pcl::uint64_t, VPointCloud::Ptr> scan_data_in_map_;//<每帧scan时间戳，去畸变后的scan转换到map下(L0下)>
  VPointCloud::Ptr map_cloud_; //scan_data_in_map_累加起来
};

}