// Copyright 2025 Ryu Yamamoto, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESKF_LIO__ESKF_LIO_HPP_
#define ESKF_LIO__ESKF_LIO_HPP_

#include "eskf_lio/eskf.hpp"
#include "eskf_lio/imu_initializer.hpp"
#include "eskf_lio/lib/concurrent_queue.hpp"
#include "eskf_lio/lib/sensor_type.hpp"
#include "eskf_lio/map_mananger.hpp"
#include "eskf_lio/submap.hpp"

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <sophus/se3.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

class ESKFLio
{
public:
  struct LioConfig
  {
    // FAST GICP
    double max_correspondence_distance;
    double transformation_epsilon;
    double correspondence_randomness;
    double resolution;
    int max_iteration;
    int omp_num_thread;

    // Local Map
    double translation_threshold;
    double rotation_threshold;
    double voxel_map_resolution;
    int max_submap_size;

    // ESKF
    double acc_noise;
    double gyro_noise;
    double acc_bias_noise;
    double gyro_bias_noise;
    double translation_noise;
    double rotation_noise;
    double gravity;

    // IMU
    double imu_calibration_time;
  };

  ESKFLio(LioConfig config = LioConfig());
  ~ESKFLio();

  // points preprocessing
  inline void set_scan_voxel_size(const double voxel_size)
  {
    scan_voxel_grid_.setLeafSize(voxel_size, voxel_size, voxel_size);
  }
  inline void set_crop_area(const Eigen::Vector4f min, const Eigen::Vector4f max)
  {
    crop_.setNegative(true);
    crop_.setMin(min);
    crop_.setMax(max);
  }

  bool is_initialized() { return initialized_; }
  void initialize(const sensor_type::Measurement & measurement);
  bool imu_static_calibration(const std::deque<sensor_type::Imu> & imu);

  std::vector<Sophus::SE3d> predict(sensor_type::Measurement & measurement);
  bool update(const sensor_type::Measurement & measurement);

  bool scan_matching(
    const PointCloudPtr input_cloud_ptr, const Eigen::Matrix4d & initial_guess,
    Eigen::Matrix4d & result_pose);

  bool update_local_map(
    const Eigen::Matrix4d & pose, const sensor_type::Lidar & lidar_points,
    const bool first = false);

  inline void insert_points(const sensor_type::Lidar & points) { lidar_buffer_.push_back(points); }
  inline void insert_imu(const sensor_type::Imu & imu) { imu_buffer_.push_back(imu); }

  bool sync_measurement(sensor_type::Measurement & measurement);

  PointCloudPtr preprocessing(const PointCloudPtr & cloud_in);

  inline Eigen::Matrix4d get_result() { return transformation_; }
  inline Eigen::Matrix<double, 6, 6> get_covariance() { return covariance_; }

  inline PointCloudPtr get_local_map() { return local_map_; }

private:
  std::shared_ptr<ImuInitializer> imu_;
  std::shared_ptr<eskf::ESKF> eskf_;
  std::shared_ptr<MapManager> map_manager_;

  std::mutex registration_mutex_;
  std::shared_ptr<fast_gicp::FastVGICP<PointType, PointType>> registration_;

  LioConfig config_;

  ConcurrentQueue<sensor_type::Lidar> lidar_buffer_;
  ConcurrentQueue<sensor_type::Imu> imu_buffer_;

  pcl::VoxelGrid<PointType> scan_voxel_grid_;
  pcl::CropBox<PointType> crop_;

  PointCloudPtr local_map_;
  Eigen::Matrix4d transformation_;
  Eigen::Vector<double, 6> imu_bias_;
  Eigen::Matrix<double, 6, 6> covariance_;

  bool initialized_{false};

  std::future<PointCloudPtr> mapping_future_;
};

#endif
