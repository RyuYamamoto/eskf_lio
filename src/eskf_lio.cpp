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

#include "eskf_lio/eskf_lio.hpp"

ESKFLio::ESKFLio(ESKFLio::LioConfig config)
: config_(config),
  transformation_(Eigen::Matrix4d::Identity()),
  imu_bias_(Eigen::Vector<double, 6>::Zero())
{
  // Registration
  registration_ = std::make_shared<fast_gicp::FastVGICP<PointType, PointType>>();
  registration_->setResolution(config.resolution);
  registration_->setMaxCorrespondenceDistance(config.max_correspondence_distance);
  registration_->setCorrespondenceRandomness(config.correspondence_randomness);
  registration_->setTransformationEpsilon(config.transformation_epsilon);
  registration_->setMaximumIterations(config.max_iteration);
  registration_->setNumThreads(config.omp_num_thread);

  // ESKF
  eskf::ESKF::ESKFConfig eskf_config;
  eskf_config.acc_noise = config.acc_noise;
  eskf_config.gyro_noise = config.gyro_noise;
  eskf_config.acc_bias_noise = config.acc_bias_noise;
  eskf_config.gyro_bias_noise = config.gyro_bias_noise;
  eskf_config.translation_noise = config.translation_noise;
  eskf_config.rotation_noise = config.rotation_noise;
  eskf_ = std::make_shared<eskf::ESKF>(eskf_config);

  // IMU Initializer
  imu_ = std::make_shared<ImuInitializer>(config.imu_calibration_time, config.gravity);

  // Map Manager
  map_manager_ = std::make_shared<MapManager>(
    config.voxel_map_resolution, config.max_submap_size, config.translation_threshold,
    config.rotation_threshold);

  local_map_.reset(new PointCloud);
}

ESKFLio::~ESKFLio()
{
}

bool ESKFLio::sync_measurement(sensor_type::Measurement & measurement)
{
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  measurement.lidar_points = lidar_buffer_.front();
  lidar_buffer_.pop_front();

  while (!imu_buffer_.empty()) {
    auto imu = imu_buffer_.front();
    if (measurement.lidar_points.lidar_end_time < imu.stamp) {
      break;
    }

    measurement.imu_queue.push_back(imu);
    imu_buffer_.pop_front();
  }

  if (!imu_buffer_.empty()) {
    measurement.imu_queue.push_back(imu_buffer_.front());
  }

  return true;
}

bool ESKFLio::imu_static_calibration(const std::deque<sensor_type::Imu> & imu_queue)
{
  bool is_initialized = imu_->is_initialized();
  if (!is_initialized) {
    for (auto imu : imu_queue) {
      imu_->add_imu(imu);
    }
  }
  return is_initialized;
}

void ESKFLio::initialize(const sensor_type::Measurement & measurement)
{
  if (!imu_static_calibration(measurement.imu_queue)) {
    return;
  }

  transformation_.block<3, 3>(0, 0) = imu_->get_initial_orientation();

  imu_bias_.head<3>() = imu_->get_acc_mean();
  imu_bias_.tail<3>() = imu_->get_gyro_mean();

  // Kalman Filter
  eskf_->initialize(
    transformation_, imu_bias_, imu_->get_gravity(), measurement.lidar_points.stamp);

  // Update Initial Local Map
  update_local_map(transformation_, measurement.lidar_points, true);

  initialized_ = true;
}

std::vector<Sophus::SE3d> ESKFLio::predict(sensor_type::Measurement & measurement)
{
  std::vector<Sophus::SE3d> imu_states;
  for (auto & imu : measurement.imu_queue) {
    eskf_->predict(imu);
    auto state = eskf_->get_state().get_x();
    imu_states.emplace_back(Sophus::SE3d(state.block<3, 3>(0, 0), state.block<3, 1>(0, 3)));
  }
  return imu_states;
}

bool ESKFLio::update(const sensor_type::Measurement & measurement)
{
  auto lidar_points = measurement.lidar_points;

  Eigen::Matrix4d initial_guess = eskf_->get_state().get_x();
  Eigen::Matrix4d result_pose;
  if (!scan_matching(lidar_points.preprocessing_points, initial_guess, result_pose)) {
    return false;
  }

  sensor_type::Pose pose_measurement;
  pose_measurement.stamp = lidar_points.stamp;
  pose_measurement.pose = result_pose;
  transformation_ = eskf_->update(pose_measurement);

  return true;
}

bool ESKFLio::scan_matching(
  const PointCloudPtr input_cloud_ptr, const Eigen::Matrix4d & initial_guess,
  Eigen::Matrix4d & result_pose)
{
  if (mapping_future_.valid()) {
    if (
      mapping_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
      map_manager_->has_map_changed()) {
      local_map_ = mapping_future_.get();
      registration_->setInputTarget(local_map_);
      map_manager_->reset();
    }
  }

  if (registration_->getInputTarget() == nullptr) {
    std::cerr << "not input target" << std::endl;
    return false;
  }

  registration_->setInputSource(input_cloud_ptr);
  PointCloudPtr align_cloud(new PointCloud);
  registration_->align(*align_cloud, initial_guess.cast<float>());

  if (!registration_->hasConverged()) {
    std::cerr << "not converged" << std::endl;
    return false;
  }

  result_pose = registration_->getFinalTransformation().cast<double>();

  return true;
}

bool ESKFLio::update_local_map(
  const Eigen::Matrix4d & pose, const sensor_type::Lidar & lidar_points, const bool first)
{
  if (first) {
    map_manager_->clear_map();
  }

  bool is_map_updated = map_manager_->is_map_update(pose);
  if (is_map_updated) {
    mapping_future_ = map_manager_->add_map_points(lidar_points, pose);
    if (first) {
      mapping_future_.wait();
    }
  }

  return is_map_updated;
}

PointCloudPtr ESKFLio::preprocessing(const PointCloudPtr & cloud_in)
{
  PointCloudPtr downsampling_cloud(new PointCloud);
  scan_voxel_grid_.setInputCloud(cloud_in);
  scan_voxel_grid_.filter(*downsampling_cloud);

  PointCloudPtr crop_cloud(new PointCloud);
  crop_.setInputCloud(downsampling_cloud);
  crop_.filter(*crop_cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*crop_cloud, *crop_cloud, indices);

  PointCloudPtr filtered_inf_cloud(new PointCloud);

  for (const auto & point : crop_cloud->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      filtered_inf_cloud->points.push_back(point);
    }
  }
  filtered_inf_cloud->width = filtered_inf_cloud->points.size();
  filtered_inf_cloud->height = 1;
  filtered_inf_cloud->is_dense = true;

  return filtered_inf_cloud;
}
