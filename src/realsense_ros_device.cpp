/*!*******************************************************************************************
 *  \file       realsense_interface.hpp
 *  \brief      Realsense camera interface header file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#include "realsense_interface/realsense_ros_device.hpp"
#include <tf2/LinearMath/Transform.h>
#include "realsense_device.hpp"

RealsenseROSDevice::RealsenseROSDevice() { device_ = RealsenseDevice(); };
RealsenseROSDevice::~RealsenseROSDevice(){};

void RealsenseROSDevice::setStaticTransform(){};

std::vector<geometry_msgs::msg::TransformStamped> RealsenseROSDevice::getTFsBetweenSensors(){};

void RealsenseROSDevice::RGBCameraCb() {
  std::cout << "RGB CAMERA CB" << std::endl;
  cv::Mat rgb_image;
  double timestamp;
  getCameraFrame(&rgb_image, RS2_STREAM_COLOR, 0, &timestamp);
};

void RealsenseROSDevice::DepthCameraCb(){
    // std::cout << "DEPTH CAMERA CB" << std::endl;
    // cv::Mat depth_image;
    // double timestamp;
    // getCameraFrame(&depth_image, RS2_STREAM_DEPTH, 0, &timestamp);
};

void RealsenseROSDevice::FishEyeCamerasCb(){
    // std::cout << "FISHEYE CAMERA CB" << std::endl;
    // cv::Mat fisheye_image;
    // double timestamp;
    // getCameraFrame(&fisheye_image, RS2_STREAM_FISHEYE, 0, &timestamp);
};

void RealsenseROSDevice::ImuCb(){
    // std::cout << "IMU CB" << std::endl;
    // cv::Mat imu_data;
    // double timestamp;
    // getCameraFrame(&imu_data, RS2_STREAM_FISHEYE, 0, &timestamp);
};

void RealsenseROSDevice::OdometryCb() {
  double pose_timestamp, twist_timestamp;
  std::array<double, 7> pose  = device_.getPoseData(&pose_timestamp);
  std::array<double, 3> twist = device_.getVelocityData(&twist_timestamp);

  rclcpp::Time timestamp = rclcpp::Time(pose_timestamp * 1000);
  tf2::Transform realsense_pose(tf2::Quaternion(pose[3], pose[4], pose[5], pose[6]),
                                tf2::Vector3(pose[0], pose[1], pose[2]));
};

void RealsenseROSDevice::enableRGB() {
  if (enableStream(RS2_STREAM_COLOR)) {
    static auto RGB_timer = node_->create_timer(std::chrono::milliseconds(100),
                                                std::bind(&RealsenseROSDevice::RGBCameraCb, this));
  }
};

void RealsenseROSDevice::enableDepth() {
  if (enableStream(RS2_STREAM_DEPTH)) {
    static auto Depth_timer = node_->create_timer(
        std::chrono::milliseconds(100), std::bind(&RealsenseROSDevice::DepthCameraCb, this));
  }
};

void RealsenseROSDevice::enableFishEye() {
  if (enableStream(RS2_STREAM_FISHEYE)) {
    static auto FishEye_timer = node_->create_timer(
        std::chrono::milliseconds(100), std::bind(&RealsenseROSDevice::FishEyeCamerasCb, this));
  }
};

void RealsenseROSDevice::enableIMU() {
  if (enableStream(RS2_STREAM_ACCEL) && enableStream(RS2_STREAM_GYRO)) {
    static auto IMU_timer = node_->create_timer(std::chrono::milliseconds(100),
                                                std::bind(&RealsenseROSDevice::ImuCb, this));
  }
};

void RealsenseROSDevice::enableOdometry() {
  if (enableStream(RS2_STREAM_POSE)) {
    static auto Odometry_timer = node_->create_timer(
        std::chrono::milliseconds(100), std::bind(&RealsenseROSDevice::OdometryCb, this));
  }
};
