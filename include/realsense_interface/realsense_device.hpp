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

#ifndef REALSENSE_DEVICE_HPP_
#define REALSENSE_DEVICE_HPP_

#include <librealsense2/hpp/rs_frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"

#include <librealsense2/h/rs_sensor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <librealsense2/rs.hpp>

#include <math.h>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

// struct stream_config {
//   bool gyro    = false;
//   bool accel   = false;
//   bool pose    = false;
//   bool color   = false;
//   bool depth   = false;
//   bool fisheye = false;

//   // make this iterable
//   std::vector<bool> get_vector() { return {imu, pose, color, depth, fisheye}; }
// };

struct CameraInfo {
  enum ENCODING { Y8, Y16, Z16, RGB8, RGBA8, BGR8, BGRA8, YUYV };
  int width, height;         // image width and height in pixels
  std::array<double, 9> K;   // 3x3 matrix of [fx 0 cx; 0 fy cy; 0 0 1]
  std::array<double, 5> D;   // distortion coefficients
  std::array<double, 12> P;  // 3x4 projection matrix of [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]
  std::array<double, 9> R;   // 3x3 rectification matrix (stereo cameras only)
  ENCODING encoding;
  std::string distortion_model;  // distortion model used normally "plumb_bob"
};

class RealsenseDevice {
public:
  /**
   * @brief Constructor of the RealsenseDevice class.
   */
  RealsenseDevice();
  ~RealsenseDevice();

private:
  // COMMON ATTRIBUTES
  // std::string realsense_name_;
  bool verbose_;
  bool device_not_found_;
  std::shared_ptr<rs2::motion_frame> accel_frame_;
  std::shared_ptr<rs2::motion_frame> gyro_frame_;

  rs2::frameset frameset_;

  // stream_config available_streams_;

  // Sensor comm
  // rs2::pipeline pipe_;

  std::string serial_;

protected:
  rs2::pipeline pipe_;
  std::vector<rs2_stream> streams_;
  std::unordered_map<rs2_stream, rs2_format> available_streams_;

  bool enableStream(rs2_stream _stream) {
    if (available_streams_.find(_stream) != available_streams_.end()) {
      streams_.emplace_back(_stream);
      return true;
    }
    return false;
  };

  rs2_extrinsics getExtrinsics(rs2_stream from, rs2_stream to) {
    return pipe_.get_active_profile().get_stream(from).get_extrinsics_to(
        pipe_.get_active_profile().get_stream(to));
  }
  bool identifySensors(const rs2::device &dev);
  bool identifyDevice();

public:
  // COMMON
  bool setup();
  void getFrameset();

  /**
   * @brief Get the Acceleration Data Vector
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return std::array<double, 3> acceleration data vector (x, y, z) in m/s^2
   */
  std::array<double, 3> getAccelData(double *timestamp = nullptr);

  /**
   * @brief Get the Gyro Data Vector
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return std::array<double, 3> gyro data vector (x, y, z) in rad/s
   */
  std::array<double, 3> getGyroData(double *timestamp = nullptr);

  /**
   * @brief Get the Pose Data Vector
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return std::array<double, 7> pose data vector (x, y, z, qx, qy, qz, qw) in m
   */
  std::array<double, 7> getPoseData(double *timestamp = nullptr);

  /**
   * @brief Get the Velocity Data Vector
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return std::array<double, 3> velocity data vector (x, y, z) in m/s
   */
  std::array<double, 3> getVelocityData(double *timestamp = nullptr);

  /**
   * @brief Get the Camera Frame from the camera stream at the given index
   * @param *cv::Mat pointer to the cv::Mat where the frame will be stored
   * @param index index of the camera stream (default 0)
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return bool true if the frame was successfully retrieved
   */
  bool getCameraFrame(cv::Mat *,
                      rs2_stream stream_type,
                      int index         = 0,
                      double *timestamp = nullptr);

  /**
   * @brief Get the Camera Info from the camera stream at the given index
   * @param index index of the camera stream (default 0)
   * @return CameraInfo struct with the camera info
   */
  CameraInfo getCameraInfo(rs2_stream stream_type, int index = 0);

  /**
   * @brief Get the Camera Frame from the Depth stream
   * @param *cv::Mat pointer to the cv::Mat where the frame will be stored
   * @param *timestamp pointer to the timestamp of the frame (optional)
   * @return bool true if the frame was successfully retrieved
   */
};

#endif  // REALSENSE_DEVICE_HPP_
