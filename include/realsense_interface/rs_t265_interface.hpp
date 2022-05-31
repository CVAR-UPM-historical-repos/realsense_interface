/*!*******************************************************************************************
 *  \file       rs_t265_interface.hpp
 *  \brief      Realsense t265 tracking camera interface header file.
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

#ifndef RS_T265_INTERFACE_HPP_
#define RS_T265_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include <as2_core/tf_utils.hpp>

#include <librealsense2/rs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iomanip>
#include <memory>
#include <iostream>
#include <string>
#include <map>
#include <algorithm>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

/**
 * @brief Check the stream type of the device
 * 
 * @param stream_requests 
 * @param out_serial 
 * @return true  Vio device found
 * @return false Vio device not found
 */
bool find_device_with_stream(std::vector<rs2_stream> stream_requests, std::string &out_serial);

/**
 * @brief Check if the device provides imu information

 * @return true  Device provides IMU information
 * @return false Device doesn't provide IMU information
 */
bool check_imu_is_supported();

class RsT265Interface : public as2::Node
{
public:
  /**
     * @brief Constructor of the RsT265Interface object
     * 
     */
  RsT265Interface();

  // void setup_imu();
  // void run_imu();
  // void ownSetup(){
  //     setup_odom();
  //     setup_tf();
  // }
  // void ownRun(){
  //     run_odom();
  //     run_tf();
  // }

  /**
     * @brief Initial setup for node odometry.
     * This function check if the VIO device is ready and starts the pipeline.
     */
  void setupOdom();

  /**
     * @brief Stop rutine for odometry node.
     * This function stops the pipeline.
     */
  void stopOdom();

  /**
     * @brief Funtionality during node lifetime.
     * This function gets the pose data from VIO device,
     * updates the transform tree and
     * and publish the odometry message.
     */
  void runOdom();

  /**
     * @brief Initial setup for node Tranforms
     * This function generate the Transforms tree
     * from the relative position between links.
     */
  void setupTf();

  /**
     * @brief Publish Transforms tree
     * This function publish the Transformation
     * between base link and device link
     */
  void publishTFs();

  /**
     * @brief Set the Tf Tree object
     * This function updates the Tranform tree
     */
  void setTfTree();

private:
  // Sensor comm
  std::string serial_;
  rs2::pipeline pipe_;
  // Sensor measurement
  std::shared_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_;
  std::shared_ptr<as2::sensors::Imu> imu_sensor_;
  // Sensor Tf
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_tree_;

  std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
  geometry_msgs::msg::TransformStamped rs_odom2rs_link_tf_;
  // geometry_msgs::msg::TransformStamped rs_odom2rs_link_vel_tf_;
  std::string base_link_str;
  std::string rs_link_str;
  std::string odom_str;

  // Camera offsets from base_link frame ENU
  const float camera_offset_x_ = 0.0f;
  const float camera_offset_y_ = -0.11f;
  const float camera_offset_z_ = 0.01f;
  const float camera_offset_roll_ = -47.0f / 180.0f * M_PI;
  const float camera_offset_pitch_ = 0.0f;
  const float camera_offset_yaw_ = 180.0f / 180.0f * M_PI;
};

#endif // RS_T265_INTERFACE_HPP_