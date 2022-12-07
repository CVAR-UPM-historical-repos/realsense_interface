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

#ifndef REALSENSE_ROS_DEVICE_HPP_
#define REALSENSE_ROS_DEVICE_HPP_

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "realsense_interface/realsense_device.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/* void RealsenseInterface::setStaticTransform(const std::string _link_frame,
                                            const std::string _ref_frame,
                                            const std::array<double, 3> &_translation,
                                            const std::array<double, 3> &_rotation) {
  std::cout << "TF_DEVICE: " << _link_frame << std::endl;
  std::cout << "TF_REF: " << _ref_frame << std::endl;
  std::cout << "TF_TRANSLATION: " << _translation[0] << " " << _translation[1] << " "
            << _translation[2] << std::endl;
  std::cout << "TF_ROTATION: " << _rotation[0] << " " << _rotation[1] << " " << _rotation[2]
            << std::endl;

  // Publish static transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp            = this->now();
  transform_stamped.header.frame_id         = _ref_frame;
  transform_stamped.child_frame_id          = _link_frame;
  transform_stamped.transform.translation.x = _translation[0];
  transform_stamped.transform.translation.y = _translation[1];
  transform_stamped.transform.translation.z = _translation[2];
  tf2::Quaternion q;
  q.setRPY(_rotation[0], _rotation[1], _rotation[2]);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  tf_static_broadcaster_->sendTransform(transform_stamped);
} */

class RealsenseROSDevice : public RealsenseDevice {
protected:
  rs2_stream base_link_stream_ = RS2_STREAM_ANY;

public:
  /**
   * @brief Constructor of the RealsenseROSDevice class.
   */
  RealsenseROSDevice();
  ~RealsenseROSDevice();

  std::vector<geometry_msgs::msg::TransformStamped> realsense_transforms_;
  void setStaticTransform();  // SET TF BETWEEN CAMERA AND BASE LINK
  // GET TF BETWEEN SENSOR_BASE_LINK AND EACH SENSOR
  std::vector<geometry_msgs::msg::TransformStamped> getTFsBetweenSensors();

  void RGBCameraCb();
  void DepthCameraCb();
  void FishEyeCamerasCb();
  void ImuCb();
  void OdometryCb();

  void enableRGB();
  void enableDepth();
  void enableFishEye();
  void enableIMU();
  void enableOdometry();

private:
  as2::Node *node_;
  RealsenseDevice device_;

protected:
private:
};

#endif  // REALSENSE_ROS2_DEVICE__
