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
#include "realsense_interface/realsense_device.hpp"
#include <iostream>

RealsenseDevice::RealsenseDevice(){};
RealsenseDevice::~RealsenseDevice() { pipe_.stop(); };
bool RealsenseDevice::identifyDevice() {
  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    return false;
  }
  for (auto dev : devices) {
    if (dev.supports(RS2_CAMERA_INFO_NAME)) {
      auto device_name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
      std::cout << "Device Name: " << device_name << std::endl;

      if (verbose_) {
        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
          std::cout << "Device Serial No: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                    << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
          std::cout << "Device FW version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
                    << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT))
          std::cout << "Device physical port: " << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT)
                    << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE))
          std::cout << "Device supports debug op code: "
                    << dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE) << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID))
          std::cout << "Device product ID: " << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID)
                    << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
          std::cout << "Device supports locked camera: "
                    << dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED) << std::endl;
        if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
          std::cout << "Device USB type descriptor: "
                    << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
      }
      identifySensors(dev);
    }
  }
  return true;
};

bool RealsenseDevice::identifySensors(const rs2::device &dev) {
  std::vector<rs2::sensor> dev_sensors = dev.query_sensors();
  std::cout << "Available streams: " << std::endl;
  for (auto sensor : dev_sensors) {
    for (auto profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() == RS2_STREAM_GYRO) {
        available_streams_[RS2_STREAM_GYRO] = RS2_FORMAT_MOTION_XYZ32F;
      }
      if (profile.stream_type() == RS2_STREAM_ACCEL) {
        available_streams_[RS2_STREAM_ACCEL] = RS2_FORMAT_MOTION_XYZ32F;
      }
      if (profile.stream_type() == RS2_STREAM_DEPTH) {
        available_streams_[RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;
      }
      if (profile.stream_type() == RS2_STREAM_COLOR) {
        available_streams_[RS2_STREAM_COLOR] = RS2_FORMAT_RGB8;
      }
      if (profile.stream_type() == RS2_STREAM_POSE) {
        available_streams_[RS2_STREAM_POSE] = RS2_FORMAT_6DOF;
      }
      if (profile.stream_type() == RS2_STREAM_FISHEYE) {
        available_streams_[RS2_STREAM_FISHEYE] = RS2_FORMAT_Y8;
      }
    }
  }

  return true;
};

bool RealsenseDevice::setup() {
  if (!identifyDevice()) {
    std::cerr << "No device found" << std::endl;
    device_not_found_ = true;
    return false;
  }
  rs2::config cfg;
  if (!serial_.empty()) {
    cfg.enable_device(serial_);
  }

  for (const auto &_stream : streams_) {
    cfg.enable_stream(_stream, available_streams_[_stream]);
  }

  // Start pipeline with chosen configuration
  pipe_.start(cfg);
  return true;
};

void RealsenseDevice::getFrameset() { frameset_ = pipe_.wait_for_frames(); }

std::array<double, 3> RealsenseDevice::getAccelData(double *timestamp) {
  if (std::find(streams_.begin(), streams_.end(), RS2_STREAM_ACCEL) == streams_.end()) {
    std::cerr << "No accel stream enabled" << std::endl;
    return {0, 0, 0};
  }
  std::array<double, 3> accel_data;
  rs2::frameset frameset  = pipe_.wait_for_frames();
  rs2::frame accel_frame  = frameset.first_or_default(RS2_STREAM_ACCEL);
  rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
  if (timestamp != nullptr) {
    *timestamp = accel.get_timestamp();
  }

  accel_data[0] = accel.get_motion_data().x;
  accel_data[1] = accel.get_motion_data().y;
  accel_data[2] = accel.get_motion_data().z;
  return accel_data;
};

std::array<double, 3> RealsenseDevice::getGyroData(double *timestamp) {
  if (std::find(streams_.begin(), streams_.end(), RS2_STREAM_GYRO) == streams_.end()) {
    std::cerr << "No gyro stream enabled" << std::endl;
    return {0, 0, 0};
  }
  std::array<double, 3> gyro_data;
  rs2::frameset frameset = pipe_.wait_for_frames();
  rs2::frame gyro_frame  = frameset.first_or_default(RS2_STREAM_GYRO);
  rs2::motion_frame gyro = gyro_frame.as<rs2::motion_frame>();
  if (timestamp != nullptr) {
    *timestamp = gyro.get_timestamp();
  }

  gyro_data[0] = gyro.get_motion_data().x;
  gyro_data[1] = gyro.get_motion_data().y;
  gyro_data[2] = gyro.get_motion_data().z;
  return gyro_data;
};

std::array<double, 7> RealsenseDevice::getPoseData(double *timestamp) {
  std::array<double, 7> pose_data;
  rs2::frameset frameset = pipe_.wait_for_frames();
  rs2::frame pose_frame  = frameset.first_or_default(RS2_STREAM_POSE);
  rs2::pose_frame pose   = pose_frame.as<rs2::pose_frame>();
  if (timestamp != nullptr) {
    *timestamp = pose.get_timestamp();
  }

  pose_data[0] = pose.get_pose_data().translation.x;
  pose_data[1] = pose.get_pose_data().translation.y;
  pose_data[2] = pose.get_pose_data().translation.z;
  pose_data[3] = pose.get_pose_data().rotation.x;
  pose_data[4] = pose.get_pose_data().rotation.y;
  pose_data[5] = pose.get_pose_data().rotation.z;
  pose_data[6] = pose.get_pose_data().rotation.w;
  return pose_data;
};

std::array<double, 3> RealsenseDevice::getVelocityData(double *timestamp) {
  std::array<double, 3> velocity_data;
  rs2::pose_frame velocity_frame = frameset_.get_pose_frame();
  if (timestamp != nullptr) {
    *timestamp = velocity_frame.get_timestamp();
  }
  const auto &velocity = velocity_frame.get_pose_data().velocity;
  velocity_data[0]     = velocity.x;
  velocity_data[1]     = velocity.y;
  velocity_data[2]     = velocity.z;
  return velocity_data;
}

bool RealsenseDevice::getCameraFrame(cv::Mat *image,
                                     rs2_stream stream_type,
                                     int index,
                                     double *timestamp) {
  if (stream_type == RS2_STREAM_COLOR) {
    auto color_frame = frameset_.get_color_frame();
    if (timestamp != nullptr) {
      *timestamp = color_frame.get_timestamp();
    }
    cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3,
                  (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color, *image, cv::COLOR_RGB2BGR);
  } else if (stream_type == RS2_STREAM_DEPTH) {
    auto depth_frame = frameset_.get_depth_frame();
    if (timestamp != nullptr) {
      *timestamp = depth_frame.get_timestamp();
    }
    cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1,
                  (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    depth.convertTo(*image, CV_32FC1, 0.001);
  } else if (stream_type == RS2_STREAM_INFRARED) {
    auto ir_frame = frameset_.get_infrared_frame(index);
    if (timestamp != nullptr) {
      *timestamp = ir_frame.get_timestamp();
    }
    cv::Mat ir(cv::Size(ir_frame.get_width(), ir_frame.get_height()), CV_8UC1,
               (void *)ir_frame.get_data(), cv::Mat::AUTO_STEP);
    ir.convertTo(*image, CV_32FC1);
  } else if (stream_type == RS2_STREAM_FISHEYE) {
    auto fisheye_frame = frameset_.get_fisheye_frame(index);
    if (timestamp != nullptr) {
      *timestamp = fisheye_frame.get_timestamp();
    }
    cv::Mat fisheye(cv::Size(fisheye_frame.get_width(), fisheye_frame.get_height()), CV_8UC1,
                    (void *)fisheye_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(fisheye, *image, cv::COLOR_GRAY2BGR);
  } else {
    throw std::runtime_error("Unsupported stream type");
    return false;
  }
  return true;
}

CameraInfo RealsenseDevice::getCameraInfo(rs2_stream stream_type, int index) {
  CameraInfo camera_info;
  // if stream type is fisheye we need to differentiate between left and right ( index 1 and 2 )

  rs2_intrinsics intrinsic;
  rs2_extrinsics extrinsic;
  // rs2_vid
  rs2::video_stream_profile stream_profile;
  if (stream_type == RS2_STREAM_FISHEYE) {
    stream_profile = pipe_.get_active_profile()
                         .get_stream(RS2_STREAM_FISHEYE, index)
                         .as<rs2::video_stream_profile>();

  } else {
    stream_profile =
        pipe_.get_active_profile().get_stream(stream_type).as<rs2::video_stream_profile>();
  }

  intrinsic = stream_profile.get_intrinsics();
  // extrinsic = stream_profile.get_extrinsics_to(stream_profile);

  camera_info.width  = intrinsic.width;
  camera_info.height = intrinsic.height;
  camera_info.K[0]   = intrinsic.fx;
  camera_info.K[2]   = intrinsic.ppx;
  camera_info.K[4]   = intrinsic.fy;
  camera_info.K[5]   = intrinsic.ppy;
  camera_info.K[8]   = 1;

  // Distortion model
  camera_info.distortion_model = rs2_distortion_to_string(intrinsic.model);
  // Distorsion coefficients
  for (int i = 0; i < 5; i++) {
    camera_info.D[i] = intrinsic.coeffs[i];
  }

  // rectification matrix
  camera_info.R[0] = 1;
  camera_info.R[4] = 1;
  camera_info.R[8] = 1;
  // projection matrix
  camera_info.R[0]  = camera_info.K[0];
  camera_info.R[2]  = camera_info.K[2];
  camera_info.R[5]  = camera_info.K[4];
  camera_info.R[6]  = camera_info.K[5];
  camera_info.R[10] = 1;

  // set encoding if stream type is depth
  if (stream_type == RS2_STREAM_DEPTH) {
    camera_info.encoding = CameraInfo::ENCODING::Z16;
  } else {
    camera_info.encoding = CameraInfo::ENCODING::BGR8;
  }
  return camera_info;
};

/* bool RealsenseDevice::getDepthFrame(cv::Mat *, double *timestamp){};
CameraInfo RealsenseDevice::getDepthInfo(){}; */

// std::array<double, 7> RealsenseDevice::getImuExtrinsics(){};

