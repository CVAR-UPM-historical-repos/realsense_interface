/*!*******************************************************************************************
 *  \file       rs_t265_interface.cpp
 *  \brief      Realsense t265 tracking camera interface implementation file.
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

#include "rs_t265_interface.hpp"

RsT265Interface::RsT265Interface():as2::Node("rs_t265"){

  odom_       = std::make_shared<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom",this);
  imu_sensor_ = std::make_shared<as2::sensors::Imu>("imu",this);
  // Initialize the transform broadcaster
  tf_broadcaster_       = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_            = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_tree_       = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_          = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_tree_);
  std::string ns = this->get_namespace();
  base_link_str = generateTfName(ns,"base_link");
  rs_link_str   = generateTfName(ns,"rs_link");
  odom_str      = generateTfName(ns,"odom");

};

void RsT265Interface::setupOdom(){

  if (!find_device_with_stream({RS2_STREAM_POSE}, serial_))
  {
    RCLCPP_ERROR(this->get_logger(),"VIO camera not found");
    exit(EXIT_SUCCESS);
  }
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  if (!serial_.empty())
    cfg.enable_device(serial_);
  // Add pose stream
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  // Start pipeline with chosen configuration
  pipe_.start(cfg);

  RCLCPP_INFO(this->get_logger(),"VIO device node ready");
    
};

void RsT265Interface::stopOdom(){
  pipe_.stop();
};

void RsT265Interface::runOdom(){
    
  // Wait for the next set of frames from the camera
  auto frames = pipe_.wait_for_frames();
  // Get a frame from the pose stream
  auto f = frames.first_or_default(RS2_STREAM_POSE);
  // Cast the frame to pose_frame and get its data
  auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

  // TF
  rclcpp::Time timestamp = this->get_clock()->now();

  rs_odom2rs_link_tf_.header.frame_id = "rs_odom";
  rs_odom2rs_link_tf_.child_frame_id  = rs_link_str;
  rs_odom2rs_link_tf_.header.stamp    = timestamp;
  rs_odom2rs_link_tf_.transform.translation.x = pose_data.translation.x;
  rs_odom2rs_link_tf_.transform.translation.y = pose_data.translation.y;
  rs_odom2rs_link_tf_.transform.translation.z = pose_data.translation.z;
  rs_odom2rs_link_tf_.transform.rotation.x = pose_data.rotation.x;
  rs_odom2rs_link_tf_.transform.rotation.y = pose_data.rotation.y;
  rs_odom2rs_link_tf_.transform.rotation.z = pose_data.rotation.z;
  rs_odom2rs_link_tf_.transform.rotation.w = pose_data.rotation.w;

  setTfTree();
  publishTFs();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odom_str;
  odom_msg.child_frame_id  = base_link_str;
  odom_msg.header.stamp    = timestamp;

  geometry_msgs::msg::Vector3Stamped vio_twist_linear_vect;
	geometry_msgs::msg::Vector3Stamped vio_twist_angular_vect ;

  geometry_msgs::msg::Vector3Stamped odom_twist_linear_vect;
  geometry_msgs::msg::Vector3Stamped rs_link_twist_angular_vect;
  geometry_msgs::msg::Vector3Stamped base_link_twist_angular_vect;
  // geometry_msgs::msg::Vector3Stamped base_link_vio_twist_linear_vect;
  // geometry_msgs::msg::Vector3Stamped base_link_sum_twist_linear_vect;

  vio_twist_linear_vect.vector.x  = pose_data.velocity.x;
  vio_twist_linear_vect.vector.y  = pose_data.velocity.y;
  vio_twist_linear_vect.vector.z  = pose_data.velocity.z;
  vio_twist_angular_vect.vector.x = pose_data.angular_velocity.x;
  vio_twist_angular_vect.vector.y = pose_data.angular_velocity.y;
  vio_twist_angular_vect.vector.z = pose_data.angular_velocity.z;

  try {
    // POSE: Obtain mav pose in odom reference frame
    auto pose_transform = tf_buffer_->lookupTransform(odom_str,base_link_str,tf2::TimePointZero);
    odom_msg.pose.pose.position.x  = pose_transform.transform.translation.x;
    odom_msg.pose.pose.position.y  = pose_transform.transform.translation.y;
    odom_msg.pose.pose.position.z  = pose_transform.transform.translation.z;
    odom_msg.pose.pose.orientation = pose_transform.transform.rotation;

    // ANGULAR VELOCITY: Obtain mav angular velocity in base_link frame 
    auto angular_twist_transform_0 = tf_buffer_->lookupTransform("rs_link_stab",rs_link_str,tf2::TimePointZero);
    tf2::doTransform(vio_twist_angular_vect, rs_link_twist_angular_vect, angular_twist_transform_0);
    auto angular_twist_transform_1 = tf_buffer_->lookupTransform(base_link_str,"rs_odom",tf2::TimePointZero);
    tf2::doTransform(vio_twist_angular_vect, base_link_twist_angular_vect, angular_twist_transform_1);
    odom_msg.twist.twist.angular = base_link_twist_angular_vect.vector;

    // LINEAR VELOCITY: Obtain mav linear velocity in odom reference frame
    auto linear_twist_rs2od_tf = tf_buffer_->lookupTransform(odom_str,"rs_odom",tf2::TimePointZero);
    linear_twist_rs2od_tf.transform.translation.x = 0;
    linear_twist_rs2od_tf.transform.translation.y = 0;
    linear_twist_rs2od_tf.transform.translation.z = 0;
    tf2::doTransform(vio_twist_linear_vect, odom_twist_linear_vect, linear_twist_rs2od_tf);
    odom_msg.twist.twist.linear = odom_twist_linear_vect.vector;

    // TODO: Add velocity from rotation.
    // LINEAR VELOCITY: Obtain mav linear velocity in odom reference frame
    // From rs_link_stab to base_link
    // auto linear_twist_rs2bl_tf = tf_buffer_->lookupTransform(base_link_str,"rs_link_stab",tf2::TimePointZero);
    // linear_twist_rs2bl_tf.transform.translation.x = 0;
    // linear_twist_rs2bl_tf.transform.translation.y = 0;
    // linear_twist_rs2bl_tf.transform.translation.z = 0;
    // tf2::doTransform(vio_twist_linear_vect, base_link_vio_twist_linear_vect, linear_twist_rs2bl_tf);

    // From base_link to odom
    // auto linear_twist_bl2od_tf = tf_buffer_->lookupTransform(odom_str,base_link_str,tf2::TimePointZero);
    // linear_twist_bl2od_tf.transform.translation.x = 0;
    // linear_twist_bl2od_tf.transform.translation.y = 0;
    // linear_twist_bl2od_tf.transform.translation.z = 0;
    // tf2::doTransform(base_link_vio_twist_linear_vect, odom_twist_linear_vect, linear_twist_bl2od_tf);

    // // Eigen::Vector3d w_vector(vio_twist_angular_vect.vector); // w_rs
    // Eigen::Vector3d r_vector(-camera_offset_y_,-camera_offset_x_,-camera_offset_z_);
    // Eigen::Vector3d w_vector(base_link_twist_angular_vect.vector.x,base_link_twist_angular_vect.vector.y,base_link_twist_angular_vect.vector.z); // w_bl
    // Eigen::Vector3d linear_vel_from_rotation = w_vector.cross(r_vector);

    // base_link_sum_twist_linear_vect.vector.x = base_link_vio_twist_linear_vect.vector.x + linear_vel_from_rotation.x();
    // base_link_sum_twist_linear_vect.vector.y = base_link_vio_twist_linear_vect.vector.y + linear_vel_from_rotation.y();
    // base_link_sum_twist_linear_vect.vector.z = base_link_vio_twist_linear_vect.vector.z + linear_vel_from_rotation.z();

    // // From base_link to odom
    // auto linear_twist_bl2od_tf = tf_buffer_->lookupTransform(odom_str,base_link_str,tf2::TimePointZero);
    // tf2::doTransform(base_link_vio_twist_linear_vect, odom_twist_linear_vect, linear_twist_bl2od_tf);
    // odom_msg.twist.twist.linear = odom_twist_linear_vect.vector;

    // // odom_msg.twist.twist.linear = base_link_vio_twist_linear_vect.vector;
    // odom_msg.twist.twist.linear = base_link_sum_twist_linear_vect.vector;

    // double roll,pitch,yaw;
    // tf2::Quaternion q(pose_transform.transform.rotation.x,
    //                 pose_transform.transform.rotation.y,
    //                 pose_transform.transform.rotation.z,
    //                 pose_transform.transform.rotation.w);
    // tf2::Matrix3x3 m(q);
    // m.getEulerYPR(yaw,pitch,roll);
    // std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
  }
  catch (tf2::TransformException &ex) {
    std::cout << "Warning: Transform fail" << std::endl;
    RCLCPP_WARN(this->get_logger(),"Failure %s\n", ex.what()); //Print exception which was caught
  }

  odom_->updateData(odom_msg);

  return;
};

void RsT265Interface::setupTf()
{
  // Change from rs_link ENU to rs OWN.
  const float camera_roll  = 90.0f/180.0f *M_PI;
  // Change from rs_link STAB to base_link
  const float camera_pitch = 90.0f/180.0f *M_PI;
  const float camera_yaw   = 90.0f/180.0f *M_PI;

  //TODO: Read camera position from config file.
  tf2_fix_transforms_.clear();

  // tf2_fix_transforms_.emplace_back(tf_buffer_->lookupTransform("map","rs_link",tf2::TimePointZero);
  // drone reference to camera reference
  tf2_fix_transforms_.emplace_back(getTransformation(odom_str,"rs_odom",camera_offset_x_,camera_offset_y_,camera_offset_z_,camera_roll,camera_offset_pitch_,camera_offset_yaw_));
  // camera position to drone position
  tf2_fix_transforms_.emplace_back(getTransformation(rs_link_str,"rs_link_stab",0,0,0,-camera_offset_roll_,0,0));
  tf2_fix_transforms_.emplace_back(getTransformation("rs_link_stab",base_link_str,camera_offset_x_,-camera_offset_z_,-camera_offset_y_,0.0,-camera_pitch,-camera_yaw));

  // Odom_rs to rs_link_own
	rs_odom2rs_link_tf_.header.frame_id = "rs_odom";
	rs_odom2rs_link_tf_.child_frame_id  = rs_link_str;
	rs_odom2rs_link_tf_.transform.rotation.w = 1.0f;

  setTfTree();
  publishTFs();

  // REMOVE WHEN FINISH
  // Change from base_link ENU to base_link FLU.
  // const float flu_roll  = 0.0f *M_PI;
  // const float flu_pitch = 0.0f *M_PI;
  // const float flu_yaw   = 90.0f/180.0f *M_PI;
  // global reference to drone reference
  // tf2_fix_transforms_.emplace_back(getTransformation("map",odom_str,0,0,0,0,0,0));
  // drone reference to camera reference
  // tf2_fix_transforms_.emplace_back(getTransformation(odom_str,"rs_odom_enu",camera_offset_x_,camera_offset_y_,camera_offset_z_,0,0,0));
  // tf2_fix_transforms_.emplace_back(getTransformation("rs_odom_enu","rs_odom",0,0,0,camera_odom_roll,camera_offset_pitch_,camera_offset_yaw_));
  // camera position to drone position
  // tf2_fix_transforms_.emplace_back(getTransformation("rs_link","rs_link_stab",0,0,0,-camera_offset_roll_,0,0));
  // tf2_fix_transforms_.emplace_back(getTransformation("rs_link_stab","rs_link_enu",0,0,0,camera_odom_roll,camera_offset_pitch_,camera_offset_yaw_));
  // tf2_fix_transforms_.emplace_back(getTransformation("rs_link_enu","base_link_enu",-camera_offset_x_,-camera_offset_y_,-camera_offset_z_,0,0,0));
  // tf2_fix_transforms_.emplace_back(getTransformation("base_link_enu",base_link_str,0,0,0,flu_roll,flu_pitch,flu_yaw));

}

// void RsT265Interface::publishTFs(){

//     rclcpp::Time timestamp = this->get_clock()->now();
    
//     for (geometry_msgs::msg::TransformStamped& transform:tf2_fix_transforms_){
//         transform.header.stamp = timestamp;
//         tfstatic_broadcaster_->sendTransform(transform);
//     }
//     rs_odom2rs_link_tf_.header.stamp = timestamp;
//     tf_broadcaster_->sendTransform(rs_odom2rs_link_tf_);

// }

void RsT265Interface::setTfTree(){

  rclcpp::Time timestamp = this->get_clock()->now();
  const std::string authority="empty";
  
  for (geometry_msgs::msg::TransformStamped& transform:tf2_fix_transforms_){
    transform.header.stamp = timestamp;
    tf_buffer_->setTransform(transform, authority, true);
  }
  rs_odom2rs_link_tf_.header.stamp = timestamp;
  tf_buffer_->setTransform(rs_odom2rs_link_tf_, authority, false);

}


void RsT265Interface::publishTFs(){

  geometry_msgs::msg::TransformStamped transform;
  rclcpp::Time timestamp = this->get_clock()->now();
  std::string ns = this->get_namespace();

  // RCLCPP_INFO(this->get_logger(),ns);

  // transform = tf_buffer_->lookupTransform(odom_str,base_link_str,tf2::TimePointZero);
  // transform.header.stamp = timestamp;
  // tf_broadcaster_->sendTransform(transform);
  // TODO: clean
  try{
    transform = tf_buffer_->lookupTransform(base_link_str,rs_link_str,tf2::TimePointZero);
    transform.header.stamp = timestamp;
    tf_broadcaster_->sendTransform(transform);
  }
  catch (tf2::TransformException &ex) {
    std::cout << "Warning: Transform fail" << std::endl;
    RCLCPP_WARN(this->get_logger(),"Failure %s\n", ex.what()); //Print exception which was caught
  }

}

// TODO:Add IMU to realsense node. Choose between loop or callback.
// void RsT265Interface::setup_imu(){
//     // Check that a device supporting IMU is connected
//     if (!check_imu_is_supported())
//     {
//         std::cerr << "Device supporting IMU not found";
//         return;
//     }
//     // Create a configuration for configuring the pipeline with a non default profile
//     rs2::config cfg;
//     // Add streams of gyro and accelerometer to configuration
//     cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
//     cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
//     // Start streaming with the given configuration;
//     // Note that since we only allow IMU streams, only single frames are produced
//     pipe_.start(cfg);
// };

// void RsT265Interface::run_imu(){
//     std::cout << "waiting for imu frame" << std::endl;
//     // Cast the frame that arrived to motion frame
//     auto frame = pipe_.wait_for_frames();
//     std::cout << "frame" << frame << std::endl;
//     std::cout << "got imu frame" << std::endl;
//     auto motion = f.as<rs2::motion_frame>();
//     std::cout << motion.get_profile().stream_type() << std::endl;

//     // If casting succeeded and the arrived frame is from gyro stream
//     if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//     {
//         // Get gyro measures
//         rs2_vector gyro_data = motion.get_motion_data();
//         std::cout << "gyro " << gyro_data << std::endl;
//     }
//     // If casting succeeded and the arrived frame is from accelerometer stream
//     if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//     {
//         // Get accelerometer measures
//         rs2_vector accel_data = motion.get_motion_data();
//         std::cout << "accl " << accel_data << std::endl;
//     }

//     return;
// };

// Auxiliar functions
bool find_device_with_stream(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
  rs2::context ctx;
  auto devs = ctx.query_devices();
  std::vector <rs2_stream> unavailable_streams = stream_requests;
  for (auto dev : devs)
  {
    std::map<rs2_stream, bool> found_streams;
    for (auto& type : stream_requests)
    {
      found_streams[type] = false;
      for (auto& sensor : dev.query_sensors())
      {
        for (auto& profile : sensor.get_stream_profiles())
        {
          if (profile.stream_type() == type)
          {
            found_streams[type] = true;
            unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
            if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
              out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          }
        }
      }
    }
    // Check if all streams are found in current device
    bool found_all_streams = true;
    for (auto& stream : found_streams)
    {
      if (!stream.second)
      {
        found_all_streams = false;
        break;
      }
    }
    if (found_all_streams)
      return true;
  }
  // After scanning all devices, not all requested streams were found
  for (auto& type : unavailable_streams)
  {
    switch (type)
    {
    case RS2_STREAM_POSE:
    case RS2_STREAM_FISHEYE:
      std::cerr << "Realsense t265" << std::endl;
      break;
    case RS2_STREAM_DEPTH:
      std::cerr << "Realsense camera with DEPTH sensor" << std::endl;
      break;
    case RS2_STREAM_COLOR:
      std::cerr << "Realsense camera with RGB sensor" << std::endl;
      break;
    default:
      throw std::runtime_error("The requested stream: " + std::to_string(type) + ", is not supported by connected devices!"); // stream type
    }
  }
  return false;
}

bool check_imu_is_supported()
{
  bool found_gyro = false;
  bool found_accel = false;
  rs2::context ctx;
  for (auto dev : ctx.query_devices())
  {
    // The same device should support gyro and accel
    found_gyro = false;
    found_accel = false;
    for (auto sensor : dev.query_sensors())
    {
      for (auto profile : sensor.get_stream_profiles())
      {
        if (profile.stream_type() == RS2_STREAM_GYRO)
          found_gyro = true;

        if (profile.stream_type() == RS2_STREAM_ACCEL)
          found_accel = true;
      }
    }
    if (found_gyro && found_accel)
      break;
  }
  return found_gyro && found_accel;
}
