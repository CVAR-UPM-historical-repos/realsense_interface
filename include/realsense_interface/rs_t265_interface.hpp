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


bool find_device_with_stream(std::vector <rs2_stream> stream_requests, std::string& out_serial);
bool check_imu_is_supported();

class RsT265Interface:public as2::Node{
    public:
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
        void setupOdom();
        void stopOdom();
        void runOdom();

        void setupTf();
        void publishTFs();
        void setTfTree();
        
        void publishEstimatedPose();


    private:
        // Sensor comm
        std::string   serial_;
        rs2::pipeline pipe_;
        // Sensor measurement
        std::shared_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_;
        std::shared_ptr<as2::sensors::Imu> imu_sensor_;
        // Sensor Tf
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
        // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        
        std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
        geometry_msgs::msg::TransformStamped rs_odom2rs_link_tf_;
        // geometry_msgs::msg::TransformStamped rs_odom2rs_link_vel_tf_;
        std::string base_link_str;
        std::string rs_link_str;
        std::string odom_str;


        // Camera offsets from base_link frame ENU
        const float camera_offset_x_ =  0.0f;
        const float camera_offset_y_ = -0.11f;
        const float camera_offset_z_ =  0.01f;
        const float camera_offset_roll_  = -47.0f/180.0f *M_PI;
        const float camera_offset_pitch_ =   0.0f;
        const float camera_offset_yaw_   = 180.0f/180.0f *M_PI;

};

#endif // RS_T265_INTERFACE_HPP_