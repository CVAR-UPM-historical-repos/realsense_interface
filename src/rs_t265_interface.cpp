#include "rs_t265_interface.hpp"

geometry_msgs::msg::TransformStamped getTransformation(	const std::string& _frame_id,
												  	const std::string& _child_frame_id,
													double _translation_x,
													double _translation_y,
													double _translation_z,
													double _roll,
													double _pitch,
													double _yaw  ){

geometry_msgs::msg::TransformStamped transformation;

	transformation.header.frame_id = _frame_id;
	transformation.child_frame_id  = _child_frame_id;
	transformation.transform.translation.x = _translation_x;
	transformation.transform.translation.y = _translation_y;
	transformation.transform.translation.z = _translation_z;
	tf2::Quaternion q;
	q.setRPY(_roll, _pitch, _yaw);
	transformation.transform.rotation.x = q.x();
	transformation.transform.rotation.y = q.y();
	transformation.transform.rotation.z = q.z();
	transformation.transform.rotation.w = q.w();
	
	return transformation;

}

RsT265Interface::RsT265Interface():aerostack2::Node("rs_t265"){
    odom_       = std::make_shared<aerostack2::Sensor<nav_msgs::msg::Odometry>>("odom",this);
    imu_sensor_ = std::make_shared<aerostack2::Sensor<sensor_msgs::msg::Imu>>("imu",this);
    // Initialize the transform broadcaster
    tf_broadcaster_       = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_            = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_          = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
};

void RsT265Interface::setupOdom(){
    if (!find_device_with_stream({ RS2_STREAM_POSE}, serial_))
    {
        RCLCPP_ERROR(this->get_logger(),"Device with stream for pose not found");
        EXIT_SUCCESS;
    }
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial_.empty())
        cfg.enable_device(serial_);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe_.start(cfg);
};

void RsT265Interface::runOdom(){
    
    // Wait for the next set of frames from the camera
    auto frames = pipe_.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

    std::string frame_id = "rs_odom_own";
    std::string child_frame_id = "rs_link_own";
    rclcpp::Time timestamp = this->get_clock()->now();

    // TF
    odom_rs_to_base_link_rs_transform_.header.frame_id = frame_id;
    odom_rs_to_base_link_rs_transform_.child_frame_id  = child_frame_id;
    odom_rs_to_base_link_rs_transform_.header.stamp    = timestamp;
    odom_rs_to_base_link_rs_transform_.transform.translation.x = pose_data.translation.x;
    odom_rs_to_base_link_rs_transform_.transform.translation.y = pose_data.translation.y;
    odom_rs_to_base_link_rs_transform_.transform.translation.z = pose_data.translation.z;
    odom_rs_to_base_link_rs_transform_.transform.rotation.x = pose_data.rotation.x;
    odom_rs_to_base_link_rs_transform_.transform.rotation.y = pose_data.rotation.y;
    odom_rs_to_base_link_rs_transform_.transform.rotation.z = pose_data.rotation.z;
    odom_rs_to_base_link_rs_transform_.transform.rotation.w = pose_data.rotation.w;

    publishTFs();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";
    odom_msg.header.stamp    = timestamp;

    geometry_msgs::msg::Vector3Stamped vio_twist_linear_vect;
	geometry_msgs::msg::Vector3Stamped vio_twist_angular_vect ;

    geometry_msgs::msg::Vector3Stamped base_link_vio_twist_linear_vect;
    geometry_msgs::msg::Vector3Stamped base_link_sum_twist_linear_vect;
    geometry_msgs::msg::Vector3Stamped odom_twist_linear_vect;
    geometry_msgs::msg::Vector3Stamped base_link_twist_angular_vect;

    vio_twist_linear_vect.vector.x  = pose_data.velocity.x;
    vio_twist_linear_vect.vector.y  = pose_data.velocity.y;
    vio_twist_linear_vect.vector.z  = pose_data.velocity.z;
    vio_twist_angular_vect.vector.x = pose_data.angular_velocity.x;
    vio_twist_angular_vect.vector.y = pose_data.angular_velocity.y;
    vio_twist_angular_vect.vector.z = pose_data.angular_velocity.z;

    try {
        // Obtain mav pose in odom reference frame
        auto pose_transform = tf_buffer_->lookupTransform("odom","base_link",tf2::TimePointZero);
        odom_msg.pose.pose.position.x  = pose_transform.transform.translation.x;
        odom_msg.pose.pose.position.y  = pose_transform.transform.translation.y;
        odom_msg.pose.pose.position.z  = pose_transform.transform.translation.z;
        odom_msg.pose.pose.orientation = pose_transform.transform.rotation;

        // Obtain mav angular speed in base_link frame 
        auto angular_twist_transform = tf_buffer_->lookupTransform("base_link","rs_link_own",tf2::TimePointZero);
        tf2::doTransform(vio_twist_angular_vect, base_link_twist_angular_vect, angular_twist_transform);
        odom_msg.twist.twist.angular = base_link_twist_angular_vect.vector;

        // const float camera_offset_roll = -45.0f/180.0f *M_PI;
        const float camera_offset_x = 0.0f;
        const float camera_offset_y = -0.11f;
        const float camera_offset_z = 0.01f;

        // TODO: Review this
        // Obtain mav linear speed in odom reference frame
        // From rs_link_own to base_link
        auto linear_twist_rs2bl_tf = tf_buffer_->lookupTransform("base_link","rs_link_own",tf2::TimePointZero);
        tf2::doTransform(vio_twist_linear_vect, base_link_vio_twist_linear_vect, linear_twist_rs2bl_tf);

        // Eigen::Vector3d w_vector(vio_twist_angular_vect.vector); // w_rs
        Eigen::Vector3d r_vector(-camera_offset_y,-camera_offset_x,-camera_offset_z);
        Eigen::Vector3d w_vector(base_link_twist_angular_vect.vector.x,base_link_twist_angular_vect.vector.y,base_link_twist_angular_vect.vector.z); // w_bl
        Eigen::Vector3d linear_vel_from_rotation = w_vector.cross(r_vector);

        base_link_sum_twist_linear_vect.vector.x = base_link_vio_twist_linear_vect.vector.x + linear_vel_from_rotation.x();
        base_link_sum_twist_linear_vect.vector.y = base_link_vio_twist_linear_vect.vector.y + linear_vel_from_rotation.y();
        base_link_sum_twist_linear_vect.vector.z = base_link_vio_twist_linear_vect.vector.z + linear_vel_from_rotation.z();

        // From base_link to odom
        auto linear_twist_bl2od_tf = tf_buffer_->lookupTransform("odom","base_link",tf2::TimePointZero);
        tf2::doTransform(base_link_sum_twist_linear_vect, odom_twist_linear_vect, linear_twist_bl2od_tf);
        odom_msg.twist.twist.linear = odom_twist_linear_vect.vector;

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

    odom_->publishData(odom_msg);

    return;
};

void RsT265Interface::setupTf()
{
    // Change ref system from standard camera frame to Realsense frame.
    const float rs_roll  = 180.0f/180.0f *M_PI;
    const float rs_pitch =   0.0f *M_PI;
    const float rs_yaw   =   0.0f *M_PI;
    // Change ref system from ENU to standard camera frame.
    const float cam_roll  = -90.0f/180.0f *M_PI;
    const float cam_pitch =   0.0f/180.0f *M_PI;
    const float cam_yaw   =   0.0f/180.0f *M_PI;

    // Change from frame ENU to frame FLU.
    const float flu_roll  = 0.0f *M_PI;
    const float flu_pitch = 0.0f *M_PI;
    const float flu_yaw   = 90.0f/180.0f *M_PI;

    //TODO: Read camera position from config file.
	tf2_fix_transforms_.clear();

    // global reference to drone reference
    tf2_fix_transforms_.emplace_back(getTransformation("map","odom",0,0,0,0,0,0));
    
    // drone reference to camera reference
    tf2_fix_transforms_.emplace_back(getTransformation("odom","rs_odom_enu",camera_offset_x_,camera_offset_y_,camera_offset_z_,0,0,0));
    tf2_fix_transforms_.emplace_back(getTransformation("rs_odom_enu","rs_odom",0,0,0,0,camera_offset_pitch_,camera_offset_yaw_));

    // odom frame adaptation to camera frame
    tf2_fix_transforms_.emplace_back(getTransformation("rs_odom","rs_odom_cam",0,0,0,cam_roll,cam_pitch,cam_yaw));
    tf2_fix_transforms_.emplace_back(getTransformation("rs_odom_cam","rs_odom_own",0,0,0,rs_roll,rs_pitch,rs_yaw));
    
    // camera frames adaptation
    tf2_fix_transforms_.emplace_back(getTransformation("rs_link_own","rs_link_cam",0,0,0,rs_roll,rs_pitch,rs_yaw));
    tf2_fix_transforms_.emplace_back(getTransformation("rs_link_cam","rs_link",0,0,0,-cam_roll,cam_pitch,cam_yaw));
    
    // camera position to drone position
    tf2_fix_transforms_.emplace_back(getTransformation("rs_link","rs_link_enu",0,0,0,camera_offset_roll_,camera_offset_pitch_,camera_offset_yaw_));
    tf2_fix_transforms_.emplace_back(getTransformation("rs_link_enu","base_link_enu",-camera_offset_x_,-camera_offset_y_,-camera_offset_z_,0,0,0));
    tf2_fix_transforms_.emplace_back(getTransformation("base_link_enu","base_link",0,0,0,flu_roll,flu_pitch,flu_yaw));

    // Odom_rs to rs_link_own
	odom_rs_to_base_link_rs_transform_.header.frame_id = "rs_odom_own";
	odom_rs_to_base_link_rs_transform_.child_frame_id  = "rs_link_own";
	odom_rs_to_base_link_rs_transform_.transform.rotation.w = 1.0f;

    publishTFs();

}

void RsT265Interface::publishTFs(){
    rclcpp::Time timestamp = this->get_clock()->now();
    
    for (geometry_msgs::msg::TransformStamped& transform:tf2_fix_transforms_){
        transform.header.stamp = timestamp;
        tfstatic_broadcaster_->sendTransform(transform);
    }
    odom_rs_to_base_link_rs_transform_.header.stamp = timestamp;
    tf_broadcaster_->sendTransform(odom_rs_to_base_link_rs_transform_);

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
            std::cerr << "Connect T26X" << std::endl;
            break;
        case RS2_STREAM_DEPTH:
            std::cerr << "Connect Realsense camera with DEPTH sensor" << std::endl;
            break;
        case RS2_STREAM_COLOR:
            std::cerr << "Connect Realsense camera with RGB sensor" << std::endl;
            break;
        default:
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
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
