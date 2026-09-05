#include "diablo_imu.hpp"

#include <cmath>

using namespace std::chrono;

void diablo_imu_publisher::imu_pub_init(void)
{
    imu_frame_id_ = this->node_ptr->declare_parameter<std::string>(
        "imu_frame_id", "diablo_robot");
    const double configured_orientation_variance = this->node_ptr->declare_parameter<double>(
        "imu_orientation_variance", 0.0025);
    const double configured_gyro_variance = this->node_ptr->declare_parameter<double>(
        "imu_gyro_variance", 0.0004);
    const double orientation_variance =
        std::isfinite(configured_orientation_variance) &&
        configured_orientation_variance > 0.0 ? configured_orientation_variance : 0.0025;
    const double gyro_variance =
        std::isfinite(configured_gyro_variance) && configured_gyro_variance > 0.0 ?
        configured_gyro_variance : 0.0004;

    imu_Publisher_ = this->node_ptr->create_publisher<sensor_msgs::msg::Imu>("diablo/sensor/Imu",10);
    euler_Publisher_ = this->node_ptr->create_publisher<ception_msgs::msg::IMUEuler>("diablo/sensor/ImuEuler",10);
    timer_ = this->node_ptr->create_wall_timer(20ms,std::bind(&diablo_imu_publisher::lazyPublisher, this));

    // The original driver left all covariance arrays at zero.  Supply
    // conservative estimates for orientation/gyro and explicitly mark the
    // acceleration estimate unavailable until its SDK unit is calibrated.
    imu_msg_.orientation_covariance.fill(0.0);
    imu_msg_.orientation_covariance[0] = orientation_variance;
    imu_msg_.orientation_covariance[4] = orientation_variance;
    imu_msg_.orientation_covariance[8] = orientation_variance;
    imu_msg_.angular_velocity_covariance.fill(0.0);
    imu_msg_.angular_velocity_covariance[0] = gyro_variance;
    imu_msg_.angular_velocity_covariance[4] = gyro_variance;
    imu_msg_.angular_velocity_covariance[8] = gyro_variance;
    imu_msg_.linear_acceleration_covariance.fill(0.0);
    imu_msg_.linear_acceleration_covariance[0] = -1.0;

    if (orientation_variance != configured_orientation_variance ||
        gyro_variance != configured_gyro_variance) {
        RCLCPP_WARN(
            this->node_ptr->get_logger(),
            "IMU covariance parameters must be positive finite values; "
            "check imu_orientation_variance and imu_gyro_variance");
    }

    RCLCPP_INFO(
        this->node_ptr->get_logger(),
        "IMU frame: %s; orientation variance=%.6g, gyro variance=%.6g; "
        "linear acceleration covariance disabled until units are verified",
        imu_frame_id_.c_str(), orientation_variance, gyro_variance);
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
    this->vehicle->telemetry->configUpdate(); 
}


void diablo_imu_publisher::lazyPublisher(void){
    if(imu_Publisher_->get_subscription_count() > 0 || euler_Publisher_->get_subscription_count() > 0)
    {
        bool imu_Pub_mark = false;
        if(this->vehicle->telemetry->newcome & 0x10)
        {
            imu_Pub_mark = true;
            imu_msg_.linear_acceleration.x = this->vehicle->telemetry-> accl.x;
            imu_msg_.linear_acceleration.y = this->vehicle->telemetry-> accl.y;
            imu_msg_.linear_acceleration.z = this->vehicle->telemetry-> accl.z;
            this->vehicle->telemetry->eraseNewcomeFlag(0xEF);
        }
        
        if(this->vehicle->telemetry->newcome & 0x08)
        {
            imu_Pub_mark = true;
            imu_msg_.angular_velocity.x = this->vehicle->telemetry->gyro.x;
            imu_msg_.angular_velocity.y = this->vehicle->telemetry->gyro.y;
            imu_msg_.angular_velocity.z = this->vehicle->telemetry->gyro.z;
            this->vehicle->telemetry->eraseNewcomeFlag(0xF7);
        }
        if(this->vehicle->telemetry->newcome & 0x20)
        {
            imu_Pub_mark = true;
            imu_msg_.orientation.w = this->vehicle->telemetry->quaternion.w;
            imu_msg_.orientation.x = this->vehicle->telemetry->quaternion.x;
            imu_msg_.orientation.y = this->vehicle->telemetry->quaternion.y;
            imu_msg_.orientation.z = this->vehicle->telemetry->quaternion.z;

            // roll (x-axis rotation)
            double sinr_cosp = +2.0 * (imu_msg_.orientation.w * imu_msg_.orientation.x + imu_msg_.orientation.y * imu_msg_.orientation.z);
            double cosr_cosp = +1.0 - 2.0 * (imu_msg_.orientation.x * imu_msg_.orientation.x + imu_msg_.orientation.y * imu_msg_.orientation.y);
            euler_msg_.roll = atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = +2.0 * (imu_msg_.orientation.w * imu_msg_.orientation.y - imu_msg_.orientation.z * imu_msg_.orientation.x);
            if (fabs(sinp) >= 1){
                euler_msg_.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            }
            else{
                euler_msg_.pitch = asin(sinp);
            }

            // yaw (z-axis rotation)
            double siny_cosp = +2.0 * (imu_msg_.orientation.w * imu_msg_.orientation.z + imu_msg_.orientation.x * imu_msg_.orientation.y);
            double cosy_cosp = +1.0 - 2.0 * (imu_msg_.orientation.y * imu_msg_.orientation.y + imu_msg_.orientation.z * imu_msg_.orientation.z);
            euler_msg_.yaw = atan2(siny_cosp, cosy_cosp);
            
            this->vehicle->telemetry->eraseNewcomeFlag(0xDF);
        }
        if(imu_Pub_mark){

            imu_timestamp = this->node_ptr->get_clock()->now();
            imu_msg_.header.stamp = imu_timestamp;
            imu_msg_.header.frame_id = imu_frame_id_;

            euler_msg_.header.stamp = imu_timestamp;
            euler_msg_.header.frame_id = imu_frame_id_;
            imu_Publisher_->publish(imu_msg_);
            euler_Publisher_->publish(euler_msg_);

            imu_Pub_mark = false;
        }
    }
}

diablo_imu_publisher::diablo_imu_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle)
{
    this->node_ptr = node_ptr;
    this->vehicle = vehicle;

}
