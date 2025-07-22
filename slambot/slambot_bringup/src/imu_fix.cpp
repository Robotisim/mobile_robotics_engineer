#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class ImuFixerNode : public rclcpp::Node
{
public:
    ImuFixerNode() : Node("imu_fixer_node")
    {
        // Publisher
        imu_fixed_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_fixed", 10);

        // Subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuFixerNode::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Fixer Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_fixed_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto fixed_msg = *msg;

        // Fix timestamp
        fixed_msg.header.stamp = this->now();

        // Fix frame_id
        fixed_msg.header.frame_id = "imu_link";

        // Orientation covariance (still unknown)
        fixed_msg.orientation_covariance[0] = -1;

        // Angular velocity covariance
        fixed_msg.angular_velocity_covariance[0] = 0.01;
        fixed_msg.angular_velocity_covariance[4] = 0.01;
        fixed_msg.angular_velocity_covariance[8] = 0.01;

        // Linear acceleration covariance
        fixed_msg.linear_acceleration_covariance[0] = 0.1;
        fixed_msg.linear_acceleration_covariance[4] = 0.1;
        fixed_msg.linear_acceleration_covariance[8] = 0.1;

        // ------------------------------
        // FIX AXIS ORIENTATION
        // ------------------------------

        // Fix angular velocity (invert X, Y, Z)
        fixed_msg.angular_velocity.x = -msg->angular_velocity.x;
        fixed_msg.angular_velocity.y = -msg->angular_velocity.y;
        fixed_msg.angular_velocity.z = -msg->angular_velocity.z;

        // Fix linear acceleration (invert X, Y, Z)
        fixed_msg.linear_acceleration.x = -msg->linear_acceleration.x;
        fixed_msg.linear_acceleration.y = -msg->linear_acceleration.y;
        fixed_msg.linear_acceleration.z = -msg->linear_acceleration.z;

        // Publish fixed IMU message
        imu_fixed_pub_->publish(fixed_msg);
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFixerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
