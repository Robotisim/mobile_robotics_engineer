#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdomGenerator : public rclcpp::Node {
public:
  OdomGenerator() : Node("odom_node"), x_(0.0), y_(0.0), theta_(0.0) {
    encoder_left_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/encoder_left", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          left_pulses_ = msg->data;
        });
    encoder_right_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/encoder_right", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          right_pulses_ = msg->data;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          imu_ = *msg;
        });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomGenerator::update_odom, this));
  }

private:
void update_odom() {
    const double dt = 0.05;
    const double wheel_diameter = 0.065;
    const double wheel_circ = M_PI * wheel_diameter;
    const double ticks_per_rev = 380.0;
    const double distance_per_tick = wheel_circ / ticks_per_rev;
    const double wheel_base = 0.165;

    double left_dist = (left_pulses_ - last_left_) * distance_per_tick;
    double right_dist = (right_pulses_ - last_right_) * distance_per_tick;

    // Debug print
    RCLCPP_INFO(this->get_logger(), "Left Ticks: %d, Right Ticks: %d, Left Dist: %f, Right Dist: %f",
                left_pulses_, right_pulses_, left_dist, right_dist);

    last_left_ = left_pulses_;
    last_right_ = right_pulses_;

    double lin_vel = (left_dist + right_dist) / (2.0 * dt);
    double ang_vel = (right_dist - left_dist) / (wheel_base * dt);

    theta_ += ang_vel * dt;
    x_ += lin_vel * cos(theta_) * dt;
    y_ += lin_vel * sin(theta_) * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = lin_vel;
    odom.twist.twist.angular.z = ang_vel;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_left_sub_, encoder_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_, y_, theta_;
  int32_t left_pulses_ = 0, right_pulses_ = 0, last_left_ = 0, last_right_ = 0;
  sensor_msgs::msg::Imu imu_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomGenerator>());
  rclcpp::shutdown();
  return 0;
}