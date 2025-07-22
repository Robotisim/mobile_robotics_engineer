#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>
#include <std_msgs/msg/int32.h>

// micro-ROS core handles
rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Int32 my_msg;
int count = 0;

void setup() {
  // 1. Set up the micro-ROS transport
  set_microros_serial_transports(Serial);

  // 2. micro-ROS initialization
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // 3. Create a node
  rclc_node_init_default(&node, "int_publisher_node", "", &support);

  // 4. Create a publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/int_data"  // Topic name
  );
}
void loop() {

  my_msg.data = count++;
  rcl_ret_t ret = rcl_publish(&publisher, &my_msg, NULL);

}
