#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);   //  inicializa o ambiente ROS 2.
  auto node = rclcpp::Node::make_shared("simple_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  auto message = std::make_shared<std_msgs::msg::Int32>();
  message->data = 0;
  rclcpp::WallRate loop_rate(2);

  while (rclcpp::ok()) {
    
    publisher->publish(*message);
    message->data++;
    rclcpp::spin_some(node);
    loop_rate.sleep();  // finaliza o ROS 2 antes de sair do programa.
  }
  rclcpp::shutdown();
  return 0;
}