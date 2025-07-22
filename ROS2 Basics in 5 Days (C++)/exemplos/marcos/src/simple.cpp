// Import the rclcpp client library
// A biblioteca core ROS client (RCL) implementa a funcionalidade padrão necessária para diversas APIs ROS. 
// Isso facilita a criação de bibliotecas cliente específicas para cada linguagem.
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  // This program creates an endless loop that repeats itself 2 times per second
  // (2Hz) until somebody presses Ctrl + C in the Shell

  rclcpp::init(argc, argv); // Initialize the ROS2 communication
  auto node =
      rclcpp::Node::make_shared("ObiWan"); // Create a ROS2 node named ObiWan

  rclcpp::WallRate loop_rate(2); // We create a Rate object of 2Hz

  // Endless loop until Ctrl + C
  while (rclcpp::ok()) {
    // Print a message to the terminal
    RCLCPP_INFO(node->get_logger(),
                "Help me Obi-Wan Kenobi, you're my only hope");
    rclcpp::spin_some(node);  // mantém o nó rodando e processando callbacks.
    // We sleep the needed time to maintain the Rate fixed above
    loop_rate.sleep();
  }
  rclcpp::shutdown(); // Shutdown the ROS2 communication
  return 0;
}
