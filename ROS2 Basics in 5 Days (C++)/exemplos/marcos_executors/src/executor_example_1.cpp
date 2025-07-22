#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // Instantiate a node
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("executor_example_1_node");

  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, making bacon pancakes");

  /*
  objeto Executor responsável pela execução de Callbacks para um ou mais Nós.
  Em um objeto SingleThreadedExecutor, todos os Callbacks no Nó são executados
  na mesma thread.
  */
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);      // adiciona o nó ao executor
  executor.spin();  // executa um Single-threaded Executor chamando seu método spin().

  rclcpp::shutdown();
  return 0;
}