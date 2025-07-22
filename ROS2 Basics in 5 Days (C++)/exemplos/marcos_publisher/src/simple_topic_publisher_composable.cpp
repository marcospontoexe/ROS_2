#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePublisher : public rclcpp::Node {
public:
  /*
  Node("simple_publisher"): Inicializa o nó ROS 2 com o nome "simple_publisher"
  (herdando de rclcpp::Node). count_(0): Inicializa a variável membro count_ com
  0.
  */
  SimplePublisher() : Node("simple_publisher"), count_(0) {
    /*
    Cria um publisher que envia mensagens do tipo std_msgs::msg::Int32 no tópico
    "counter", com uma fila de tamanho (QoS depth) igual a 10.
    */
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    /*
    Agenda um timer que chama o método timer_callback() a cada 500 milissegundos
    (2 Hz).
    */
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePublisher::timer_callback, this));
    /*
    O std::bind é usado para transformar o método membro timer_callback (que
    precisa de um this) em uma função sem parâmetros, compatível com o callback
    esperado por create_wall_timer()
    */
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv); //  inicializa o ambiente ROS 2.
  /*
  std::make_shared<SimplePublisher>(): cria uma instância do nó.
  rclcpp::spin(...): mantém o nó rodando.
  */
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown(); // finaliza o ROS 2 antes de sair do programa.
  return 0;
}