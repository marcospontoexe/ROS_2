#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

/*
permite usar _1 no std::bind, referenciando o primeiro argumento passado ao callback.
*/
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:

  SimpleSubscriber()  // Declara uma classe SimpleSubscriber que herda de rclcpp::Node, ou seja, é um nó ROS 2
  : Node("simple_subscriber")   //  chama o construtor da classe base Node, definindo o nome do nó como "simple_subscriber"
  {
    /*
    cria um objeto de subscrição:
    Tipo da mensagem: std_msgs::msg::Int32
    Tópico: "counter"
    Fila: tamanho 10
    Callback: resultado de std::bind(&SimpleSubscriber::topic_callback, this, _1), que liga o método da classe como callback, passando _1 como lugar do argumento msg
    */
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "counter", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)   // é chamado sempre que uma nova mensagem chega no tópico "counter"
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);  // registra a mensagem no console
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;  // variável que mantém a subscrição ativa
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //  inicializa o ambiente ROS 2.
  /*
  std::make_shared<SimpleSubscriber>(): cria uma instância do nó.
  rclcpp::spin(...): mantém o nó rodando e processando callbacks.
  */
  rclcpp::spin(std::make_shared<SimpleSubscriber>()); 
  rclcpp::shutdown(); // finaliza o ROS 2 antes de sair do programa.
  return 0;
}