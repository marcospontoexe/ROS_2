# Nav2
O Nav2 oferece novos recursos e ferramentas que facilitam a criação de aplicações robóticas.

Nesta unidade, você revisará os novos recursos essenciais introduzidos no Nav2, que incluem:

* Operações básicas do Nav2 por meio da **API Simple Commander**
* Uso dos plugins **Waypoint Follower** e **Task Executor** via **FollowWaypoints**
* Introdução às **Zonas de Exclusão** e **zonas de velocidade restrita**

Em seguida, você criará uma demonstração básica de robótica autônoma baseada no Nav2. Você fará isso em um armazém simulado onde robôs são frequentemente utilizados.

Você usará o AWS [Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) e o robô industrial móvel [MP-400](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-400) da Neobotix.

# A API Simple Commander
O Nav2 Simple Commander é uma API Python que simplifica a interação com o sistema Nav2, permitindo controlar e gerenciar a navegação do robô por meio de código Python3. Ele fornece uma interface de alto nível para executar tarefas de navegação sem a necessidade de interagir diretamente com mensagens e serviços ROS 2 de nível inferior.

Nesta unidade, você explorará métodos essenciais para a construção de uma aplicação de robô de navegação, incluindo **goToPose()** para se mover para um local específico, **goThroughPoses()** para seguir uma sequência de poses e **followWaypoints()** para navegar por pontos de referência predefinidos. A compreensão dessas funções permitirá que você desenvolva comportamentos de navegação mais flexíveis e autônomos.

A Simple Commander (nav2_simple_commander) é **exclusivamente em Python**. Não existe versão oficial em **C++** no Navigation2.

**Motivo**

* Ela foi pensada como uma **API de alto nível e simples**, voltada a scripts e prototipagem rápida, onde Python é mais ágil.
* No C++, já existe acesso direto e relativamente estruturado às **actions** e **services** do Nav2, então o time do projeto considerou que não era necessário criar uma camada simplificada.

**Alternativas em C++:**

Se você quiser algo parecido em C++, tem que implementar manualmente usando os **Action Clients** do ROS 2:

* Para enviar um destino:
  * Usar `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`
* Para calcular caminho:
  * Usar `rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>`
* Para controlar mapa e localização:
  * Chamar services como `/clear_costmaps`, `/get_map`, `/localization_pose`, etc.

Ou seja, você mesmo precisa criar a sua própria versão de um "Simple Commander" em C++ se quiser essa simplicidade.

## 