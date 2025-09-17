# Recursos do Nav2
O Nav2 oferece novos recursos e ferramentas que facilitam a criação de aplicações robóticas.

Nesta unidade, você revisará os novos recursos essenciais introduzidos no Nav2, que incluem:

* Operações básicas do Nav2 por meio da **API Simple Commander**
* Uso dos plugins **Waypoint Follower** e **Task Executor** via **FollowWaypoints**
* Introdução às **Zonas de Exclusão** e **zonas de velocidade restrita**

Em seguida, você criará uma demonstração básica de robótica autônoma baseada no Nav2. Você fará isso em um armazém simulado onde robôs são frequentemente utilizados.

Você usará o AWS [Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) e o robô industrial móvel [MP-400](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-400) da Neobotix.

## A API Simple Commander
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

## Navigate To Pose
A ação NavigateToPose é mais adequada para solicitações de navegação de um ponto para outro ponto ou outras tarefas que podem ser representadas em uma árvore de comportamento (behavior tree) com uma pose de condição de contorno, como o acompanhamento dinâmico de objetos.

Veja abaixo a estrutura da ação NavigateToPose (NavigateToPose.action):

```txt
#goal definition
geometry_msgs/PoseStamped pose
string behavior_tree
---
#result definition
std_msgs/Empty result
---
# feedback definition
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

Como você pode ver, as entradas principais da ação são a pose para a qual você deseja que o robô navegue e a behavior_tree (opcional) a ser usada. Ela usa a árvore de comportamento padrão do BT Navigator, caso nenhuma seja especificada. Durante a execução da ação, você recebe feedback com informações essenciais, como:
* a pose do robô,
* o tempo decorrido,
* o tempo estimado restante,
* a distância restante
* e o número de recuperações executadas durante a navegação até o objetivo. 
Essas informações podem ser usadas para tomar boas decisões de autonomia ou monitorar o progresso.

### Demo
Na demonstração a seguir, você usa a ação NavigateToPose para fazer seu robô se deslocar do ponto de parada até uma prateleira para que um humano coloque um item nele. Em seguida, você se deslocará até a transpaleteira para embarque no próximo caminhão a sair do armazém. Para isso crie um pacote chamado **nav2_new_features**.

Na pasta scripts tem o código [**navigate_to_pose.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_new_features/scripts/navigate_to_pose.py).

Vamos analisar o código. Preste atenção especial à parte em que você usa a API. Comece do início:

```python
navigator = BasicNavigator()
```

Claro, você também precisa instanciar a classe para usá-la.

```python
navigator.setInitialPose(initial_pose)
```

Aqui, você está usando o método **setInitialPose()**, que definirá a pose inicial do robô (o mesmo que com a ferramenta Estimativa de Pose 2D do RVIZ2). Neste caso, você está especificando a pose inicial na variável initial_pose (ela deve ser do tipo PoseStamped).

```python
navigator.waitUntilNav2Active()
```

O método **waitUntilNav2Active()** bloqueia a execução do programa até que o Nav2 esteja totalmente online e os nós do ciclo de vida estejam no estado ativo.

```python
navigator.goToPose(shelf_item_pose)
```

Aqui, você está usando o método **goToPose()**, que solicita que o robô navegue até a pose especificada. Neste caso, você está especificando a pose na variável shelf_item_pose (ela deve ser do tipo PoseStamped).

```python
while not navigator.isTaskComplete():
```

O método **isTaskComplete()** retornará True somente após o robô atingir o objetivo. Ele retornará False enquanto ainda estiver em andamento.

```python
feedback = navigator.getFeedback()
```

O método **getFeedback()** retorna o feedback do servidor de ação NavigateToPose.

```python
result = navigator.getResult()
```

O método **getResult()** retorna o resultado do servidor de ação NavigateToPose.

Você pode revisar todos os métodos do Nav2 Simple Commander [aqui](https://docs.nav2.org/commander_api/index.html).

## Navigate Through Poses
A ação NavigateThroughPoses é mais adequada para solicitações de navegação com restrição de pose ou outras tarefas representadas em uma árvore de comportamento com um conjunto de poses. Ela NÃO para em cada ponto de referência, mas percorre cada um deles como uma restrição de pose.

NavigateThroughPoses.action:

```txt
#goal definition
geometry_msgs/PoseStamped[] poses
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
int16 number_of_poses_remaining
```

Como você pode ver, as entradas da ação são quase idênticas às de NavigateToPose, exceto que agora você recebe um vetor de poses. O feedback também é semelhante, contendo apenas o novo campo **number_of_poses_remaining** para acompanhar o progresso pelos pontos de passagem.

### Demo
Nesta demonstração, use a ação NavigateThroughPoses para que seu robô se desloque do seu ponto de parada por todo o armazém seguindo uma rota conhecida. A ação NavigateThroughPoses, assim como NavigateToPose, **pode desviar na presença de obstáculos**, como você verá com a transpaleteira nesta demonstração. Assim que a rota for concluída, ela recomeça e continua até parar. Veja o código criado nesse script [**navigate_through_poses.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_new_features/scripts/navigate_through_poses.py).

Como você pode ver, o código é semelhante ao primeiro script, com algumas modificações.

```python
security_route = [
        [1.792, 2.144],
        [1.792, -5.44],
        [1.792, -9.427],
        [-3.665, -9.427],
        [-3.665, -4.303],
        [-3.665, 2.330],
        [-3.665, 9.283]]
```

Primeiro, defina um conjunto de poses que você quer que o robô execute.

```python
for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        route_poses.append(deepcopy(pose))
```

Adicione essas poses a um vetor route_poses. Observe que cada pose deve ser definida como uma mensagem PoseStamped.

```python
navigator.goThroughPoses(route_poses)
```

Em seguida, use esse vetor como entrada para chamar o método **goThroughPoses()**. Este método solicita que o robô navegue por um conjunto de poses.

```python
if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                print('Navigation has exceeded timeout of 180s, canceling the request.')
                navigator.cancelTask()
```

Observe que, neste script, você também está adicionando um tempo limite. Se a tarefa de navegação demorar mais de 180 segundos, você cancelará a tarefa atual usando o método cancelTask().

### Waypoint Following
A ação FollowWaypoints é mais adequada para tarefas de autonomia simples, nas quais você deseja parar em cada ponto de referência e executar um comportamento (por exemplo, pausar por 2 segundos, tirar uma foto, esperar que alguém coloque uma caixa sobre ele, etc.). O servidor  waypoint follower de referência Nav2 contém plugins TaskExecutor para executar uma tarefa em cada ponto de referência.



```python

```


```python

```


```python

```