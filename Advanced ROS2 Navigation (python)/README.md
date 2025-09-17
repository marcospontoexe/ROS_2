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
Permite enviar várias poses em sequência (ex.: P1 → P2 → P3). O planejador global gera um único plano contínuo que passa por todas as poses

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
A ação FollowWaypoints é mais adequada para tarefas de autonomia simples, nas quais você deseja parar em cada ponto de referência e executar um comportamento (por exemplo, pausar por 2 segundos, tirar uma foto, esperar que alguém coloque uma caixa sobre ele, etc.). O servidor  waypoint follower de referência Nav2 contém plugins **TaskExecutor** para executar uma tarefa em cada ponto de referência.

FollowWaypoints.action:

```txt
#goal definition
geometry_msgs/PoseStamped[] poses
---
#result definition
int32[] missed_waypoints
---
#feedback definition
uint32 current_waypoint
```

Como você pode ver, a API é simples. Ela recebe o conjunto de poses, onde a última pose é o objetivo final. O feedback é o ID do waypoint atual que está sendo executado e retorna um vetor de IDs de waypoints perdidos no final, caso algum deles esteja inavegável.

### Launch the FollowWaypoints action
A ação FollowWaypoints não é iniciada por padrão com o sistema de navegação. Portanto, adicione-a ao seu arquivo de inicialização de navegação se desejar utilizá-la.

Comece criando um arquivo de configuração para ela. Na pasta **config** do seu pacote **path_planner_server**, adicione um novo arquivo de configuração chamado [**waypoint_follower.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_new_features/config/waypoint_follower.yaml).

Neste exemplo, carregue o plugin padrão **WaitAtWaypoint**. Esses plugins pausam o robô por um tempo específico após atingir cada ponto de referência. O tempo de espera pode ser configurado com o parâmetro **waypoint_pause_duration** (especificado em milissegundos).

Existem outros plugins disponíveis, como **PhotoAtWaypoint** ou **InputAtWaypoint**. Se quiser saber mais sobre eles, consulte a documentação oficial [aqui](https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html).

Agora é hora de adicionar a ação FollowWaypoints ao seu arquivo de inicialização. Para isso, serão necessárias algumas modificações.

Primeiro, adicione o novo caminho do arquivo de configuração:

navigation.launch.py:
```python
waypoint_follower_yaml = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'waypoint_follower.yaml')
```

Em seguida, adicione um novo elemento Node para iniciar o servidor waypoint_follower:

navigation.launch.py:
```python
Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[waypoint_follower_yaml]),
```

Por fim, e importante, você precisa adicionar o novo nó a ser iniciado no gerenciador de ciclo de vida:

navigation.launch.py:
```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager',
    output='screen',
    parameters=[{'autostart': True},
                {'node_names': ['map_server',
                                'amcl',
                                'controller_server',
                                'planner_server',
                                'recoveries_server',
                                'bt_navigator',
                                'waypoint_follower']}])
```

### Demo
Nesta demonstração, use a ação FollowWaypoints para que seu robô se desloque do ponto de parada até um conjunto de pontos de inspeção. O plugin TaskExecutor,  waypoint follower do Nav2, captura imagens e escaneia RFID das prateleiras, que podem ser analisadas para gerenciamento de estoque. Veja esse script [**follow_waypoints.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_new_features/scripts/follow_waypoints.py) como exemplo.

Assim como você fez antes, use o vetor inspection_points como entrada para chamar o método followWaypoints(). Este método solicita que o robô siga um conjunto de pontos de referência (lista de mensagens PoseStamped). Isso executará o plugin TaskExecutor escolhido em cada pose.

```python
i = 0
while not navigator.isTaskComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
        print('Executing current waypoint: ' +
              str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
```

Neste código, você está imprimindo o ponto de referência atual do robô. No entanto, lembre-se de que você poderia realizar outras tarefas (mais significativas) aqui.

## Filtros de Costmap
O Nav2 oferece um poderoso conjunto de recursos conhecidos como filtros de Mapa de Custo, que permitem modificar o comportamento de navegação do robô com base em áreas específicas do mapa. Um filtro de Mapa de Custo atua como uma camada adicional (ou máscara) aplicada aos seus Mapas de Custo existentes, permitindo estratégias de navegação mais dinâmicas e sensíveis ao contexto.

Com esses filtros, você pode definir zonas específicas que influenciam a movimentação do robô. Por exemplo, você pode criar Zonas de Exclusão, que impedem o robô de entrar em áreas restritas, ou Zonas de Velocidade Restrita, onde a velocidade do robô é ajustada automaticamente dependendo da região designada. Esses recursos aumentam a segurança, a eficiência e a adaptabilidade da navegação em ambientes complexos.

## Keepout Mask (Máscara de Exclusão)
Revise como fazer o robô evitar certas zonas do ambiente usando um filtro de Máscara de Exclusão.

A Máscara de Exclusão é um arquivo semelhante a um mapa, contendo a máscara a ser aplicada como Zona de Exclusão. Portanto, para aplicar uma máscara, você precisará de um arquivo de mapa do ambiente. Agora, você pintará de PRETO a área do mapa que deseja que seu robô evite. Um exemplo simples é o seguinte:

![map_keepout](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)/imagens)

Você pode se perguntar: "Por que preciso pintar de preto?". A tonalidade de cada pixel na máscara representa informações codificadas para o filtro Costmap específico que você usará. O arquivo de máscara recebido está sendo lido pelo Map-Server e convertido em valores de OccupancyGrid no intervalo [0 a 100], onde:
* 0 significa célula livre
* 100 significa célula ocupada

Para a máscara do Filtro Keepout, quanto maior o valor, mais restrita é a área. Portanto, uma área preta corresponde ao valor 100 (o robô não pode navegar por ela). Para valores intermediários, o robô poderá se mover nessas áreas, mas sua presença será "indesejada" (quanto maior o valor, mais cedo os planejadores tentarão tirar o robô dessa área).

Depois de editar seu mapa, carregue-o novamente. 

### Launch the Costmap Filter nodes
Após criar os arquivos de mapa editados, é hora de iniciar os nós necessários. Mas antes disso, atualize os arquivos de configuração. Primeiro, você atualizará a configuração do Costmap, que pode ser encontrada nos arquivos planner_server.yaml e controller.yaml, por exemplo

Comece adicionando um parâmetro de filtro:

```yaml
filters: ["keepout_filter"]
```

E, claro, você tem que adicionar a configuração do filtro:

```yaml
keepout_filter:
    plugin: "nav2_costmap_2d::KeepoutFilter"
    enabled: True
    filter_info_topic: "/costmap_filter_info"
```

Como você pode ver, você especifica o filtro a ser usado, **KeepoutFilter**.


```yaml

```