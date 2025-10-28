# Índice

1.  [Recursos do Nav2](#recursos-do-nav2)
    *   [A API Simple Commander](#a-api-simple-commander)
    *   [Navigate To Pose](#navigate-to-pose)
        *   [Demo](#demo)
    *   [Navigate Through Poses](#navigate-through-poses)
        *   [Demo](#demo-1)
    *   [Waypoint Following](#waypoint-following)
        *   [Launch the FollowWaypoints action](#launch-the-followwaypoints-action)
        *   [Demo](#demo-2)
    *   [Filtros de Costmap](#filtros-de-costmap)
    *   [Keepout Mask (Máscara de Exclusão)](#keepout-mask-máscara-de-exclusão)
        *   [Launch the Costmap Filter nodes](#launch-the-costmap-filter-nodes)
    *   [Speed Limits](#speed-limits)
        *   [Configure os Speed Limit nodes](#configure-os-speed-limit-nodes)
2.  [Behavior Trees (BTs)](#behavior-trees-bts)
    *   [bt_navigator node](#bt_navigator-node)
    *   [Como criar um comportamento (behavior)](#como-criar-um-comportamento-behavior)
    *   [Nós BT usados](#nós-bt-usados)
        *   [RecoveryNode](#recoverynode-nó-definido-pelo-nav2)
        *   [PipelineSequence](#pipelinesequence-nó-definido-por-nav2)
        *   [RateController](#ratecontroller-nó-definido-por-nav2)
        *   [ComputePathToPose](#computepathtopose-nó-definido-por-nav2)
        *   [ClearEntireCostmap](#clearentirecostmap-nó-definido-nav2)
        *   [SequenceStar](#sequencestar-nó-definido-por-bt)
        *   [FollowPath](#followpath-nó-definido-pelo-nav2)
        *   [Spin](#spin-nó-definido-pelo-nav2)
        *   [Wait](#wait-nó-definido-pelo-nav2)
        *   [Conclusões](#conclusões)
    *   [Como fornecer o comportamento ao bt_navigator](#como-fornecer-o-comportamento-ao-bt_navigator)
    *   [Exemplo](#exemplo)
    *   [Recovery Behaviors](#recovery-behaviors)
        *   [Configurações](#configurações)
        *   [Como eles funcionam](#como-eles-funcionam)
3.  [Plugins Nav2 e criação de plugins personalizados](#plugins-nav2-e-criação-de-plugins-personalizados)
    *   [Plugins no Nav2](#plugins-no-nav2)
    *   [Plugins padrão](#plugins-padrão)
        *   [Costmaps](#costmaps)
        *   [Planner](#planner)
        *   [Controller](#controller)
    *   [Criação de plugins Nav2 personalizados](#criação-de-plugins-nav2-personalizados)
        *   [Costmap Plugin](#costmap-plugin)
        *   [Planner Plugin](#planner-plugin)
        *   [Controller Plugin](#controller-plugin)
4.  [Controller Server In Depth](#controller-server-in-depth)
    *   [Controller Server Config](#controller-server-config)
    *   [Controller Server Plugins](#controller-server-plugins)
        *   [1. progress_checker_plugin](#1-progress_checker_plugin)
        *   [2. goal_checker_plugins](#2-goal_checker_plugins)
        *   [3. controller_plugins](#3-controller_plugins)


# Recursos do Nav2
**NOTA**: Este curso foi criado para o ROS2 Humble.
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

## Waypoint Following
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

![map_keepout](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/map_keepout.png)

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

Agora, defina parâmetros extras para os novos nós de filtro que você iniciará. Você iniciará dois novos nós:

* Servidor de Informações do Filtro Costmap: Este nó publicará mensagens nav2_msgs/CostmapFilterInfo. Essas mensagens contêm metadados, como o tipo de filtro ou coeficientes de conversão de dados.
* Servidor de Mapa de Máscaras: Este nó publicará mensagens OccupancyGrid.

Ambas as mensagens precisam ser publicadas em pares para gerar o filtro Costmap. Se quiser saber mais sobre esse processo interno, consulte o [documento de design](https://github.com/ros-navigation/navigation2/blob/main/doc/design/CostmapFilters_design.pdf).

Comece criando um novo arquivo de configuração chamado [**filters.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_new_features/config/filters.yaml), onde você coloca os parâmetros necessários para esses nós.

O parâmetro **type** define o tipo de filtro Costmap utilizado. Os valores são:
* 0 para filtro de Zonas de Exclusão/faixas preferenciais
* 1 para filtro de velocidade (se o limite de velocidade for especificado em % da velocidade máxima)
* 2 para filtro de velocidade (se o limite de velocidade for especificado em valor absoluto (m/s))

O parâmetro **mask_topic** define o tópico para publicar a máscara de filtro. Portanto, o nome do tópico especificado deve ser o mesmo que o parâmetro **topic_name** do Map-Server.

A base e o multiplicador são coeficientes usados ​​para aplicar a máscara ao filtro. Eles são usados ​​na **seguinte fórmula**: filter_space_value = base + multiplicador * valor_da_máscara

Para Zonas de Exclusão, eles precisam ser definidos como 0,0 e 1,0, respectivamente, para mostrar explicitamente que você tem uma conversão um-para-um de valores da OccupancyGrid -> para um espaço de valores de filtro.

Se você seguiu todas as instruções corretamente, você obterá um Costmap como o mostrado abaixo:

![keepout_filter](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/keepout_filter.png)

Agora, quando você enviar um gol para perto da Keepout Zone, o plano evitará a área rosa.

**Cuidado**:

Ao tentar navegar perto da Keepout Zone, você pode acabar recebendo erros como este:

```shell
[controller_server-3] [ERROR] [1654803915.474550722] [DWBLocalPlanner]: 1.00: ObstacleFootprint/Trajectory Hits Obstacle.
```

Se você receber esse erro, poderá abrir os parâmetros **controller.yaml** e remover o crítico chamado **ObstacleFootprint** da lista de críticos.

```yaml
#critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
    
critics: ["RotateToGoal", "Oscillation", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

## Speed Limits
O princípio do Filtro de Limites de Velocidade é semelhante ao usado para Zonas de Exclusão. No entanto, neste caso, os valores da máscara terão um significado diferente: limites de velocidade codificados para as áreas correspondentes à célula no mapa.

Como você já sabe, os valores da Grade de Ocupação pertencem ao intervalo [0 a 100]. Para o Filtro de Velocidade, um valor 0 significa que não há limite de velocidade na área correspondente ao mapa. Valores do intervalo [1 a 100] estão sendo convertidos linearmente em um valor de limite de velocidade pela seguinte fórmula: limite_de_velocidade = dados_da_máscara_do_filtro * multiplicador + base

Simplificar significa que quanto mais clara a porcentagem de cinza usada, menor será o limite de velocidade e vice-versa.

Pinte de CINZA (use tons diferentes para áreas diferentes) as áreas do mapa onde você deseja que seu robô tenha limites de velocidade.

Um exemplo é o seguinte:

![map_speeds_edit](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/map_speeds_edit.png)

Após editar seu mapa, carregue-o novamente. 

### Configure os Speed Limit nodes
Nesta unidade, aplique o filtro de velocidade apenas ao Costmap global. Portanto, primeiro, atualize o parâmetro do filtro:

Add to planner_server.yaml:
```yaml
# For Keepout Zones
#filters: ["keepout_filter"]

# For Speed Limits
filters: ["speed_filter"]
```

E, claro, você tem que adicionar a configuração do filtro:

Add to planner_server.yaml:
```yaml
speed_filter:
    plugin: "nav2_costmap_2d::SpeedFilter"
    enabled: True
    filter_info_topic: "/costmap_filter_info"
    speed_limit_topic: "/speed_limit"
```

Como você pode ver, você especifica o filtro a ser usado, **SpeedFilter**.

Além disso, você precisa especificar o mesmo **speed_limit_topic** para a configuração do **controller_server**:

controller.yaml
```yaml
...

controller_server:
  ros__parameters:
    ...
    # For Speed Limits
    speed_limit_topic: "/speed_limit"
    ...
```

Agora atualize o arquivo filters.yaml para definir os parâmetros necessários para o filtro de velocidade:

filters.yaml
```yaml
costmap_filter_info_server:
  ros__parameters:
    use_sim_time: true
    filter_info_topic: "/costmap_filter_info"
    # For Keepout Zones
    #type: 0
    #mask_topic: "/keepout_filter_mask"
    #base: 0.0
    #multiplier: 1.0
    # For Speed Limits
    type: 1
    mask_topic: "/speed_filter_mask"
    base: 100.0
    multiplier: -1.0

filter_mask_server:
  ros__parameters:
    use_sim_time: true
    frame_id: "map"
    # For Keepout Zones
    #topic_name: "/keepout_filter_mask"
    #yaml_filename: "/home/user/ros2_ws/src/map_server/maps/map_keepout.yaml"
    # For Speed Limits
    topic_name: "/speed_filter_mask"
    yaml_filename: "/home/user/ros2_ws/src/nav2_pkgs/map_server/maps/map_speeds.yaml"
```

Se você seguiu todas as instruções corretamente, você obterá um Costmap como o mostrado abaixo:

![speed_filter_costmap](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/speed_filter_costmap.png)

Para visualizar a máscara de filtro, adicione uma exibição de Mapa com a seguinte configuração:

![speed_display](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/speed_display.png)

Envie uma meta Nav2 para o robô e verifique como o limite de velocidade muda dependendo da área de navegação do robô.

Você também pode visualizar o tópico **/speed_limit** para obter dados adicionais. Sempre que o robô entrar em uma nova área com limite de velocidade, uma mensagem como a seguinte será publicada neste tópico:

![speed_limit_topic](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/speed_limit_topic.png)

# Behavior Trees (BTs)
Nesta unidade, você explorará como as Árvores de Comportamento (BTs) são integradas ao Nav2 para controlar o comportamento de navegação de um robô. As BTs são essenciais para lidar tanto com a navegação padrão direcionada a objetivos quanto com situações desafiadoras em que o robô encontra obstáculos ou erros.

Ao final desta unidade, você terá uma compreensão clara de como as BTs aprimoram a tomada de decisões e a adaptabilidade na navegação autônoma.

![bt](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/groot_behavior2.png)

O pacote do sistema de navegação é responsável por gerenciar as árvores de comportamento no nav2_bt_navigator.

O nav2_bt_navigator é responsável por controlar o movimento até o objetivo do robô. Ele é composto por várias partes:
1. O nó bt_navigator e seu arquivo de configuração
2. O comportamento do nó bt_navigator
3. O nó recoveries_server e seu arquivo de configuração

O comportamento do nó bt_navigator e como ele se conecta ao recoveries_server são especificados usando Árvores de Comportamento. Nesta unidade, você aprenderá como criar esses comportamentos e fornecê-los ao sistema de navegação.

## bt_navigator node
O bt_navigator é o nó do Nav2 responsável por **gerenciar** o planejador de caminho, o controlador e os comportamentos de recuperação.

A maneira como o bt_navigator deve gerenciar esses nós é definida em uma árvore de comportamento.

VOCÊ DEVE CRIAR UM COMPORTAMENTO PARA O bt_navigator definir corretamente como ele deve mover o robô.

## Como criar um comportamento (behavior)
Para criar o comportamento, crie um **arquivo XML** usando os tipos de nós para comportamentos disponíveis. Este XML será fornecido ao bt_navigator para ser executado quando necessário.

A **biblioteca Behavior Tree.CPP** fornece um conjunto de nós de comportamento já prontos. No entanto, o Nav2 incluiu seu próprio tipo de nós de comportamento. Neste capítulo, você aprenderá sobre os nós de comportamento que o Nav2 fornece.

IMPORTANTE: Você não deve confundir nós ROS com nós de comportamento; são duas coisas diferentes. Para evitar confusão, nesta unidade, você escreverá nós ROS em letras normais e *nós* de comportamento em itálico.

Exemplo de comportamento: o arquivo **behavior.xml** que você usou neste curso:

```xml
<!--
  This Behavior Tree replans the global path periodically at 1 Hz, and it also has
  recovery actions.
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

Esse comportamento especificado em XML pode ser representado por esta imagem:

![bt](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/groot_behavior2.png)

Agora analise os *nós* de comportamento envolvidos no comportamento anterior.

Para cada comportamento, comece com as seguintes tags:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    ...
  </BehaviorTree>
</root>
```

* A tag **root** identifica a árvore principal a ser executada.
* A tag **BehaviorTree** identifica o que se segue como uma árvore de comportamento com um nome específico (o ID contém o nome).

Em seguida, dentro da tag **BehaviorTree**, inclua os nós da árvore de comportamento que você deseja usar e na ordem em que o comportamento desejado é construído. Por exemplo, para o comportamento padrão que você usou no curso de navegação, você queria o seguinte comportamento:

* Replaneje o caminho global a cada segundo (1 Hz)
* Siga esse caminho.
* Se o robô estiver travado, faça o seguinte:
  1. Limpe o Mapa de Custo local.
  2. Limpe o Mapa de Custo global.
  3. Gire para verificar os novos obstáculos e construa novamente os Mapas de Custo.
  4. Aguarde 5 segundos e volte para o comportamento principal.

Veja como ele é implementado usando os nós de comportamento fornecidos pelo Nav2.

### Nós BT usados
#### **RecoveryNode** (nó definido pelo Nav2)

Este nó é usado para encapsular dois outros subnós e controlar sua ativação da seguinte maneira:

1. Este nó iniciará a execução do primeiro subnó.
2. Se o primeiro subnó retornar SUCESSO, o RecoveryNode retornará SUCESSO.
3. Se o primeiro subnó retornar FALHA, ele executa o segundo subnó.
4. Se o segundo subnó retornar SUCESSO, ele executa o primeiro subnó novamente.
5. Se o segundo subnó retornar FALHA, ele retornará FALHA e encerrará.

Geralmente, é usado para iniciar uma tarefa de navegação (o primeiro subnó) e definir sua ação de recuperação em caso de falha (o segundo subnó).

Veja um exemplo do arquivo XML anterior:

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
    <PipelineSequence name="NavigateWithReplanning">
      ...
    </PipelineSequence>
    <SequenceStar name="RecoveryActions">
      ...
    </SequenceStar>
</RecoveryNode>
```

* Este nó é denominado *NavigateRecovery*.
* Seu comportamento inicia iniciando o nó **PipelineSequence**, denominado *NavigateWithReplanning*.
* Se o nó *NavigateWithReplanning* falhar, ele iniciará o nó **SequenceStar**, denominado *RecoveryActions*.
* Se *RecoveryActions* for bem-sucedido, ele iniciará *NavigateWithReplanning* novamente.
* Se *RecoveryActions* falhar, o nó *NavigateRecovery* também falhará e encerrará sua execução.

No entanto, este *nó* fará até seis tentativas para concluir a sequência com sucesso. Se, após seis tentativas, ainda retornar FAILURE, ele também retornará FAILURE.

**NOTA**: Lembre-se de que os *nós* de comportamento podem ter portas de entrada (o equivalente aos parâmetros passados ​​ao *nó*) e portas de saída (o equivalente aos resultados retornados pelo *nó*). Por exemplo, o **RecoveryNode** tem uma porta de entrada chamada **number_of_retries** (o nome é outra porta de entrada que existe para cada nó por padrão). Consulte a documentação de cada *nó* para obter a lista de portas de entrada e saída.

#### **PipelineSequence** (nó definido por Nav2)

Este nó ativará os subnós da seguinte maneira:

1. O primeiro nó ativa o primeiro subnó até que ele retorne SUCCESS.
2. Isso ativa o primeiro e o segundo subnós (novamente) até que o segundo retorne SUCCESS.
3. Em seguida, ele ativa o primeiro, o segundo e o terceiro subnós (novamente) até que o terceiro retorne SUCCESS, e assim por diante.
4. Ele será interrompido se algum dos subnós retornar FAILURE ou o último submódulo retornar SUCCESS.

Veja um exemplo do arquivo XML anterior:

```xml
<PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
    ...
    </RateController>
    <RecoveryNode number_of_retries="1" name="FollowPath">
    ...
    </RecoveryNode>
</PipelineSequence>
```

Aqui, este nó é usado para calcular o caminho global a cada segundo e, em seguida, fazer o robô seguir o caminho recém-calculado. Isso é útil para considerar modificações na posição atual do robô, adaptando o caminho global a essas modificações.

* Este nó é denominado *NavigateWithReplanning* e possui dois subnós.
* O primeiro é um nó **RateController** (veja abaixo seu significado). Este nó inclui o cálculo do caminho até o objetivo.
* O segundo é um nó **RecoveryNode**. Este é usado para fazer o robô seguir o caminho calculado.

#### **RateController** (nó definido por Nav2)

Este nó chamará os nós subsequentes em uma determinada frequência que você especificar como parâmetro.

Exemplo do arquivo XML anterior:

```xml
<RateController hz="1.0">
    <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            ...
    </RecoveryNode>
</RateController>
```

Neste caso, o **RateController** chamará um RecoveryNode a 1 Hz.

```xml
<RecoveryNode number_of_retries="1" name="ComputePathToPose">
   <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
   <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
</RecoveryNode>
```

O RecoveryNode chamado *ComputePathToPose* possui dois subnós:

1. Um nó **ComputePathToPose**
2. Um nó **ClearEntireCostmap**

Ambos são explicados abaixo.

#### **ComputePathToPose** (nó definido por Nav2)

Chamadas ao servidor de ações **ComputePathToPose** do ROS2 fornecido pelo nó ROS2 **nav2_planner** (o Servidor do Planejador) para calcular o caminho até o objetivo. O objetivo é introduzido no nó usando uma **variável do quadro-negro** (variável chamada {goal}). Em seguida, o resultado do planejador é introduzido em outra variável do quadro-negro chamada {path}.

**O que é o quadro-negro?**

O quadro-negro é como um espaço para variáveis ​​que todos os nós podem acessar. Ele é usado para compartilhar informações entre os nós. Um nó pode inserir um valor ali e outro nó pode lê-lo. 

Exemplo do arquivo XML anterior:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
```

Nesse caso, alguém colocou o valor da variável {goal} no quadro-negro, e o nó está preenchendo a variável {path} com o caminho calculado resultante. *Goal e planner_id são as **portas de entrada** do nó, e path é sua **porta de saída**.

#### **ClearEntireCostmap** (nó definido Nav2)
Chama o serviço que limpa o Costmap. Você deve indicar qual servidor chamar para limpar o Costmap local ou global.

Exemplo do arquivo XML anterior:

```xml
<ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
```

#### **SequenceStar** (nó definido por BT)
Funciona da mesma forma que o nó **PipelineSequence**, mas não ativará os nós que já concluíram com SUCCESS.

Exemplo do arquivo XML anterior:

```xml
<SequenceStar name="RecoveryActions">
    <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    <Spin spin_dist="1.57"/>
    <Wait wait_duration="5"/>
</SequenceStar>
```

#### **FollowPath** (nó definido pelo Nav2)
Chama o servidor de ações no controlador que enviará comandos às rodas do robô para seguir o caminho calculado.

Exemplo do arquivo XML anterior:

```xml
<FollowPath path="{path}" controller_id="FollowPath"/>
```

#### **Spin** (nó definido pelo Nav2)
Chama o servidor de ação spin ROS2 fornecido pelo nó ROS nav2_recoveries. Este servidor fará o robô girar no lugar o número de graus indicado no parâmetro spin_dist.

Exemplo do arquivo XML anterior:

```xml
<Spin spin_dist="1.57"/>
```

#### **Wait** (nó definido pelo Nav2)
Chame o servidor de ação ROS2 de espera fornecido pelo nó ROS nav2_recoveries. Isso fará com que o comportamento aguarde o número de segundos indicado.

Exemplo do arquivo XML anterior:

```xml
<Wait wait_duration="5"/>
```

### Conclusões
Existem muitos outros nós que você pode usar para o seu comportamento. [Confira aqui a lista oficial de nós Nav2 disponíveis](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html) para saber mais sobre nós e como configurá-los.

Além disso, se você precisa trabalhar bastante com comportamentos, deve aprender mais sobre árvores de comportamento em geral e sobre a biblioteca de implementação BehaviorTree.CPP. Confira os seguintes recursos:

* [Documentação oficial do Nav2 BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html#)
* [Livro "Árvores de Comportamento e Robótica em IA"](https://arxiv.org/pdf/1709.00084)
* [Documentação do BehaviorTree.CPP](https://www.behaviortree.dev/)

## Como fornecer o comportamento ao bt_navigator
Você deve especificar duas coisas para o nó bt_navigation durante a inicialização do nó:

1. o arquivo **bt_navigator.yaml** com a configuração do nó.
2. o arquivo **behavior.xml** usando o parâmetro **default_bt_xml_filename**.

bt_navigator.yaml:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_bt_xml_filename: "/home/user/ros2_ws/src/path_planner_server/config/behavior.xml"
    default_server_timeout: 20
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
```

No parâmetro **plugin_lib_names**, especifique a lista de nós de comportamento necessários para o seu arquivo de comportamento XML. Lembre-se de adicionar os que você precisa neste parâmetro.

Uma lista completa de plugins de nós disponíveis pode ser encontrada [aqui](https://docs.nav2.org/plugins/index.html#behavior-tree-nodes).

Verifique o código que você criou na unidade de Planejamento de Caminho, onde ele executa o carregamento do arquivo de configuração:

```yaml
bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
...
Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[bt_navigator_yaml])
```

## Exemplo
Crie um novo arquivo de comportamento chamado abort_when_low_battery.xml que execute o seguinte comportamento:

1. Sempre que um novo destino for fornecido, o robô se deslocará até esse destino.
2. Se a bateria estiver abaixo de 25% em qualquer momento, aborte o objetivo atual.
3. Chame o tópico da bateria de /battery. Publique nesse tópico um status de bateria de 100%.
4. Em algum momento, altere o valor publicado nesse tópico para 20%. O robô deve mudar seu comportamento e abortar seu objetivo naquele momento.

**Notas:**
* Existe um nó de comportamento do Nav2 usado para detectar o status da bateria: IsBatteryLow. [Consulte a documentação oficial](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsBatteryLow.html) para aprender como usá-lo em seu código.
* Como o nó IsBatteryLow retorna FALSE quando o nível da bateria está ok (porque está verificando IsBatteryLow), você precisará usar um nó BT chamado <Inverter> para negar essa verificação.
* Para publicar em um tópico sobre bateria, use o seguinte comando (de acordo com seus testes necessários): `ros2 topic pub /battery sensor_msgs/BatteryState '{voltage: 12.0, percentage: 1.0, power_supply_status: 3}' `

```xml
<!--
  This Behavior Tree replans the global path periodically at 1 Hz, and it also has
  recovery actions.
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <Inverter>
            <IsBatteryLow battery_topic="/battery" is_voltage="false" min_battery="0.25" />
        </Inverter>
        <RateController hz="1.0">                
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
            </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Recovery Behaviors
**NOTA**: Este curso foi criado para o ROS2 Humble. Antes do ROS2 Humble, o behavior_server era chamado de recoveries_server.

Os Comportamentos de Recuperação do Nav2 são ativados automaticamente quando o robô fica preso, para tentar se recuperar.

Os Comportamentos de Recuperação são ativados pelo bt_navigator quando um dos verificadores especificados no arquivo de configuração do controller_server sinaliza que o robô não está progredindo em direção ao seu objetivo:

1. O controller_server detecta a situação em que o robô está preso e notifica o bt_navigator.
2. Conforme indicado em seu arquivo de configuração, o bt_navigator chama o behavior_server para ativar os plugins de recuperação.

Você configurou os Comportamentos de Recuperação no arquivo de configuração **recovery.yaml** do behavior_server.

```yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

Atualmente, existem três plugins disponíveis fornecidos pelo pacote nav2_behaviors:

1. **Spin**: executa uma rotação no local enquanto os Mapas de Custo são atualizados. Isso é útil quando o robô vê o Mapa de Custo ao seu redor cheio de obstáculos (que podem existir na realidade ou não). Esse comportamento ajudará a determinar os obstáculos existentes no momento e, portanto, aumentará as chances de encontrar um novo caminho até o objetivo.
2. **BackUp** (Recuar): executa um movimento linear do robô por uma determinada distância.
3. **Wait**: para o robô no local e aguarda um determinado tempo. O tempo de espera é fornecido na solicitação de ação.

### Configurações
Alguns parâmetros estão relacionados ao behavior_server em si, enquanto outros estão relacionados apenas aos plugins.

Os parâmetros do recovery_server são:

```yaml
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
```

Todos os plugins operarão sob um determinado conjunto de condições que se aplicam a todos eles. Essas condições dizem respeito às limitações de velocidade e aos quadros a serem utilizados.

```yaml
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

### Como eles funcionam
Cada plugin fornece um servidor de ação que será chamado pelos nós de comportamento que o requerem.

Inicie o sistema de navegação, caso ainda não o tenha feito, e solicite uma lista dos servidores de ação disponíveis: `ros2 action list`

Você deverá obter algo parecido com isto:

```shell
/backup
/compute_path_through_poses
/compute_path_to_pose
/follow_path
/move_robot_as
/navigate_through_poses
/navigate_to_pose
/spin
/wait
```

Como você pode ver, existem os servidores de ação **/backup, /spin e /wait** prontos para serem chamados.

Quando um nó BT solicita um comportamento de recuperação, o navegador BT chamará o servidor de ação do comportamento de recuperação. O navegador BT também os chamará se o controller_server o solicitar devido à falta de progresso no objetivo.

# Plugins Nav2 e criação de plugins personalizados
O Navigation2 (Nav2) é construído com uma arquitetura modular que utiliza plugins para lidar com diferentes aspectos da navegação, como mapas de custos, planejamento e controle. Entender esses plugins é fundamental para personalizar e otimizar o comportamento de navegação do seu robô.

Nesta unidade, você explorará os plugins padrão usados ​​no Nav2 e aprenderá como estender sua funcionalidade criando seus próprios plugins personalizados. Ao final desta unidade, você será capaz de desenvolver plugins personalizados de Mapa de Custos, Planejador e Controlador, permitindo adaptar o Nav2 às necessidades específicas da sua aplicação robótica.

## Plugins no Nav2
A primeira pergunta é:

**Por que vocês usam PLUGINS no Nav2?**

Plugins são usados ​​porque melhoram a flexibilidade do pipeline do Nav2. O uso de plugins permite alterar apenas um arquivo .yaml do controlador, planejador, etc., e alterar completamente a funcionalidade sem muita sobrecarga de compilação.

É especialmente útil para Costmaps porque os filtros do Costmap se acumulam, permitindo que os plugins alterem rapidamente a maneira como os Costmaps são processados ​​para navegação.

O Nav2 possui uma enorme seleção de plugins PLUG and PLAY. Esta é outra grande vantagem. Os desenvolvedores podem criar plugins seguindo uma API básica. Eles estarão prontos para uso na aplicação desejada, com a segurança de que se conectarão aos sistemas de navegação necessários.

Aqui está um link para a lista de plugins disponíveis atualmente: [PLUGINS do Nav2](https://docs.nav2.org/plugins/index.html).

A criação de NOVOS PLUGINS e o upload deles para o REPO geral também são incentivados. Isso enriquecerá a pilha Nav2 e ajudará todos a ter melhores sistemas e aplicativos de navegação.

## Plugins padrão
Aqui, você verá os plugins padrão comumente usados ​​para navegação. Também comentaremos alguns trechos de código padrão para todos os plugins.

**Primeiro, onde você define o plugin que deseja usar?**

A resposta está nos **arquivos .yaml** que carregam os parâmetros para seus diferentes nós de navegação.

Veja diferentes exemplos que você já viu neste curso, mas que talvez não conhecesse:

### Costmaps
No arquivo planner_server.yaml, você pode ver a seguinte seção de parâmetros:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
```

Dê uma olhada nesta linha:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        ...
```

Dentro do namespace **global_costmap/global_costmap/ros__parameters**, você carrega uma lista dentro dos plugins de parâmetros com vários nomes.

Esses nomes podem ser os que você quiser, pois correspondem ao namespace que você definir posteriormente, como:

```yaml
static_layer:
    ...PARAMETERS
inflation_layer:
    ...PARAMETERS
obstacle_layer:
    ...PARAMETERS
```

Neste caso, você tem três plugins chamados:

* static_layer, que carrega o plugin plugin: **"nav2_costmap_2d::StaticLayer"**, [DEFINIÇÃO](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/static_layer.cpp).
* inflation_layer, que carrega o plugin plugin: **"nav2_costmap_2d::InflationLayer"**, [DEFINIÇÃO](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/inflation_layer.cpp).
* obstacle_layer, que carrega o plugin plugin: **"nav2_costmap_2d::ObstacleLayer"**, [DEFINIÇÃO](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/obstacle_layer.cpp).


Todos esses plugins estão na lista fornecida aqui: [PLUGINS Nav2](https://docs.nav2.org/plugins/index.html).

Acesse o código onde esses plugins estão definidos. Você verá que eles têm estruturas muito semelhantes, especialmente quando regulam o mesmo elemento, como Mapas de Custo, Planejamento, Controlador ou Árvores de Comportamento.

Aqui você tem uma versão simplificada do plugin Static Layer:

```cpp
...INCLUDES...


PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)


namespace nav2_costmap_2d
{

StaticLayer::StaticLayer()
: map_buffer_(nullptr)
{
}

StaticLayer::~StaticLayer()
{
}

void
StaticLayer::onInitialize()
{
  ...CODE...
}

void
StaticLayer::activate()
{
    ...CODE...
}

void
StaticLayer::deactivate()
{
  ...CODE...
}

void
StaticLayer::reset()
{
  ...CODE...
}

void
StaticLayer::getParameters()
{
  ...CODE...
}

void
StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  ...CODE...
}

void
StaticLayer::matchSize()
{
  ...CODE...
}

unsigned char
StaticLayer::interpretValue(unsigned char value)
{
  ...CODE...
}

void
StaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  ...CODE...
}

void
StaticLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  ...CODE...
}


void
StaticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  ...CODE...
}

void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  ...CODE...
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
StaticLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  ...CODE...
}

}  // namespace nav2_costmap_2d
```

Como você pode ver, são vários métodos. Estamos mostrando isso porque existem dois tipos de métodos aqui:

* **Métodos Padrão** são usados ​​pelo seu código exclusivo porque você precisa calcular algo, acessar um tópico ou um banco de dados e executar um algoritmo de aprendizado profundo - o que for necessário para executar sua tarefa.
* **Métodos de PLUGIN de API**: Você os sobrescreve da classe pai, neste caso, nav2_costmap_2d::Layer. Cada um tem seus métodos **COMPULSORY** e **OPTIONAL**. Esses métodos permitem que os plugins funcionem **plug and play**, pois o sistema de carregamento de plugins os chama para executar as funções essenciais do plugin para a tarefa. Aqui está a lista. Como você pode ver, todos eles são virtuais para permitir essa funcionalidade:

```cpp
virtual void onInitialize();

virtual void activate();

virtual void deactivate();

virtual void reset();

virtual bool isClearable() {return false;}

virtual void updateBounds(
double robot_x, double robot_y, double robot_yaw, double * min_x,
double * min_y, double * max_x, double * max_y);

virtual void updateCosts(
nav2_costmap_2d::Costmap2D & master_grid,
int min_i, int min_j, int max_i, int max_j);

virtual void matchSize();
```

### Planner
O mesmo acontece com o planejador. Dentro do arquivo planner_server.yaml, você encontra os seguintes parâmetros de carregamento:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

Veja aqui a linha de código:

```yaml
planner_plugins: ["GridBased"]  
```

Após alguns testes, parece que **NÃO É POSSÍVEL alterar o NOME da TAG DO PLUGIN**. Portanto, o nome precisa ser **GridBased**. Caso contrário, você receberá um erro ao executar o planejador:

```shell
planner GridBased is not a valid planner. Planner names are: WHATEVER_NAME_YOU_GAVE.
```

Neste caso, ele está carregando o plugin [nav2_navfn_planner/NavfnPlanner CODE](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner).

```cpp
NavfnPlanner();

~NavfnPlanner();


void configure(
const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


void cleanup() override;


void activate() override;

void deactivate() override;

nav_msgs::msg::Path createPlan(
const geometry_msgs::msg::PoseStamped & start,
const geometry_msgs::msg::PoseStamped & goal) override;
```

Você não está definindo virtualmente, apenas configurando para **override**.

Você usa **virtual** para a declaração da função da classe base e, se quiser a sua própria, tem a opção de ser sobrescrita. É necessário apenas para o método base.

Você usa override para uma substituição de classe derivada. Não é obrigatório, mas gera um erro de compilação se os métodos não corresponderem, o que ajuda a manter o código.

Neste caso de código, o desenvolvedor queria verificar se os métodos coincidiam com os da classe base. Nenhum virtual foi usado.

### Controller
Neste caso, você pode encontrar o parâmetro load do plugin no arquivo controller.yaml, assim:

```yaml
controller_server:
  ros__parameters:
    ...
    controller_plugins: ["FollowPath"]

    ...
    FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        ..NEEDED PLUGIN PARAMETERS
        critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

Aqui, você usa a tag de plugin **FollowPath** e carrega o plugin a partir do **dwb_core::DWBLocalPlanner**. [O código está AQUI](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller).

Observe que você também está carregando dentro dele MAIS plugins, neste caso, críticos:

```yaml
critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

Esses plugins são definidos [**AQUI**](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller/dwb_critics). Por exemplo, aqui está o plugin [**OSCILLATION CRITIC**](https://github.com/ros-navigation/navigation2/blob/main/nav2_dwb_controller/dwb_critics/src/oscillation.cpp). Os plugins críticos são da classe base dwb_core::TrajectoryCritic.

Como você pode ver, este é um tópico profundo e seria necessário um curso inteiro para explicar os plugins em detalhes.

Agora, você criará vários exemplos e exercícios que ensinam como modificar e criar plugins PERSONALIZADOS para Mapas de Custo, Planejamento e Controladores.

## Criação de plugins Nav2 personalizados
A maioria das etapas para a criação de plugins segue uma estrutura semelhante, mas os detalhes específicos da **implementação** dependem da classe base da qual cada plugin é derivado. A classe base define a funcionalidade principal e a interface às quais o plugin deve aderir, influenciando sua integração com o sistema geral. Entender essas diferenças é crucial para implementar e estender corretamente o comportamento de cada plugin dentro do framework Nav2.

Antes de se aprofundar na criação de plugins personalizados, você precisa de uma pilha de navegação funcional para testar seus novos plugins. 

### Costmap Plugin
Neste primeiro exemplo, você aprenderá os conceitos fundamentais para criar seu próprio plugin no Nav2. Especificamente, você desenvolverá um plugin personalizado de filtragem de Costmap, que permite modificar a maneira como a pilha de navegação processa os mapas de custo. Esse conhecimento servirá como base para a construção de plugins mais avançados posteriormente.

#### Crie um pacote
Crie um novo pacote ROS 2 ([**custom_nav2_costmap_plugin**](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin)) para armazenar todos os plugins do Costmaps. Execute os seguintes comandos para isso:

```shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_costmap_plugin --dependencies rclcpp nav2_costmap_2d pluginlib
cd ~/ros2_ws
colcon build --packages-select custom_nav2_costmap_plugin
source install/setup.bash
```

Vamos analisar o código [**gradient_layer.hpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/include/custom_nav2_costmap_plugin/gradient_layer.hpp):

Primeiro, vamos comentar os princípios básicos do arquivo `*.hpp`:

Primeiro, defina um namespace para a classe do seu plugin. Isso evita que o sistema se confunda com métodos que têm o mesmo nome de outros plugins. No seu caso, o namespace usado é **custom_nav2_costmap_plugin**.

```cpp
namespace custom_nav2_costmap_plugin
{
  ..CUSTOM CLASS DEFINITION  
}
```

O nome da sua classe personalizada é **GradientLayer** e herda da classe base chamada **nav2_costmap_2d::Layer**. É a partir dela que você herda os métodos e os substitui.

```cpp
class GradientLayer : public nav2_costmap_2d::Layer
{
        ...CLASS METHODS AND VARIABLES
};
```

Todos os métodos que você substituirá da classe base estão dentro da seção pública da sua classe:

```cpp
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}
```

* **updateBounds()**: OBRIGATÓRIO. Como este método é obrigatório, você PRECISA DEFINI-LO AQUI no seu plugin. Ele é o responsável por decidir quanto do Costmap será atualizado com base nos limites, posição, etc. do robô. Quanto menor a área, mais rápido será o seu plugin.

* **updateCosts()**: OBRIGATÓRIO. Ele atualiza os custos dentro dos limites definidos no método **updateBlounds()**. Isso pode ser feito adicionando-os aos custos já existentes calculados por plugins anteriores em execução, ou você pode sobrescrever os valores anteriores completamente.

* **reset()**: OBRIGATÓRIO. Quando o sistema do plugin de navegação reinicia o plugin, por exemplo, por meio das árvores de comportamento, este é o método executado.

* **onInitialize()**: NÃO OBRIGATÓRIO. Insira aqui qualquer código que precise ser executado ao iniciar o plugin, como buscar parâmetros, inicializar variáveis ​​e contadores.

* **matchSize()**: NÃO OBRIGATÓRIO. Você o chama quando o tamanho do mapa é alterado.

* **onFootprintChanged()**: NÃO OBRIGATÓRIO. Isso é útil se você tiver um robô que pode alterar o footprint, como conversíveis, anexar ferramentas ou carregar objetos maiores do que footprint original. Execute aqui o que for necessário quando isso acontecer.

Como você pode ver no exemplo, aqui você não definiu **matchSize()**, mas como NÃO É OBRIGATÓRIO, ele usará o método padrão (que não faz nada, porque não está implementado na classe base CPP layer.hpp, layer.cpp).

Vamos analisar o código [**gradient_layer.cpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/src/gradient_layer.cpp):

**onInitialize()**:

```cpp
void
GradientLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
}
```

Aqui, você está buscando um parâmetro chamado **enabled**. Este parâmetro está no arquivo **.yaml**, onde você carregará o plugin e seus parâmetros, neste caso, enabled, como mostrado abaixo:

```cpp
custom_gradient_layer:
    plugin: "custom_nav2_costmap_plugin/GradientLayer"
    enabled: True
```

**updateBounds()**:

```cpp
void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}
```

Aqui você pode ver:

Os parâmetros de entrada para os métodos, alguns comentados no arquivo .cpp:

```cpp
double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/
```

* Isso evita que compiladores excessivamente pedantes reclamem de variáveis ​​não utilizadas definidas como parâmetros. Você não está usando esses parâmetros, mas precisa tê-los, pois os métodos virtuais da classe base são assim. Portanto, faça isso para evitar avisos ou até mesmo erros de parâmetros UNUSED.
* Este código obtém os maiores limites e, em seguida, é atualizado com os valores fornecidos como parâmetros.

**onFootprintChanged():**

```cpp
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}
```

É uma função fictícia que imprime uma mensagem quandofootprint é alterado e define **need_recalculation_** como true, para forçar novamente essa seção no método **updateBounds()**.

**updateCosts():**

```cpp
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case, using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}
```

Neste método, você sobrescreve quaisquer custos que estavam anteriormente em **master_grid**.

Se você quisesse trabalhar com base nos filtros anteriores do Costmap, como consta no código, você teria que usar os métodos e **costmap_** em vez de **master_grid**:

```cpp
updateWithAddition()
updateWithMax()
updateWithOverwrite()
updateWithTrueOverwrite()
```

Você atualiza os custos com base na posição no Mapa de Custos e em um padrão fixo e repetitivo. Não é útil, mas é uma maneira de saber como gerenciar Mapas de Custos.

* O conceito mais importante aqui é o valor do custo. Ele varia de 0 a 255, sendo 0 SEM custo, e 255, o custo máximo. Ou, em outras palavras: 0 SEM perigo de bater em um obstáculo, 255 RISCO MUITO ALTO de bater em um obstáculo.
* i e j são equivalentes a x e y no mapa. A única diferença é que não se trata de distância, mas de quadrados de grade, nos quais o mapa é dividido para calcular os Mapas de Custos.

**Export Plugin MACRO:**

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_nav2_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
```

Usar isso como um plugin na pilha ROS2 Nav2 é vital. Então, aqui você define o seguinte:

```cpp
PLUGINLIB_EXPORT_CLASS(NAMESPACE_OF_OUR_CUSTOM_PLUGIN::NAME_OF_CUSTOM_CLASS, BASE_CLASS_NAMESPACE::BASE_CLASS)
```

#### Crie o arquivo XML de informações do plugin
Para que o sistema carregue seu plugin, crie um arquivo de informações em formato **XML** ([**gradient_layer.xml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/gradient_layer.xml)). Este arquivo conterá todas as informações necessárias para encontrar a **biblioteca** compilada do seu plugin, o **nome** que você deu a ele e o **namespace** e a **classe** que ele utiliza.

Vamos comentar cada TAG XML:

* **library path**: Este é o nome que você dá à biblioteca ao compilá-la. Você o define dentro do **CMakelists.txt**, que será revisado na próxima etapa.
* **class type**: Este nome indica o **CUSTOM_PLUGIN_NAMESPACE** e a **CUSTOM_CLASS** que você definiu nos arquivos **.cpp e .hpp.** No seu caso, é CUSTOM_PLUGIN_NAMESPACE=custom_nav2_costmap_plugin, CUSTOM_CLASS=GradientLayer.
* **class base_class_type**: Aqui você declara novamente a **BASE CLASS** com seu namespace no qual sua classe personalizada é baseada.
* **class name**: Esta tag é opcional; se não for inserida, o nome fornecido será o mesmo do tipo de classe. Mas é melhor usá-la porque torna seu código mais legível. Neste caso, o nome é custom_nav2_costmap_plugin/GradientLayer.

#### Configure o CMakelists.txt e o package.xml para compilação
Esta última etapa é necessária para dizer ao ROS para compilar seu plugin como uma biblioteca e exportá-lo para o sistema de plugins, para que ele possa ser encontrado e usado.

Vamos analisar o [**CMakelists.txt**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/CMakeLists.txt):

**Configurando o nome da biblioteca:**
Primeiro, defina uma variável chamada **lib_name** que contém o nome que você selecionou para sua biblioteca de plugins quando compilada:

```txt
set(lib_name ${PROJECT_NAME}_core)
```

Neste caso, o nome é ${PROJECT_NAME}_core = custom_nav2_costmap_plugin_core

É por isso que, no arquivo de informações do plugin gradient_layer.xml, você define custom_nav2_costmap_plugin_core como o caminho.

```txt
<library path="custom_nav2_costmap_plugin_core">
```

**lista de dependências de nomes de conjuntos:**

```txt
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)
```

Nomeie **dep_pkgs** a lista de dependências na sua biblioteca que você usará depois.

**Adicionando a biblioteca:**

```txt
add_library(${lib_name} SHARED
            src/gradient_layer.cpp)
```

Declare aqui Compile o arquivo

```txt
gradient_layer.cpp into a library named lib_name=custom_nav2_costmap_plugin_core.
```

**incluir definição de pasta:**
Incluir os arquivos de cabeçalho dentro da pasta include do pacote

```txt
include_directories(include)
```

**instalar a biblioteca compilada:**

```txt
install(TARGETS ${lib_name}
        DESTINATION lib)
```

**disponibilizar a biblioteca para o sistema de plugins:**
```txt
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
```

Diga ao pacote onde encontrar todas as informações relacionadas ao seu plugin no arquivo [**package.xml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/package.xml).

Esta é a única linha alterada:

```xml
<costmap_2d plugin="${prefix}/gradient_layer.xml" />
```

#### Configure, Compile and Test

**Configure:**
Crie um novo lançamento do [**Pathplanner**](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server) que carregue seu plugin personalizado. Para isso, crie um novo lançamento e alguns novos arquivos de configuração:

```shell
cd ~/ros2_ws/
touch ~/ros2_ws/src/nav2_pkgs/path_planner_server/launch/pathplanner_custom_costmap_plugin.launch.py
mkdir ~/ros2_ws/src/nav2_pkgs/path_planner_server/custom_costmap
cd ~/ros2_ws/src/nav2_pkgs/path_planner_server/custom_costmap
touch controller.yaml
touch planner_server.yaml
```

Você adicionará o plugin no Mapa de Custo **Local** e no Mapa de Custo **Global**.

Veja aqui o arquivo [**controller.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/config/custom_costmap/controller.yaml).

* Como você pode ver, você substituiu a **inflation_layer** pela **custom_gradient_layer**.
* Use o nome custom_nav2_costmap_plugin/GradientLayer especificado no arquivo gradient_layer.xml.

```yaml
# plugins: ["voxel_layer", "inflation_layer"]
plugins: ["voxel_layer", "custom_gradient_layer"]

...
# inflation_layer:
#   plugin: "nav2_costmap_2d::InflationLayer"
#   cost_scaling_factor: 3.0
#   inflation_radius: 0.55
...
custom_gradient_layer:
    plugin: "custom_nav2_costmap_plugin/GradientLayer"
    enabled: True
```

Veja aqui o arquivo [**planner_server.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/config/custom_costmap/planner_server.yaml).

E aqui, no mesmo procedimento, você substituiu a **inflation_layer** pela sua **custom_gradient_layer**. Você não precisaria adicionar nenhum dos outros plugins, pois estaria sobrescrevendo os dados deles, devido à forma como definiu o método **updateCosts()**.

Veja aqui o arquivo [**pathplanner_custom_costmap_plugin.launch.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/launch/pathplanner_custom_costmap_plugin.launch.py)

Altere os arquivos dos quais você obtém e carrega os parâmetros para os nós **controller_server** e **planner_server**.

```python
# Custom Plugin
controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'controller.yaml')
# Custom Plugin
planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'planner_server.yaml')
```

Altere o arquivo [**setup.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/setup.py):

Para encontrar a nova pasta **config/custom_costmap** e seus arquivos, adicione-os ao setup.py.

Adicionando esta linha de código:

```python
(os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
```

Compile e execute: `ros2 launch path_planner_server pathplanner_custom_costmap_plugin.launch.py`

Agora você deve ver algo parecido com isto:

![customgradientcostmap_nav2_plugins](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/customgradientcostmap_nav2_plugins.png)

Como você pode ver, o controlador não consegue mover o robô. Isso ocorre porque o Costmap tem uma configuração que não permite que o robô se mova livremente.

### Planner Plugin
Agora, você explorará um novo exemplo focado na criação de um plugin de planejamento que gera caminhos em linha reta sem considerar obstáculos. Este planejador simples servirá como base para a compreensão de como os planejadores de caminhos personalizados funcionam dentro da estrutura Nav2.

Como muitos dos conceitos fundamentais permanecem consistentes entre os diferentes tipos de plugins, este exemplo será um pouco menos detalhado do que o anterior. No entanto, seguindo os passos fornecidos, você ganhará experiência prática na implementação e integração de um planejador personalizado à pilha de navegação.

#### Crie um pacote
Crie um pacote ([**custom_nav2_planner_plugin**](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin)) onde você armazenará todos os plugins para planejadores. Execute os seguintes comandos:

```shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_planner_plugin --dependencies rclcpp rclcpp_action rclcpp_lifecycle std_msgs visualization_msgs nav2_util nav2_msgs nav_msgs geometry_msgs builtin_interfaces tf2_ros nav2_costmap_2d nav2_core pluginlib
cd ~/ros2_ws
colcon build --packages-select custom_nav2_planner_plugin
source install/setup.bash
```

Vamos dar uma olhada no arquivo [**straight_line_planner.hpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin/include/custom_nav2_planner_plugin/straight_line_planner.hpp).

Todos os métodos que você substituirá da classe base nav2_core::GlobalPlanner estão dentro da seção pública da sua classe:

```cpp
public:
  StraightLine() = default;
  ~StraightLine() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
```

* **configure()**: OBRIGATÓRIO. Este método é chamado quando o nó está no estado configure. Isso significa que é aqui que você deve carregar todos os parâmetros e inicializar todas as variáveis.
* **activate()**: OBRIGATÓRIO. Ao ativar este método, ele será executado, portanto, coloque tudo o que você precisa aqui antes que o plugin seja ativado.
* **deactivate()**: OBRIGATÓRIO.
* **cleanup()**: OBRIGATÓRIO. Ao entrar no estado clean, libera memória para tarefas e similares.
* **createPlan()**: OBRIGATÓRIO. Os métodos são chamados quando o servidor deseja gerar um caminho global com base em start e goal_pose. Este método retorna um nav_msgs::msg::Path que será usado como o caminho global planejado.

Agora, dê uma olhada no código do [**straight_line_planner.cpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin/src/straight_line_planner.cpp). Há basicamente dois métodos que valem a pena comentar:

**configure():**

```cpp
void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}
```

Aqui você obtém o valor do parâmetro **interpolation_resolution** que será usado para a geração do caminho em linha reta. Ele será definido no arquivo YAML:

```yaml
planner_plugins: ["GridBased"]    
GridBased:
  plugin: StraightLineCustomPlugin
  interpolation_resolution: 0.1
```

**createPlan():**

```cpp
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

        RCLCPP_WARN(
      node_->get_logger(), "Plann Straight line DONE");

  return global_path;
}
```

Aqui você pode ver:

* Os parâmetros de entrada para o método são **start e goal**.
* Você está calculando a distância entre esses dois pontos 2D e dividindo-a pela **interpolation_resolution**:

```cpp
int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
```

Isso nos dará o número de partes em que o caminho em linha reta será dividido. Quanto maior a distância, maior o número de partes que o caminho terá.

```cpp
for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }
```

E aqui, você está gerando os pontos desse caminho, usando incrementos lineares para gerar uma linha de caminho perfeitamente reta.

#### Crie o arquivo XML de informações do plugin
Veja aqui o arquivo [**straight_line_plugin_info.xml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin/straight_line_plugin_info.xml)

#### Configure o CMakelists.txt e o package.xml para compilação
Vamos analisar o [**CMakelists.txt**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin/CMakeLists.txt)

Mesmo procedimento, exceto para alguns elementos:

**target_compile_definitions**: Isso se deve à necessidade de desabilitar as funções BOOST ao compilar este plugin.

```txt
target_compile_definitions(${straight_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```

Diga ao pacote onde encontrar todas as informações relacionadas ao seu plugin no arquivo [**package.xml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_planner_plugin/package.xml)

#### Configure
Crie um novo lançamento do Pathplanner que carregue seu plugin personalizado. Para isso, crie um novo lançamento e alguns novos arquivos de configuração:

```shell
cd ~/ros2_ws/
touch ~/ros2_ws/src/nav2_pkgs/path_planner_server/launch/pathplanner_straightlineplanner_plugin.launch.py
mkdir ~/ros2_ws/src/nav2_pkgs/path_planner_server/config/straightline_planner
cd ~/ros2_ws/src/nav2_pkgs/path_planner_server/config/straightline_planner
touch planner_server.yaml
```

Aqui você só precisa do [**planner_server.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/config/straightline_planner/planner_server.yaml) porque seu plugin é para o servidor do planejador que gerou os caminhos globais.

* Como você pode ver, você não alterou o nome do parâmetro. Você usa o **GridBased**. Por enquanto, se isso for alterado, ocorrerão erros.
* Você está carregando o plugin com o nome que você deu a ele no **straight_line_plugin_info.xml**, **StraightLineCustomPlugin**.

```yaml
planner_plugins: ["GridBased"]    
GridBased:
  plugin: StraightLineCustomPlugin
  interpolation_resolution: 0.1
```

De uma olhada no [**pathplanner_straightlineplanner_plugin.launch.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/launch/pathplanner_straightlineplanner_plugin.launch.py).

Como você pode ver, você altera apenas o **planner_yaml**. O restante é a configuração padrão.

```py
# Custom Plugin
planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'straightline_planner', 'planner_server.yaml')
```

Para encontrar a nova pasta **config/straightline_planner** e seus arquivos, você precisa adicioná-los ao [**setup.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/setup.py).

Adicionando esta linha de código:

```py
(os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
```

Esta linha adiciona o config/straightline_planner e os arquivos internos.

Compile e execute: `ros2 launch path_planner_server pathplanner_straightlineplanner_plugin.launch.py`.

Agora você deve ver algo parecido com isto:

![rout](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/imagens/Capturar.PNG)

Como você pode ver, agora todos os caminhos são LINHAS RETAS.

Mesmo aqueles que ATRAVESSAM OBSTÁCULOS. Você pode ver que ele não planeja o caminho global em torno dele. MAS o robô para porque o Planejamento de Caminho LOCAL entra em ação. Mas esse local não se recupera bem porque, no final, é o Caminho Global que faz tudo funcionar e se adaptar.

### Controller Plugin
Neste exemplo final, você criará um plugin personalizado para o planejador local. Embora muitos planejadores locais já tenham sido implementados, este é baseado no algoritmo Pure Pursuit desenvolvido por Steve Macenski. Você pode [encontrar o código original aqui](https://github.com/ros-navigation/navigation2/tree/b92561bd8deba1589f5739ffffd71594adadc2f4/nav2_regulated_pure_pursuit_controller). Esta abordagem oferece uma nova perspectiva sobre o controle local, com o objetivo de aprimorar os métodos existentes. Ela é inspirada no artigo de pesquisa [SOURCE](https://www.enseignement.polytechnique.fr/profs/informatique/Eric.Goubault/MRIS/coulter_r_craig_1992_1.pdf).

Em vez de usar o controlador DWB tradicional, este planejador baseado em Pure Pursuit olha para a frente ao longo do caminho para gerar uma trajetória. O grau de aderência ao caminho depende da distância de observação à frente — uma distância maior resulta em um acompanhamento mais solto, enquanto uma distância menor mantém o robô mais alinhado com o caminho.

#### Crie um novo pacote ROS 2
Crie um novo pacote ([**custom_nav2_controller_plugin**](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_controller_plugin)) ROS 2 com os seguintes comandos:

```shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_controller_plugin --dependencies rclcpp geometry_msgs nav2_costmap_2d pluginlib nav_msgs nav2_util nav2_core tf2
cd ~/ros2_ws
colcon build --packages-select custom_nav2_controller_plugin
source install/setup.bash
```

Veja aqui o código [**regulated_pure_pursuit_controller.cpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_controller_plugin/src/regulated_pure_pursuit_controller.cpp).


Veja aqui o código [**regulated_pure_pursuit_controller.hpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_controller_plugin/include/custom_nav2_controller_plugin/regulated_pure_pursuit_controller.hpp).

Como antes, esta classe é baseada, neste caso, em **nav2_core::Controller**. Esta classe base possui vários métodos que precisam estar dentro do seu plugin para funcionar plug and play. Todos eles são OBRIGATÓRIOS, e alguns são os mesmos do nav2_core::GlobalPlanner usado no Exemplo anterior.

* configure(): OBRIGATÓRIO.
* activate(): OBRIGATÓRIO.
* deactivate(): OBRIGATÓRIO.
* cleanup(): OBRIGATÓRIO.
* setPlan(): OBRIGATÓRIO. Chame-o quando o plano global for atualizado.
* computeVelocityCommands(): OBRIGATÓRIO. Gera os comandos de velocidade enviados ao robô para seguir o plano global gerado no plugin global path planner. Retorna uma função geometry_msgs::msg::TwistStamped usada para mover o robô. Como parâmetros, possui a pose atual do robô e a velocidade atual.

#### Crie o arquivo XML de informações do plugin
Vamos analisar o arquivo [**regulated_pure_pursuit_controller_info.xml**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_controller_plugin/regulated_pure_pursuit_controller_info.xml)

Observe que isso é um pouco diferente. Também pode ser feito desta forma. Observe que:

* Você tem uma **class_libraries** extra. Isso está disponível em versões mais recentes do ROS2 e é recomendado.
* Não há <nome da classe>. Isso significa que, ao adicionar este plugin ao arquivo .yaml, o nome é o mesmo do tipo de classe. Neste caso, custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController.

#### Configure
Veja o arquivo de configuração [**controller.yaml*](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/config/pure_pursuit_controller/controller.yaml)

Como você pode ver, este plugin possui muitos PARÂMETROS. Mas os mais importantes que afetam o comportamento são:

* **desired_linear_vel**: Isso faz com que o robô se mova muito rápido. Neste caso, a velocidade é definida como 2,5 metros por segundo, o que é rápido.
* **lookahead_dist**: Isso regula a distância até a qual você verifica se há colisões no caminho. Quanto maior, mais suaves serão as trajetórias, mas menos precisas ao seguir o caminho. É limitado pelos parâmetros min_lookahead_dist e max_lookahead_dist.

Veja o arquivo [**pathplanner_purepursuit_controller_plugin.launch.py**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/nav2_pkgs/path_planner_server/launch/pathplanner_purepursuit_controller_plugin.launch.py)

Execute em um terminal: 

```shell
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch localization_server localization.launch.py
```

Execute em outro terminal: 

```shell
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch path_planner_server pathplanner_purepursuit_controller_plugin.launch.py
```

Agora você precisa acessar as Ferramentas Gráficas (que devem abrir automaticamente) e alterar o arquivo de configuração do RVIZ para planejamento de caminho. 

Vá em Arquivo -> Abrir Configuração e selecione o arquivo.

Você pode ver que o robô se move MUITO RÁPIDO. Isso porque você definiu a velocidade máxima de 2,5 m/s.

# Controller Server In Depth
O Servidor do Controlador é um componente-chave do Navigation2 (Nav2) que gerencia a execução do caminho gerando comandos de velocidade para o robô. Ele desempenha um papel vital para garantir uma navegação suave e eficiente, selecionando e executando plugins do controlador.

Nesta unidade, você analisará em detalhes como o Servidor do Controlador funciona, explorará sua arquitetura e analisará alguns aspectos críticos de sua funcionalidade. Você também examinará diferentes plugins do controlador e seu impacto no desempenho da navegação.

## Controller Server Config
O controller_server é onde as velocidades da roda são calculadas. Esta é a chave para resolver problemas quando o seu robô não se comporta corretamente. Portanto, você precisa dominar como ele funciona.

Se você verificar o arquivo de configuração do controller_server, verá a configuração geral dele no início:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
```

Esta parte indica os parâmetros básicos para controlar o controller_server. No final desta parte, você pode ver que três plugins importantes são usados:

* o **progress_checker_plugin**
* os **goal_checker_plugins**
* os **controller_plugins**

Você precisa dominá-los para adaptar o Nav2 às suas necessidades.

**IMPORTANTE**: Enquanto isso, o progress_checker_plugin requer a especificação de um único plugin. Os goal_checker_plugins e os controller_plugins podem conter mais de um plugin.

## Controller Server Plugins
O controller_server controla seu comportamento usando esses três plugins. Portanto, você deve especificá-los para sua tarefa de navegação.

### 1. progress_checker_plugin
Este plugin verifica se o robô está progredindo em direção à meta, ou seja, se o robô está apresentando alguma melhora em relação à meta.

Este plugin é chamado pelo controlador.

Atualmente, há apenas um plugin disponível no Nav2 como progress_check_plugin: o plugin **SimpleProgressChecker**.

```yaml
    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
```

O nav2_controller fornece o plugin atualmente disponível. Ele verifica se o robô se moveu uma determinada quantidade (especificada em **required_movement_radius**) em um determinado período de tempo (que você especifica em **movement_time_allowance**).

Este é um plugin simples. Ele não verifica se o movimento faz sentido ou se está em direção ao objetivo. Em vez disso, verifica se o robô se moveu pelo menos required_movement_radius metros em movement_time_allowance segundos.

Este plugin é básico. Você pode tentar implementar plugins mais complexos por conta própria. Revise a unidade sobre como criar seus próprios plugins para o Nav2.

### 2. goal_checker_plugins
Este plugin verifica se o robô atingiu a posição desejada dentro de tolerâncias específicas. Lembre-se de que diminuir demais as tolerâncias pode fazer com que o robô nunca atinja a posição desejada. Por outro lado, aumentar demais as tolerâncias pode fazer com que a posição final fique longe do esperado.

O nav2_controller fornece o plugin atualmente disponível. Atualmente (ROS2 Galactic), o Nav2 fornece duas implementações de goal_checker_plugins:

#### 1. SimpleGoalChecker
Veja um exemplo de parâmetros de configuração:

```yaml
    # Goal checker parameters
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25    # m
      yaw_goal_tolerance: 0.25   # rad
```

Verifique se o robô está dentro da faixa de tolerância indicada pelos parâmetros de tolerância. Em seguida, você pode especificar as tolerâncias para a posição e a orientação alcançadas.

#### 2. StoppedGoalChecker

```yaml
    # Goal checker parameters
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::StopedGoalChecker"
      xy_goal_tolerance: 0.25        # m
      yaw_goal_tolerance: 0.25       # rad
      trans_stopped_velocity: 0.25   # m/s
      rot_stopped_velocity: 0.25     # rad/s
```

Semelhante ao anterior, ele verifica se o robô está dentro das tolerâncias para a meta e se as velocidades (linear e rotacional) estão abaixo do valor indicado nos parâmetros.

### 3. controller_plugins
Eles são responsáveis ​​por calcular o plano local, as velocidades das rodas para seguir esse caminho e considerar os obstáculos próximos.

Atualmente, existem dois controller_plugins disponíveis:

#### 1. DWBLocalPlanner controller
Este controlador é fornecido pelo dwb_core. É uma implementação do algoritmo de prevenção de obstáculos DWA.

```yaml
    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      trajectory_generator_name: “dwb_plugins::StandardTrajectoryGenerator”
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

Para uma explicação completa de cada um dos parâmetros, consulte a [documentação oficial aqui](https://docs.nav2.org/configuration/packages/dwb-params/controller.html).

As partes mais importantes de sua configuração são explicadas aqui: o plugin gerador de trajetória e os críticos de trajetória.

##### Plugin Gerador de Trajetória
Este plugin implementa o cálculo da trajetória local. Atualmente, apenas dois plugins possíveis estão disponíveis:

**Definições da documentação oficial**

1. **StandardTrajectoryGenerator**: É semelhante ao algoritmo de implementação de trajetória usado no base_local_planner no ROS 1.
2. **LimitedAccelGenerator**: É semelhante ao DWA usado no ROS 1.

##### Críticas (Critics) de Trajetória do DWB
Críticas são pequenos plugins que afetam o comportamento do DWB para forçar mais ou menos o seguimento da trajetória global, especialmente diante de obstáculos inesperados que ainda não estão incluídos no Mapa de Custo global.

Cada crítica pontuará as diferentes trajetórias calculadas pelo plugin gerador de trajetórias.

**Definições da documentação oficial**

* **BaseObstacle**: Pontua uma trajetória com base em onde o caminho passa sobre o Mapa de Custo. Para usar isso corretamente, você deve usar a **camada de inflação** no Mapa de Custo para expandir os obstáculos pelo raio do robô.
* **GoalAlign**: Pontua uma trajetória com base em quão bem alinhada a trajetória está com a pose-alvo.
* **GoalDist**: Pontua uma trajetória com base em quão perto a trajetória leva o robô da pose-alvo.
* **ObstacleFootprint**: Pontua uma trajetória com base na verificação de que todos os pontos ao longo da pegada do robô não tocam em um obstáculo marcado no Mapa de Custo.
* **Oscillation**: Impede que o robô se mova para frente e para trás.
* **PathAlign**: Pontua uma trajetória com base em quão bem ela está alinhada ao caminho fornecido pelo planejador global.
* **PathDist**: Pontua uma trajetória com base em quão longe ela termina do caminho fornecido pelo planejador global.
* **PreferForward**: Pontua trajetórias que movem o robô para frente mais alto.
* **RotateToGoal**: Permite que o robô gire para a orientação do objetivo somente quando estiver suficientemente próximo do local do objetivo.
* **Twirling**: Impede que robôs holonômicos girem à medida que avançam em direção ao objetivo.

#### 2. Regulated Pure Pursuit
Este plugin foi desenvolvido especificamente para aplicações industriais onde a segurança em velocidade é fundamental. Ele regula as velocidades lineares pela curvatura da trajetória para ajudar a reduzir o excesso de velocidade em altas velocidades em curvas cegas, permitindo que as operações sejam muito mais seguras.

```yaml
FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
      use_interpolation: false
```

Confira a documentação completa desse plugin [aqui](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html).

#### 3. TEB planner
Mesmo que anunciado, este plugin voltado para robôs holonômicos ou semelhantes a carros ainda não está disponível.
