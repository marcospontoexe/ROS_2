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

[**CMakelists.txt**](https://github.com/marcospontoexe/ROS_2/blob/main/Advanced%20ROS2%20Navigation%20(python)/exemplos/custom_nav2_costmap_plugin/CMakeLists.txt):