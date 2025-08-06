# Introdução

esse curso é voltado para a versão **Humble**.

# Como os robôs navegam

Os robôs navegam de uma forma que espelha a forma como os humanos se movem de um lugar para outro. Embora o processo pareça simples para nós, por ser instintivo, destrinchar o processo revela várias etapas essenciais.

Para se mover de um ponto a outro, um robô deve:

* Mapear o ambiente (Mapping) – O robô precisa criar ou acessar um mapa de seus arredores.
* Determinar sua localização (Localization) – Ele deve identificar onde está dentro do ambiente mapeado.
* Planejar uma rota (Path Planning) – O robô calcula a melhor maneira de se mover entre dois pontos.
* Executar o movimento evitando obstáculos (Controle e Desvio de Obstáculos) – Ele envia comandos para suas rodas (ou outros atuadores) para seguir o caminho planejado, detectando e evitando obstáculos em tempo real.

Desenvolver todos esses recursos do zero é complexo e demorado. É aí que entra o ROS (Sistema Operacional do Robô) – ele fornece pacotes de navegação pré-construídos que simplificam o processo.

Você poderá baixar um pacote de navegação [aqui](git clone https://bitbucket.org/theconstructcore/ros2_nav_files.git)

# Nav2
O Nav2 é usado no ROS2, sendo o sucessor do **ROS Navigation Stack**. Ele fornece um conjunto abrangente de ferramentas que permite que um robô navegue com segurança do Ponto A ao Ponto B.

O Nav2 inclui ferramentas para:

* Mapeamento – Carregar, servir e armazenar mapas do ambiente (Servidor de Mapas).
* Localização – Determinar a posição do robô no mapa (AMCL).
* Planejamento de Trajeto – Calcular um trajeto de A a B evitando obstáculos (Planejador Nav2).
* Execução de Trajeto – Controlar o robô conforme ele segue o trajeto planejado (Controlador Nav2).
* Reconhecimento de Obstáculos – Processar dados de sensores para criar uma representação do mundo com reconhecimento de obstáculos (Nav2 Costmap 2D).
* Comportamentos de Recuperação – Implementar estratégias de fallback em caso de falhas de navegação (Nav2 Recoveries).
* Gerenciamento do Ciclo de Vida – Gerenciar o ciclo de vida dos servidores de navegação (Nav2 Lifecycle Manager).
* Personalização – Ampliar a funcionalidade com plugins para algoritmos e comportamentos personalizados (Servidor Nav2 BT - behavior tree).

Abaixo, você encontrará um diagrama ilustrando como esses componentes interagem. Não se preocupe se parecer complexo no início — você entenderá completamente ao final deste curso.

![Nav2 Lifecycle Manager](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/architectural_diagram.png)

[Repositório oficial do NAV2.](https://github.com/ros-navigation/navigation2)

# Mapa
O mapeamento é uma parte fundamental da navegação de robôs. Antes que um robô possa se mover com eficiência em um ambiente, ele precisa compreender o espaço ao seu redor. Nesta unidade, você aprenderá a criar mapas que permitam que um robô navegue com segurança e inteligência.

Um mapa é uma representação do ambiente em que o robô opera. O robô depende de um mapa para:

* Localizar-se no ambiente.
* Planejar trajetórias para navegar com segurança de um ponto a outro.

Em ROS, um mapa é tipicamente um **mapa de grade de ocupação**, onde cada célula contém valores que indicam se uma área está **livre**, **ocupada** por um obstáculo ou **desconhecida**.

## SLAM
SLAM (Simultaneous Localization and Mapping) é uma técnica que permite a um robô criar um mapa de um ambiente desconhecido enquanto, simultaneamente, determina sua própria localização nele. Os algoritmos SLAM permitem que robôs explorem e naveguem sem conhecimento prévio do ambiente ao redor.

Algumas soluções SLAM amplamente utilizadas no ROS 2 incluem:

* Cartographer
* SLAM-Toolbox

Neste curso, você utilizará o Cartographer devido à sua simplicidade, confiabilidade e facilidade de compreensão.

O **Cartographer** é um sistema de Localização e Mapeamento Simultâneo (SLAM) em tempo real que funciona em 2D e 3D em diversas plataformas e configurações de sensores. Desenvolvido pelo Google, o Cartographer já foi usado no Google Street View para mapear interiores de edifícios.

Para integrar o Cartographer ao ROS, o pacote [**Cartographer_ros**](https://google-cartographer-ros.readthedocs.io/en/latest/) fornece um wrapper compatível com ROS.

## Iniciando o Cartographer
Agora você está pronto para criar um arquivo de inicialização para o Cartographer. O principal benefício de um arquivo de inicialização é que você pode iniciar vários nós a partir de um único arquivo e definir um parâmetro específico para cada nó ao iniciar um nó. Os parâmetros podem ser carregados de um arquivo YAML ou especificados no arquivo de inicialização.

Para iniciar o `Cartographer', inicie dois nós:

* cartographer_node.
* occupancy_grid_node

### cartographer_node
Estes são os campos que você precisa indicar na inicialização do nó:

* O cartographer_node é fornecido pelo pacote **cartographer_ros**
* O executável é chamado **cartographer_node**
* Os parâmetros necessários são:
    * **use_sim_time**: é um booleano que indica se o nó deve sincronizar seu horário com a simulação
* Os argumentos são:
    * **configuration_directory**: o diretório onde encontrar os arquivos de configuração
    * **configuration_basename**: o nome do arquivo de configuração

**launch:**

```python
    package='cartographer_ros', 
    executable='cartographer_node', 
    name='cartographer_node',
    output='screen',
    parameters=[{'use_sim_time': True}],
    arguments=['-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename]
```

**NOTA1**
Use a seguinte linha para obter o diretório de configuração dentro do seu arquivo de inicialização:

```python
cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
```

Essa linha tem duas funções interessantes:

* **os.path.join** faz a concatenação de dois caminhos para gerar o caminho final. Essa função é fornecida por **os** (que você deve importar).
* **get_package_share_directory** é uma função para encontrar o caminho completo no disco rígido de um determinado pacote ROS. Essa função é fornecida por **ament_index_python.packages** (que você deve importar).

Portanto, você precisa importar alguns elementos no seu arquivo de inicialização do Python:

```python
import os
from ament_index_python.packages import get_package_share_directory
```

### occupancy_grid_node
Estes são os campos que você precisa indicar na inicialização do nó:

* O **cartographer_occupancy_grid_node** é fornecido pelo pacote **cartographer_ros**
* O executável é chamado **cartographer_occupancy_grid_node**
* Os parâmetros necessários são:
    * **use_sim_time**: é um booleano que indica se o nó deve sincronizar seu tempo com a simulação
* Os argumentos são:
* **resolution**: número de metros por grade no mapa
* **publish_period_sec**: com que frequência (em segundos) o mapa é publicado no tópico /map

**launch:**

```python
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    output='screen',
    name='occupancy_grid_node',
    parameters=[{'use_sim_time': True}],
    arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
```

**NOTA2**
Você precisa incluir duas chamadas **Node()** dentro do arquivo de inicialização. Para isso, adicione uma após a outra, separadas por vírgulas.

```python
    Node(
        ...
    ),
    
    Node(
        ...
    ),
```

### EXEMPLO
a) Crie um novo pacote no ambiente de trabalho ros2_ws chamado **cartographer_slam** dentro do diretório **src/**: 
```shell
ros2 pkg create --build-type ament_python cartographer_slam --dependencies rclpy
```

b) Crie os diretórios de inicialização (**launch**) e configuração (**config**) em ros2_ws/src/cartographer_slam.
c) Escreva um arquivo de inicialização para iniciar o Cartographer com o nome **cartographer.launch.py**, onde os dois nós são iniciados.

```python
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    return LaunchDescription([
        
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ]) 
```

d) Crie um arquivo LUA chamado **cartographer.lua** no diretório config com os seguintes parâmetros de configuração:

```cartographer.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
```

e) Altere o arquivo **setup.py** para reconhecer o arquivo **cartographer.launch.py**.

```python
from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'cartographer_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

f) Compile, e execute o arquivo de inicialização recém-criado: `ros2 launch cartographer_slam cartographer.launch.py`

g) Inicie o **RVIZ2** para ver o mapa sendo criado. Você configurará o RVIZ para exibir os dados que deseja controlar.

h) Adicione a exibição do mapa no RVIZ e configure-o para visualizar o mapa que você está gerando.

1. Clique no botão **Add** em Exibições e escolha a exibição do **Map**.
2. Nas propriedades de exibição do mapa, defina o tópico como **/map**.

Se você não conseguir visualizar o mapa, verifique se os parâmetros de qualidade de serviço (**QoS**) do tópico /map estão corretos (como na figura). 

![map_qos_config](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/map_qos_config.png)

3. Adicione mais algumas exibições:

    * TF para ver os frames do robô
    * LaserScan para ver o laser colidindo com os objetos no mapa. Você também pode adicioná-lo facilmente acessando a aba "By topic" após clicar em "Add". Lembre-se de definir os parâmetros de QoS adequados para o laser. Especificamente, você precisa alterar LaserScan -> Topic -> Reliability Policy de "Reliable" para "Best Effort'" para que os dados do escaneamento a laser sejam exibidos.

![rviz2](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/rviz2_QoS.png)

h) Mova o robô pelo mundo do Gazebo usando o teclado teleop para criar um mapa completo do ambiente.

[Veja aqui](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Navigation%20(python)/exemplos/cartographer_slam) o pacote criado nesse exemplo.

### Salvando o map
Para salvar o mapa criado, você precisa executar o executável **map_saver**, que executa um nó map_saver do **nav2_map_server**.

IMPORTANTE: chame o nó dentro do diretório onde deseja salvar o mapa.

O comando é o seguinte: `ros2 run nav2_map_server map_saver_cli -f map_name`.

O comando de salvamento gerará dois arquivos:

* O arquivo de imagem map_name.pgm contém o mapa como uma imagem de grade de ocupação.
* O arquivo map_name.yaml contém detalhes sobre a resolução do mapa.

#### O arquivo YAML
O arquivo YAML gerado conterá os 7 campos a seguir:
* **Image**: Nome do arquivo que contém a imagem do Mapa gerado.
* **mode**: 
* **Resolution**: Resolução do mapa (em metros/pixel). 
* **origin**: Coordenadas do pixel inferior esquerdo no mapa. Essas coordenadas são fornecidas em 2D (x,y). O terceiro valor indica a rotação. Se não houver rotação, o valor será 0.
* **negate**: Inverte as cores do Mapa. Por padrão, branco significa completamente livre e preto significa completamente ocupado.
* **occupied_thresh**: Os pixels que possuírem um valor superior a este valor serão considerados como uma zona completamente ocupada.
* **free_thresh**: Pixels que tenham um valor menor que este valor serão considerados como uma zona completamente livre.

#### O arquivo de imagem (PGM - Portable Gray Map) 
Para visualizar o arquivo PGM você pode fazer o seguinte:
* Abra-o através do IDE. Para poder fazer isso, o arquivo deve estar no diretório catkin_ws/src.
* Abra-o através do Web Shell. Você pode usar, por exemplo, o editor **vi** digitando o comando `vi nome_do_mapa.pgm`.
* Baixe o arquivo e visualize-o no seu computador local com o seu próprio editor de texto.

## Fornecendo o mapa para outros nodes
Após criar o mapa com o **Cartographer** e salvá-lo com o **map_saver**, você pode fechar todos os programas anteriores. Você não precisará usar o Cartographer novamente (a menos que queira criar outros mapas).

O mapa criado precisa ser fornecido a outros aplicativos de navegação, como o **sistema de localização** ou o **planejador de rotas**. Para isso, inicie o map_server.

Você iniciará os seguintes nós para carregar o mapa e visualizá-lo no RVIZ:

* map_server
* nav2_lifecycle_manager

### map_server node
Estes são os campos que você precisa indicar na inicialização do nó:

* O **map_server** é fornecido pelo pacote **nav2_map_server**
* O executável é chamado **map_server**
* Os parâmetros necessários são:
    * **use_sim_time**: é um booleano que indica se o map_server deve sincronizar seu horário com a simulação.
    * **yaml_filename**: é o caminho completo para o arquivo yaml do mapa.

### lifecycle_manager node
Este nó gerencia o **ciclo de vida dos nós** envolvidos na navegação. Você aprenderá mais sobre ele posteriormente.

* O gerenciador de ciclo de vida é fornecido pelo pacote **nav2_lifecycle_manager**.
* O executável é chamado de **lifecycle_manager**.
* Parâmetros necessários:
    * use_sim_time: é um booleano que indica se o map_server deve sincronizar seu horário com a simulação.
    * autostart: é um booleano que indica se o gerenciador de ciclo de vida deve iniciar ao ser iniciado.
    * node_names: é uma lista com os nomes dos nós que o gerenciador de ciclo de vida deve cuidar. Até o momento, apenas o map_server.

[Veja nesse pacote](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Navigation%20(python)/exemplos/map_server) chamado **map_server** como fornecer um mapa criado.

# Lifecycle Nodes
É possível controlar o status de um nó de navegação e modificar seu status de execução.

Nós de navegação são o que chamamos de nós gerenciados (ou nós de ciclo de vida). Nós gerenciados podem ser facilmente controlados para serem reiniciados, pausados ou em execução. Os nós gerenciados controlam isso estando em qualquer um dos seguintes estados:

* Não configurado
* Inativo
* Ativo
* Finalizado

O diagrama a seguir indica como os nós transitam de um estado para outro.

![lifecycle](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/lifecycle.png)

O estado normal de um nó gerenciado deve ser **ativo**, que é quando o nó está executando seu código principal e suas funções de temporização.

Os nós gerenciados começam em um estado **não configurado**. Para fazer a transição do estado não configurado para o **ativo**, os nós precisam de um agente externo que os mova para o novo estado.

Vários nós no Nav2, como map_server, amcl, planner_server e controller_server, são habilitados para ciclo de vida, o que significa que **são nós gerenciados**. Esses nós fornecem as substituições necessárias das funções de ciclo de vida: **on_configure(), on_activate(), on_deactivate(), on_cleanup(), on_shutdown() e on_error()**.

## Gerenciador do Nav2 Lifecycle
No Nav2, o nó que ativa todos os nós de navegação é chamado de **nav2_lifecycle_manager**. O nav2_lifecycle_manager altera os estados dos nós gerenciados para realizar **inicialização, desligamento, reinicialização, pausa ou retomada controlada da pilha de navegação**.

(Veja abaixo uma figura da documentação oficial do Nav2 por Steve Macenski).

![diagram_lifecycle_manager](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/diagram_lifecycle_manager.png)

O Nav2 utiliza um wrapper do LifecycleNode, o **nav2_util LifecycleNode**. Este wrapper oculta muitas das complexidades dos LifecycleNodes para aplicações típicas. Ele também inclui uma conexão de vínculo para o gerenciador de ciclo de vida, garantindo que, após a transição de um nó para cima, ele também permaneça ativo. Quando um nó falha, ele informa o gerenciador de ciclo de vida e faz a transição para baixo no sistema para evitar uma falha crítica.

O nav2_lifecycle_manager fornece um serviço ROS, a partir do qual outros nós ROS podem invocar as funções de **inicialização, desligamento, redefinição, pausa ou retomada**. Com base nessa solicitação de serviço, o nav2_lifecycle_manager chama os serviços de ciclo de vida necessários nos nós gerenciados.

Você pode chamar esse serviço a partir dos seus programas para **reiniciar/desativar** a navegação de forma controlada.

O serviço fornecido pelo nav2_lifecycle_manager é chamado `<o nome do seu nó gerenciador de ciclo de vida>/manage_nodes`. O nav2_lifecycle_manager requer uma lista de nós a serem gerenciados. Veja este trecho de arquivo de inicialização como exemplo:

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
                                'bt_navigator']}])
```

Ele usará a lista node_names e a ordem nessa lista para identificar os nós a serem gerenciados, a **ordem** em que eles devem ser movidos para a inicialização (do primeiro para o último) e a ordem em que devem ser pausados/parados (do último para o primeiro).

Certifique-se de que este gerenciador tenha um parâmetro **{'autostart': True}** que indica o que o nav2_lifecycle_manager fará por padrão assim que os nós forem carregados na memória.

Vamos verificar o sistema de navegação da sessão anterior (mapa), do pacote maps_server. 

Agora verifique se o serviço fornecido pelo nav2_lifecycle_manager está em execução. Para isso, digite o seguinte comando em outro terminal: `ros2 service list | grep lifecycle`. 

```shell
/lifecycle_manager_mapper/describe_parameters
/lifecycle_manager_mapper/get_parameter_types
/lifecycle_manager_mapper/get_parameters
/lifecycle_manager_mapper/is_active
/lifecycle_manager_mapper/list_parameters
/lifecycle_manager_mapper/manage_nodes
/lifecycle_manager_mapper/set_parameters
/lifecycle_manager_mapper/set_parameters_atomically
```

Como você pode ver, o serviço **/lifecycle_manager_mapper/manage_nodes** é fornecido pelo **nav2_lifecycle_manager**.

Agora, vamos ver qual tipo de mensagem deve ser chamada:

```shell
nav2_msgs/srv/ManageLifecycleNodes
```

Agora, vamos ver como essa mensagem é composta com o seguinte comando:

```shell
uint8 STARTUP = 0
uint8 PAUSE = 1
uint8 RESUME = 2
uint8 RESET = 3
uint8 SHUTDOWN = 4

uint8 command
---
bool success
```

Ao chamar o serviço, você deve fornecer um número entre 0 e 4 para indicar o estado em que deseja colocar o sistema de navegação. O serviço retornará um valor booleano indicando se foi bem-sucedido.

Assim, por exemplo, você pode chamar esse serviço com a seguinte mensagem para pausar o sistema de navegação: `ros2 service call /lifecycle_manager_mapper/manage_nodes nav2_msgs/srv/ManageLifecycleNodes command:\ 1\`. Em seguida, pressione ENTER e você deverá obter a seguinte resposta:

```shell
requester: making request: nav2_msgs.srv.ManageLifecycleNodes_Request(command=1)

response:
nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)
```

E se você verificar o terminal onde você iniciou o map_server, você deverá ver uma mensagem como a seguinte aparecendo:

```shell
[lifecycle_manager-2] [INFO] [1655285986.303567128] [lifecycle_manager_mapper]: Terminating bond timer...
[lifecycle_manager-2] [INFO] [1655285986.303659326] [lifecycle_manager_mapper]: Pausing managed nodes...
[lifecycle_manager-2] [INFO] [1655285986.303698330] [lifecycle_manager_mapper]: Deactivating map_server
[map_server-1] [INFO] [1655285986.307166044] [map_server]: Deactivating
[map_server-1] [INFO] [1655285986.307227673] [map_server]: Destroying bond (map_server) to lifecycle manager.
[lifecycle_manager-2] [INFO] [1655285986.412322210] [lifecycle_manager_mapper]: Managed nodes have been paused
```

Agora **você não pode enviar metas de navegação** para o robô porque o sistema de navegação, mesmo em execução, está pausado. Se quiser retomar a navegação, chame o serviço novamente com o seguinte comando: `ros2 service call /lifecycle_manager_mapper/manage_nodes nav2_msgs/srv/ManageLifecycleNodes command:\ 2\`

O objetivo deste serviço é simplificar o gerenciamento do status de todo o sistema de navegação. Você pode adicionar funcionalidades para iniciar, pausar, parar e reconfigurar o sistema de navegação adicionando uma chamada de cliente de serviço, em vez de gerenciar todos os nós de uma só vez.

# Como localizar um robô em um ambiente
Após a criação do mapa, a próxima etapa essencial da navegação é a localização — determinar a posição e a orientação do robô nesse mapa. Sem uma localização precisa, o robô não consegue navegar com eficácia nem planejar caminhos de forma confiável.

## AMCL
O ROS possui um algoritmo muito robusto para localização, o **AMCL** (Adaptive Monte-Carlo Localization). É um sistema de localização probabilística para um robô se movendo em 2D.

Ele implementa a abordagem de localização adaptativa (ou amostragem KLD) de Monte Carlo (conforme descrito por Dieter Fox), que utiliza um filtro de partículas para rastrear a pose de um robô em relação a um mapa conhecido.

Um robô ROS é localizado quando alguém publica uma **transformação entre o quadro /map e o quadro /odom**.
Isso significa que o quadro /odom do robô conhece sua posição relativa ao quadro /map. Portanto, o robô conhece sua posição no mapa, pois seu quadro /base_link está diretamente conectado ao quadro /odom.

Quando tudo estiver correto, o **AMCL é quem publica essa transformação**.

## Iniciando o AMCL com um arquivo launch
Você precisa iniciar três nós:

* O **map_server** fornece o mapa para o algoritmo de localização.
* O algoritmo de localização (**amcl**).
* O gerenciador de ciclo de vida (**life cycle manager**).

### Inicialização do nó map_server
Estes são os campos que você precisa indicar na inicialização do nó:

* O map_server é fornecido pelo pacote **nav2_map_server**
* O executável é chamado **map_server**
* Os parâmetros necessários são:
    * **use_sim_time**: é um booleano que indica se o map_server deve sincronizar seu horário com a simulação.
    * **yaml_filename**: é o caminho completo para o arquivo yaml do mapa.

```python
package='nav2_map_server',
executable='map_server',
name='map_server',
output='screen',
parameters=[{'use_sim_time': True}, 
            {'yaml_filename':map_file} 
            ]),
```

### Inicialização do nó amcl
Estes são os campos que você precisa indicar na inicialização do nó:

* O amcl é fornecido pelo pacote **nav2_amcl**
* O executável é chamado **amcl**
* O parâmetro necessário é:
    * o arquivo **yaml** que contém todos os parâmetros de configuração do nó

```python
package='nav2_amcl',
executable='amcl',
name='amcl',
output='screen',
parameters=[nav2_yaml]
```

### Inicialização do nó lifecycle_manager
Este nó gerencia o ciclo de vida dos nós envolvidos na navegação.

* O gerenciador de ciclo de vida é fornecido pelo pacote **nav2_lifecycle_manager**.
* O executável é chamado de **lifecycle_manager**.
* Parâmetros necessários:
    * **use_sim_time**: é um booleano que indica se o map_server deve sincronizar seu horário com a simulação.
    * **autostart**: é um booleano que indica se o gerenciador de ciclo de vida deve iniciar ao ser iniciado.
    * **node_names**: é uma lista com os nomes dos nós que o gerenciador de ciclo de vida deve gerenciar.

```python
package='nav2_lifecycle_manager',
executable='lifecycle_manager',
name='lifecycle_manager_localization',
output='screen',
parameters=[{'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server','amcl']}])
```

[Nesse exemplo, do pacote **localization_server**](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Navigation%20(python)/exemplos/localization_server) foi um arquivo de inicialização (localization.launch.py) que inicie o sistema de localização para o robô simulado usando o mapa que você criou na sessão **mapa**.

Compile e execute o exemplo: `ros2 launch localization_server localization.launch.py`. Se o sistema de localização tiver sido iniciado corretamente, você deverá ver na tela uma mensagem como esta:

```shell
[amcl-2] [WARN] [1632242660.692356589] [amcl]: ACML cannot publish a pose or update the transform. Please set the initial pose...
```

Essa mensagem está correta. O sistema de localização está aguardando a posição inicial do robô. Isso significa que, uma vez iniciado o sistema de localização, o algoritmo não sabe onde posicionar o robô inicialmente no mapa. Ele precisa que alguém indique isso para que o algoritmo possa começar a rastrear o robô enquanto ele se move.

#### **Manually provide the initial localization**
Para fornecer a localização inicial manualmente, siga os próximos passos:

1. Inicie o RVIZ para visualizar e definir a localização: `rviz2`. Use o arquivo **default.rviz** salvo no pacote localization_server.
2. Defina a pose inicial do robô no mapa. Clique em **2D pose estimate** para localizar o robô no ambiente e, em seguida, clique no ponto do mapa que corresponde à posição real do robô na simulação.

Ao definir a Estimativa de Pose, todo o sistema AMCL começará a localizar o robô. Isso significa que ele começará a publicar a **transformação entre o mapa e o odom**. Todo o sistema estará ativo!

Você deverá ver a seta do centro do robô até o quadro do mapa. Essa é a transformação de odom para quadros do mapa que a AMCL publica.

Em seguida, mova o robô com o teclado e observe como a localização se adapta à nova localização do robô. Você deverá ver as **partículas** se concentrarem na localização mais provável do robô.