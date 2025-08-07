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

#### **Como definir a localização inicial do robô a partir do arquivo de configuração**
Toda vez que seu robô começar a trabalhar, você deve indicar sua localização inicial.

[Nesta launch (**localization_and_initial_pose.launch.py**)](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/exemplos/localization_server/launch/localization_and_initial_pose.launch.py), é carregado um arquivo (**amcl_config_initialized.yaml**) com as definições da pose inicial do robô durante a inicialização.

#### **Como definir a localização inicial do robô a partir da linha de comando**
O nó AMCL fornece um tópico no qual você pode publicar a pose inicial desejada para o robô. O tópico é chamado **/initialpose**.

Publique essas coordenadas no tópico /initialpose com o seguinte comando: `ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"`

#### **Como definir a localização inicial do robô programaticamente**
[Nessa launch **init_robot.launch.py**](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/exemplos/localization_server/launch/init_robot.launch.py) é carregado os parametros **amcl_config.yaml** (sem definição de pose inicial) e executado o programa **initial_pose_pub.py**, que ao receber a posição pelo tópico **/clicked_point** define a **posição inicial** do robo.

Para enviar uma posição para o tópico **/clicked_point**: `ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: -0.05768, y: -0.0388262, z: 0.0}}"`

## Localização global
A localização global é usada quando nem a pessoa nem o robô sabem onde o robô está localizado no mapa. O robô deve então tentar se localizar. Com a localização global, o robô tentará identificar sua localização no mapa.

Para usar a localização global, são necessárias duas etapas:

1. Distribuir todas as partículas do filtro pelo mapa. Esta etapa é realizada chamando o serviço **/reinitialize_global_localization**: `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`.
2. Movimentar o robô até que ele detecte sua localização. Esta etapa pode ser realizada movendo o robô aleatoriamente, evitando obstáculos. Se o ambiente for simples, isso também pode ser feito girando o robô em sua posição (solução segura).

# Planejamento de trajetórias (Path Planning)
O Planejamento de Trajetória é o processo de calcular uma trajetória de um ponto (Ponto A) a outro (Ponto B), evitando obstáculos e garantindo que o robô chegue ao seu destino com segurança. É um componente essencial da navegação autônoma e permite que o robô se mova de um lugar para outro com precisão.

# Iniciando (launh) Path Planning
Para iniciar o sistema de Planejamento de Caminhos no ROS 2, vários nós devem ser iniciados. Além disso, certos nós pré-requisitos são necessários para o funcionamento adequado.

Antes de iniciar o sistema de Planejamento de Caminhos, certifique-se de que os seguintes nós estejam em execução:

* **map_server** – Fornece o mapa para navegação.
* **amcl** – Lida com a localização adaptativa de Monte Carlo.

Para habilitar o planejamento de caminho, inicie os seguintes nós:

* **planner_server** – Calcula o caminho ideal (planejador de rotas globais).
* **controller_server** – Gera comandos de movimento para seguir o caminho planejado (planejador de rotas locais).
* **behaviour_server** – Gerencia ações de recuperação em caso de falhas de navegação.
* **bt_navigator** – Executa lógica de navegação de alto nível usando árvores de comportamento.
* **nav2_lifecycle_manager** – Gerencia o ciclo de vida dos nós de navegação.

## Iniciando o planejador de rotas globais (planner_server)
O **Nav2 planner** é equivalente ao Global Planner do ROS1. Sua tarefa é encontrar um caminho para o robô do Ponto A ao Ponto B.

Ele calcula o caminho evitando os obstáculos conhecidos incluídos no mapa. O cálculo do caminho é iniciado assim que o robô recebe uma **2d_Goal_Pose**.

O planejador também tem acesso a uma representação ambiental global (Mapa de Custo Global) e dados de sensores.

Atualmente (Galactic), apenas um algoritmo do Planner está disponível no ROS2, o **Nav2Fn_Planner**.

Os campos que você precisa indicar na inicialização do nó são:

1. O pacote **nav2_planner** fornece o controlador (controller_server).
2. O executável é chamado **planner_server**.
3. Os parâmetros necessários são:
    * O arquivo **yaml** que contém todos os parâmetros de configuração do nó.

```python
package='nav2_planner',
executable='planner_server',
name='planner_server',
output='screen',
parameters=[nav2_yaml])

```

## Iniciando o controlador (controller_server)
O controlador ROS Nav2 é equivalente ao Planejador Local do ROS1. Sua principal tarefa é realizar o planejamento reativo da trajetória a partir da posição atual até alguns metros à frente (até o alcance dos sensores). Em seguida, ele constrói uma trajetória para evitar os obstáculos dinâmicos (que não aparecem no mapaglobal, mas podem ser detectados com a ajuda dos dados dos sensores), enquanto tenta seguir o plano global.

Ele também é responsável por gerar os comandos do volante, para fazer o robô seguir a trajetória.

Atualmente, existem apenas dois planejadores locais disponíveis no ROS2:

* **dwb_controller**: geralmente usado com robôs com acionamento diferencial
* **Controlador TEB**: geralmente usado com robôs do tipo carro (este ainda não funciona corretamente no Galactic)

Os campos que você precisa indicar na inicialização do nó são:

* O pacote **nav2_controller** fornece o controlador
* O executável é chamado **controller_server**
* Os parâmetros necessários são:
    * O arquivo **yaml** que contém todos os parâmetros de configuração do nó

```python
name='controller_server',
package='nav2_controller',
executable='controller_server',
output='screen',
parameters=[controller_yaml])
```

### Progress Checker
Verifica se o robô está preso ou progrediu em direção à conclusão da meta.

### Goal Checker
Verifica se o robô atingiu a pose desejada.

## Iniciando o bt_navigator
Você já viu o nó que calcula o planejamento do caminho e o nó que gera os comandos de roda para seguir esse caminho.

Em seguida, temos o nó que coordena o nó que chama o nó planejador de caminho, solicitando um caminho e, em seguida, chama o controlador para mover o robô ao longo dele. Esse nó é o **bt_navigator**.

Estes são os campos que você precisa indicar na inicialização do nó:

* O **bt_navigator** é fornecido pelo pacote **nav2_bt_navigator**
* O executável é chamado **bt_navigator**
* Os parâmetros necessários são: 
    * O arquivo yaml que contém todos os parâmetros de configuração do bt_navigator

```python
package='nav2_bt_navigator',
executable='bt_navigator',
name='bt_navigator',
output='screen',
parameters=[bt_navigator_yaml])
```

**IMPORTANTE**: O comportamento do bt_navigator é definido por um arquivo XML que contém a árvore de comportamento correspondente a esse comportamento. Essa árvore de comportamento inclui quando chamar o planejador de caminho, quando chamar o controlador e quando ativar um comportamento de recuperação.

Você pode encontrar um exemplo abaixo. 

## Iniciando recoveries_server
O que acontece se o robô não conseguir encontrar um caminho válido para o objetivo fornecido? E se o robô ficar preso no meio do processo e não conseguir descobrir o que fazer?

Nessas situações, o robô usa **comportamentos de recuperação**. Esses movimentos simples e predefinidos geralmente resolvem a situação, chamados por meio de **árvores de comportamento**.

O **recoveries_server** é o nó responsável por executar os comportamentos de recuperação. Por exemplo, o **bt_navigator** chamará o recoveries_server quando acreditar que o robô está preso.

Os campos que você precisa indicar na inicialização do nó são:

* O pacote **nav2_behaviors** fornece o controlador
* O executável é chamado **behavior_server**
* Os parâmetros necessários são: 
    * o arquivo **yaml** que contém todos os parâmetros de configuração do behavior_server


```python
package='nav2_behaviors',
executable='behavior_server',
name='recoveries_server',
parameters=[recovery_yaml],
output='screen'),
```

Atualmente, existem três comportamentos de recuperação disponíveis no Nav2:

* **spin** - executa uma rotação no local em um determinado ângulo
* **backup** - executa uma translação linear em uma determinada distância
* **wait** - leva o robô a um estado estacionário

[Neste pacote criado (**path_planner_server**)](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Navigation%20(python)/exemplos/path_planner_server) é carregado o mapa e o amcl usando o lifecicle_mananger nomeado como **lifecycle_manager_localization**, e carregado os nós necessários para o planning_path usando o lifecicle_mananger nomeado como **lifecycle_manager_pathplanner**.

Agora que tudo está funcionando, tente definir uma meta de navegação para o robô:

1. Execute a launch do pacote: `ros2 launch path_planner_server pathplanner.launch.py`.
2. Abra o arquivo **default.rviz** encontrado no pacote path_planner_server, e clique no botão RVIZ **2d Goal Pose** e, em seguida, clique em qualquer ponto do mapa para direcionar seu robô. Você deverá ver uma linha verde indicando o caminho que o robô seguirá, da sua localização atual até o objetivo.

## Enviando um meta de navegação por linha de comando
Há duas maneiras de enviar uma meta de navegação pela linha de comando: usando o **servidor de ações** ou o **tópico**. Ambos os métodos são semelhantes e podem ser usados quando você precisar de feedback sobre o resultado.

### Com um servidor de ações
Como o sistema de navegação usa um servidor de ações chamado **/navigate_to_pose** para receber metas do RVIZ, você pode usar a linha de comando para chamar o servidor de ações e fornecer uma meta ao robô. Use o seguinte comando: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: -0.0483484, y: 3.77634, z: 0}, orientation:{x: 0.0, y: 0.0, z: 0.554863, w: 0.831942}}}"`.

Você precisa alterar os valores de posição e orientação da mensagem para o local no mapa para onde deseja enviar o robô (lembre-se de que a orientação requer um quatérnio). Você pode usar o RVIZ para identificar os valores de pose do local.

### Com um tópico
Use o **ros2 topic pub** para publicar essas coordenadas no tópico **/goal_pose**: `ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: -0.0483484, y: 3.77634, z: 0}, orientation: {w: 0.831942}}}"`.

# Desviando de obstáculos
Na navegação autônoma, evitar obstáculos é crucial para garantir que um robô possa navegar com segurança pelo ambiente.

## Mapa de custo (Costmap)
Um Mapa de Custo é uma representação 2D de obstáculos detectados por robôs em um mapa de grade.

![nav2_world_local_costmap](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/nav2_world_local_costmap.png)

* Cada célula da grade contém informações sobre os obstáculos detectados pelos sensores.
* O custo da célula pode ser desconhecido, livre, ocupado ou inflado.
* Cores diferentes indicam a probabilidade de colisão com um obstáculo.
* Essas informações são então usadas pelos controladores, planejadores e equipes de recuperação para calcular suas tarefas com segurança e eficiência.

Existem dois tipos de Mapas de Custo:

* Um Mapa de Custo **Global** é gerado a partir dos obstáculos no mapa estático. É o mapa usado pelo planejador global (**planner_server**) para gerar o caminho de longo prazo.

* Um Mapa de Custo **Local** é criado a partir de novos obstáculos detectados pelo robô em uma pequena área ao redor do robô. O controlador (**controller_server**) o utiliza para gerar o caminho de curto prazo e evitar obstáculos dinâmicos.

Diferenças entre Mapas de Custo Global e Local:

* O Mapa de Custo Global ajuda a evitar obstáculos conhecidos no mapa, enquanto o Mapa de Custo Local é usado para evitar obstáculos dinâmicos, que não estão presentes no mapa global.
* O Mapa de Custo Global cobre todo o mapa, enquanto o Mapa de Custo Local cobre uma pequena área ao redor do robô.
* O Mapa de Custo Global é estático sobre o mapa, enquanto o Mapa de Custo Local rola sobre o Mapa de Custo Global conforme o robô se move pelo espaço.

## Adicionando um mapa de custo global na navegação
Para adicionar um mapa de custos global, adicione a seguinte série de parâmetros ao arquivo de configuração do planner:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.15
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
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
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      always_send_full_costmap: True
```

No rviz, adicione uma exibição de **map** para o tópico **/global_costmap/costmap**.

### obstacle layers (camadas)
O mapa de custo global é gerado como a superposição de diferentes camadas de obstáculos. Cada camada adiciona um conjunto de obstáculos ao mapa de custo global com base em como a camada os calcula.

Camadas de mapa de custo disponíveis:

* Camada estática (Static Layer) - Adiciona como obstáculos ao mapa de custo global qualquer ponto preto existente no mapa estático
* Camada de inflação (Inflation Layer) - Adiciona uma inflação a qualquer obstáculo no mapa de custo global, como uma distância segura a ser mantida
* Camada de obstáculo (Obstacle Layer) - Adiciona ao mapa de custo global qualquer objeto detectado por um sensor 2D
* Camada de voxel (Voxel Layer) - Adiciona ao mapa de custo global obstáculos 3D a partir de dados da nuvem de pontos

Você pode especificar uma ou todas as camadas anteriores para aplicar ao mapa de custo global. No entanto, para incluí-las, você precisa adicioná-las à lista de plugins e, em seguida, incluir os parâmetros de configuração de cada uma.

```txt
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

**MUITO IMPORTANTE**: Inclua o "**obstacle_layer**" na lista de plugins antes do "**inflation_layer**". Caso contrário, os obstáculos recém-detectados não serão inflados pelo plugin "inflation_layer".

#### Static Layer
A camada do mapa estático representa uma parte praticamente inalterada do Costmap global, como aquelas geradas pelo SLAM.

#### Inflation Layer
A camada de inflação é uma otimização que adiciona novos valores em torno de obstáculos letais (ou seja, infla os obstáculos) para fazer com que o Mapa de Custo represente o espaço de configuração do robô.

#### Obstacle Layer
O **ObstacleCostmapPlugin** marca e traça raios em obstáculos em duas dimensões, enquanto o **VoxelCostmapPlugin** faz isso em três dimensões.

#### Voxel layer
A camada de voxel inclui obstáculos detectados em 3D nos Costmaps 2D.

```yaml
    voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: pointcloud
        combination_method: 1
        pointcloud:  # no frame set, uses frame from message
          topic: /intel_realsense_r200_depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

## Adicionando um mapa de custo local na navegação
Para adicionar um mapa de custos local, você deve adicionar uma série de parâmetros ao arquivo de configuração do controller_server:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 1
      height: 1
      resolution: 0.05
      robot_radius: 0.15
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
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
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
```

Como você pode ver, o mapa de custos local contém a mesma estrutura e conjunto de parâmetros do mapa de custos global. Isso faz todo o sentido, visto que este mapa de custos é apenas uma versão reduzida e mais dinâmica do global.

No rviz, adicione uma exibição de **map** para o tópico **/local_costmap/costmap**. Selecione o Color Scheme como Costmap.

# Navegação multi robo
À medida que você avança na navegação do robô, você pode encontrar cenários em que vários robôs operam no mesmo ambiente.

Se você tiver um ambiente multi-robô, cada robô provavelmente terá o mesmo nome de tópico /cmd_vel, o mesmo nome de quadro base_footprint e até mesmo o mesmo nome de nó de controle. Isso é incorreto para ambientes multi-robô, pois o algoritmo de navegação não consegue identificar com qual robô está se comunicando.

Primeiro, configure corretamente seus robôs e separe tópicos, frames e nomes de nós em namespaces. Portanto, no restante desta unidade, você assumirá que os namespaces separam seus robôs corretamente.

O namespace é o rótulo que você atribuirá a cada robô (por exemplo, tb3_0, tb3_1, etc.). Portanto, cada robô pode ter os mesmos tópicos, quadros e nomes de nós, mas todos eles começarão com um namespace exclusivo que o identifica.

A configuração dos robôs para o uso adequado dos namespaces é feita em:

1. Em robôs simulados: no arquivo de descrição URDF e nos plugins Gazebo
2. Em robôs reais: na rotina de inicialização fornecida pelo fornecedor

Se a URDF ou os plugins não forem criados corretamente para permitir namespaces, ou se a inicialização não for criada corretamente, você poderá ter dificuldades para inicializar seus robôs no modo de namespaces.

IMPORTANTE: É obrigatório que seus robôs já trabalhem separados por namespaces. Neste curso, fizemos isso para você. No entanto, se você quiser aplicar esta unidade ao seu próprio robô, pode fazer isso sozinho.

Certifique-se de que cada um dos seus robôs tenha um nome específico para:

* Tópicos
* Quadros
* Nós

Pode ser necessário alterar o modelo URDF do seu robô e a configuração dos seus drivers (para sensores e atuadores) para incluir os namespaces neles.

Por exemplo, se existir dois nameSpace **tb3_0** e **tb3_1**, ao listar os nós existentes com o comando `ros2 node list` verá o seguinte:

```shell
/gazebo
/tb3_0/robot_state_publisher
/tb3_0/turtlebot3_diff_drive
/tb3_0/turtlebot3_imu
/tb3_0/turtlebot3_joint_state
/tb3_0/turtlebot3_laserscan
/tb3_1/robot_state_publisher
/tb3_1/turtlebot3_diff_drive
/tb3_1/turtlebot3_imu
/tb3_1/turtlebot3_joint_state
/tb3_1/turtlebot3_laserscan
```

Como você pode ver, cada nó relacionado a um robô começa com o namespace desse robô.

Se listar os tópicos (`ros2 topic list`):

```shell
/clock
/parameter_events
/performance_metrics
/rosout
/tb3_0/cmd_vel
/tb3_0/imu
/tb3_0/joint_states
/tb3_0/odom
/tb3_0/robot_description
/tb3_0/scan
/tb3_1/cmd_vel
/tb3_1/imu
/tb3_1/joint_states
/tb3_1/odom
/tb3_1/robot_description
/tb3_1/scan
/tf
/tf_static
```

Como você pode ver, cada robô tem uma IMU diferente, odom, scan, cmd_vel, robot_description e um tópico de joint_states, separados pelo namespace do robô

Imprima o TF completo para este sistema ROS2 com dois robôs (`cd ~/ros2_ws/src; ros2 run tf2_tools view_frames`):

![two_robots_tf_v2](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/imagens/two_robots_tf_v2.png)

## Mapeamento do ambiente
Para que todos os robôs se movam, configurem e iniciem de forma autônoma um sistema de navegação completo PARA CADA ROBÔ. Isso significa um sistema de localização, um controller_server, um planner_server, um navigator, etc., para cada robô.

O único sistema comum a todos os robôs é o map_server. Haverá um único map_server para todos os robôs, pois todos usarão o mesmo mapa. Portanto, comece iniciando-o.

Mesmo que seja possível fazer com que vários robôs colaborem para criar um único mapa para todos eles, esta seção não abordará este tópico, por se tratar de um assunto mais complexo.

O procedimento que você usará é o seguinte:

1. Um único robô será responsável por criar o mapa.
2. Após a criação do mapa, um único map_server será iniciado usando esse mapa.
3. Todos os sistemas de navegação de cada robô usarão o mesmo mapa, solicitando o mesmo map_server.

Por exemplo, para criar um mapa usando o cartographer, modifique o arquivo de configuração para que ele agora use os dados do laser, os dados de odometria, o tracking_frame, o published_frame e o odom_frame do robô **tb3_0**.

```lua
tracking_frame = "tb3_0/base_footprint",
published_frame = "tb3_0/odom",
odom_frame = "tb3_0/odom",
```

Pode ser necessário remapear os tópicos no **cartographer_node**:

```python
remappings=[
            ('/cmd_vel', '/tb3_0/cmd_vel'),
            ('/odom', '/tb3_0/odom'),
            ('/scan', '/tb3_0/scan'),],
```

Para mover o robo tb3_0 e realizar o mapeamento, use o teleop com o tópico **cmd_vel** remapeado para **tb3_0/cmd_vel**: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tb3_0/cmd_vel`

[Veja nessa launch (**multi_cartographer.launch.py**)](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation%20(python)/exemplos/cartographer_slam/launch/multi_cartographer.launch.py) como deve ser remapeado os tópicos no cartographer_node, e como carregar as configurações do arquivo .lua do cartographer para um robo com nameSpace configurado.

## Localizando
Agora, inicie um sistema de localização para cada robô. Isso significa iniciar dois nós amcl, cada um configurado apropriadamente para o robô que deve operar.

### PASSO 1 - Adicionar uma variável de namespace aos launch de nós amcl
* Você precisa iniciar dois nós amcl.
* Para cada inicialização de nó, adicione um argumento namespace com o namespace do robô ao qual esse nó amcl corresponde.
* Um namespace é um identificador único sob o qual todos os tópicos, quadros TF e dados de sensores serão encontrados.
* É amplamente utilizado em robótica, especialmente em enxames de robôs do mesmo modelo.

**IMPORTANTE**: Ao adicionar um namespace, o nó iniciado modificará todos os seus tópicos, nome do nó e serviços, anexando o namespace ao início dos nomes.

**IMPORTANTE 2**: Adicionar um namespace no arquivo de inicialização não modificará automaticamente os **quadros** indicados no arquivo de configuração. Estes devem ser modificados manualmente no próprio arquivo de configuração.

Exemplo de launch para o robô tb3_0:

```python
Node(
    namespace='tb3_0',
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[tb3_0_config]
),
```

O nó iniciado com esse código não será chamado de amcl. Em vez disso, será chamado de ***tb3_0/amcl*** e o outro de ***tb3_1/amcl***. O mesmo acontecerá com seus tópicos e serviços.

[Veja nessa launch (**multi_localization.launch.py**)]() como configurar o nameSpace para o amcl de cada robo, e como fornecer esses nós criados para o lifecycle_manager. **IMPORTANTE**, Observe que você está adicionando um novo parâmetro **{'bond_timeout':0.0}**, necessário para evitar erros de inicialização. Para evitar erros, especifique o **topic_name** e o **frame_id** do **map_server**.

### PASSO 2 - Crie um arquivo de configuração específico para cada robô
Cada robo deverá ter seu pŕoprio arquivo de configurações do sistema de localização. Por tanto todos os quadros devem ser modificados para incluir o namespace. O único quadro que não precisa ser alterado é o **global_frame_id**, pois existe um único quadro global para todos os robôs (aquele no mapa).

Os **tópicos não precisam ser modificados** porque o argumento namespace do arquivo de inicialização os modifica automaticamente. **Isso não se aplica a quadros**.

O parâmetro **map_topic** deve ser forçado para **/map**. Se você não o forçar com `/`, ele se conectará automaticamente a tb3_0/map, que não é o tópico publicado pelo servidor de mapas.

[Veja]() os parametros do arquivo de configuração do amcl do tb3_0
[Veja]() os parametros do arquivo de configuração do amcl do tb3_1

Ao compilar e executar a launch **multi_localization.launch.py**, você deverá receber uma mensagem indicando que ambos os nós amcl aguardam a posição inicial. Você usará o RVIZ para inicializar os robôs.