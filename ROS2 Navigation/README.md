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

[Nav2 Lifecycle Manager](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation/imagens/architectural_diagram.png)

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

### occupancy_grid_node
Estes são os campos que você precisa indicar na inicialização do nó:

* O **occupancy_grid_node** é fornecido pelo pacote **cartographer_ros**
* O executável é chamado **cartographer_occupancy_grid_node**
* Os parâmetros necessários são:
    * use_sim_time: é um booleano que indica se o nó deve sincronizar seu tempo com a simulação
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

### **NOTA1**
Use a seguinte linha para obter o diretório de configuração dentro do seu arquivo de inicialização:

```python
cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
```

Essa linha tem duas funções interessantes:

* **os.path.join** faz a concatenação de dois caminhos para gerar o caminho final. Essa função é fornecida por os (que você deve importar).
* **get_package_share_directory** é uma função para encontrar o caminho completo no disco rígido de um determinado pacote ROS. Essa função é fornecida por **ament_index_python.packages** (que você deve importar).

### **NOTA2**
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

```

b) Crie os diretórios de inicialização (**launch**) e configuração (**config**) em ros2_ws/src/cartographer_slam.

c) Escreva um arquivo de inicialização para iniciar o Cartographer com o nome **cartographer.launch.py**, onde os dois nós são iniciados.

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

f) Execute o arquivo de inicialização recém-criado: `ros2 launch cartographer_slam cartographer.launch.py`

g) Inicie o **RVIZ** para ver o mapa sendo criado. Você configurará o RVIZ para exibir os dados que deseja controlar.

h) Adicione a exibição do mapa no RVIZ e configure-o para visualizar o mapa que você está gerando.

1. Clique no botão **Add** em Exibições e escolha a exibição do Mapa.
2. Nas propriedades de exibição do mapa, defina o tópico como **/map**.

Se você não conseguir visualizar o mapa, verifique se os parâmetros de qualidade de serviço (**QoS**) do tópico /map estão corretos (como na figura). S

![map_qos_config](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation/imagens/map_qos_config.png)

3. Adicione mais algumas exibições:

    * TF para ver os frames do robô
    * LaserScan para ver o laser colidindo com os objetos no mapa. Você também pode adicioná-lo facilmente acessando a aba "By topic" após clicar em "Add". Lembre-se de definir os parâmetros de QoS adequados para o laser. Especificamente, você precisa alterar LaserScan -> Topic -> Reliability Policy de "Reliable" para "Best Effort'" para que os dados do escaneamento a laser sejam exibidos.

h) Mova o robô pelo mundo do Gazebo usando o teclado teleop para criar um mapa completo do ambiente.

