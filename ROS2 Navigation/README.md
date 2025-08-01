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

1[Nav2 Lifecycle Manager](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Navigation/imagens/architectural_diagram.png)

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
    *   **configuration_directory**: o diretório onde encontrar os arquivos de configuração
    *   **configuration_basename**: o nome do arquivo de configuração

```python
    package='cartographer_ros', 
    executable='cartographer_node', 
    name='cartographer_node',
    output='screen',
    parameters=[{'use_sim_time': True}],
    arguments=['-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename]
```