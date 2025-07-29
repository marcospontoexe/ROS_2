# URDF
URDF (Unified Robot Description Format) é um formato de arquivo em XML usado no ROS para descrever a estrutura física de um robô — como seus links (partes rígidas), juntas (articulações), sensores e outras propriedades.

## O que o URDF descreve?
1. Links (Elos) – Os corpos rígidos que compõem o robô
    * As partes físicas do robô (base, rodas, braços, sensores etc.)
    * Cada link tem: Massa, Inércia, Geometria (caixa, cilindro, malha STL, etc.), Cor e material

2. Joints (Articulações) – As conexões entre os elos que permitem o movimento
    * As conexões entre os links (ex: uma roda presa ao chassi)
    * Tipos de juntas: fixed (fixa), revolute (rotacional com limite), continuous (rotacional sem limite), prismatic (linear), floating, planar (menos comuns)

3. Transmissions (opcional)
    * Usadas em conjunto com ros_control, para mapear motores para juntas.

4. Plugins e sensores (opcional)
    * Câmeras, LiDARs, IMUs etc., geralmente com uso de extensões como o Gazebo.


## Por que preciso de URDFs?
Os URDFs são usados para duas aplicações principais:

1. Ter uma definição do modelo para o sistema ROS que precisa deles, como publicação, navegação e manipulação de TF.
2. Você pode usá-los para gerar uma versão simulada do seu robô em um Simulador Gazebo.

## Por que preciso de um modelo de robô?
Existem muitos motivos pelos quais alguém precisaria simular um robô:

* Você precisa de uma maneira de descrever seu robô físico no ROS: Os modelos URDF não são necessários apenas para simulações. Eles também fornecem uma descrição virtual do robô real que publica dados no ROS. Por exemplo, programas ROS como o RVIZ podem representar o robô com base em valores de juntas reais.
* É impossível trabalhar com um robô real: Existem muitos robôs que não estão disponíveis ao público em geral, como ASIMO, VALKIRIA, ROBONAUT e ATLAS, para citar alguns. Eles são robôs bem projetados, mas poucas pessoas podem usá-los e, se puderem usá-los, estarão sujeitos a condições rigorosas. Então, por que não criar sua própria versão virtual dele?
* Quero criar um robô físico que não existe: Construir um robô é caro, então você deve confirmar com antecedência se não há falhas significativas de projeto. Ao criar o robô em uma simulação, você pode iterar muito mais rápido no projeto e detectar quaisquer erros antes de iniciar a impressão 3D. Você também pode testar e alterar o projeto sem nenhum custo real. Por exemplo, o que aconteceria se você adicionasse uma ou duas pernas, câmeras, usasse servidores mais potentes ou o tornasse mais pesado? A lista é infinita.
* É muito arriscado fazer meus testes preliminares no robô físico: você tem o robô, mas pode querer testar algo muito perigoso para a integridade do robô. Talvez você queira testar um algoritmo para interação humano-robô pela primeira vez e não queira que ninguém se machuque durante os testes.

# Construindo um modelo de robô com URDF
Nesta unidade, você aprenderá a criar uma representação visual de um robô em URDF (Unified Robot Description Format). É importante entender que, embora este modelo visual não seja uma simulação física, ele serve como estrutura fundamental para fins de **simulação**. Embora o modelo URDF em si não simule comportamento físico, ele já é altamente útil.

Se você tem um robô real e deseja aproveitar a infraestrutura do ROS, precisa de uma descrição virtual de como o robô é montado e onde cada sensor está localizado. Por exemplo, se uma câmera estiver montada na cabeça do robô, o arquivo URDF permite usar a estrutura TF no ROS para identificar a localização exata da câmera, com base nas leituras das juntas e dos sensores. Além disso, essa descrição permite visualizar o modelo do robô dentro do RViz.

## O que é um arquivo URDF
URDF (Unified Robot Description Format) é uma forma estruturada de definir as propriedades **físicas e cinemáticas** de um robô no ROS 2. Simplificando, é um formato baseado em XML usado para descrever a estrutura e os componentes de um robô.

Um arquivo URDF pode definir vários aspectos de um robô, incluindo:
* Links (Elos) – Os corpos rígidos que compõem o robô
* Joints (Articulações) – As conexões entre os elos que permitem o movimento
* Shapes (Formas) e meshes (malhas) – A geometria visual e de colisão do robô
* Sensores – Componentes que fornecem dados sobre o ambiente
* Atuadores – Motores e outros mecanismos que permitem o movimento
* Propriedades físicas – Parâmetros como massa, inércia e atrito para simulação

Os arquivos urdf devem ser criados dentro do diretório **urdf**.

### O que é um LINK (Elo)
LINKS são as unidades individuais que, por meio da montagem, formam um robô.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/exemplos/marcos_bot_description/urdf/marcos_bot_simple.urdf) um arquivo urdf chamado **marcos_bot_simple.urdf**:

* Este URDF será apenas um único LINK.
* O nome do link é **base_link**.
* Dentro dele, defina uma geometria dentro do elemento visual.
* Uma geometria é o formato que você deseja dar a este link, neste caso, uma caixa simples.

Todas as medidas em URDF estão no **Sistema Internacional de Unidades**:

* metros para distância
* radianos para ângulos
* quilogramas para peso

Existem três geometrias básicas suportadas para LINKS:

* caixa
* cilindro
* esfera

Eles devem ser definidos dentro do elemento de **geometry** da seguinte maneira:

```xml
<cylinder radius="0.06" length="0.09"/>
<box size="0.0005 0.0005 0.0005"/> 
<sphere radius="0.06"/>
```

## Visualize arquivos URDF no RVIZ2
[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/exemplos/marcos_bot_description/launch/urdf_visualize.launch.py) um launch para executar um arquivo urdf.

Aqui, revise alguns elementos do arquivo launch:

```python
####### DATA INPUT ##########
urdf_file = 'marcos_bot_simple.urdf'
#xacro_file = "urdfbot.xacro"
package_description = "marcos_bot_description"

####### DATA INPUT END ##########
print("Fetching URDF ==>")
robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
```

Aqui, você procura o caminho absoluto para o arquivo URDF que deseja usar. Neste caso, você precisa do arquivo URDF **marcos_bot_simple.urdf** dentro do pacote **marcos_bot_description**.

```python
# Robot State Publisher
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher_node',
    emulate_tty=True,
    parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
    output="screen"
)
```

Aqui, instancie um nó que você chamará de robot_state_publisher_node, que inicia o robot_state_publisher.

No ROS2, ele tem a função de: 

* Publicar no tópico **/robot_description** o conteúdo do arquivo URDF que você forneceu através do parâmetro **robot_description**.

```python
Command(['xacro ', robot_desc_path])
```

Observe o método **Command**. Ele é usado para executar qualquer comando de shell. Neste caso, você chama o binário XACRO com o argumento do caminho absoluto para o arquivo URDF. O binário XACRO é usado para processar arquivos URDF e XACRO.

* Execute diretamente o comando XACRO: `xacro /home/user/ros2_ws/src/marcos_bot_description/urdf/marcos_bot_simple.urdf`
* Isso mostrará o que será publicado dentro do tópico robot_description:

```shell
user:~/ros2_ws$ xacro /home/user/ros2_ws/src/marcos_bot_description/urdf/marcos_bot_simple.urdf
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from /home/user/ros2_ws/src/marcos_bot_description/urdf/marcos_bot_simple.urdf | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="urdfbot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

Execute o arquivo **urdf_visualize.launch.py**.
Ele publicará o conteúdo do arquivo marcos_bot_simple.urdf no tópico **/robot_description**.
O RVIZ2 lerá o tópico robot_description e mostrará os links definidos.

Abra o RVIZ2, à qual você adicionará as representações do TF e um Modelo do Robô:

Configuração:

1. Fixed Frame: Na parte superior do painel esquerdo, clique no triângulo preto à esquerda de Opções Globais para visualizar/ocultar os detalhes das Opções Globais. Um dos elementos que serão exibidos é o campo Quadro Fixo com "mapa" como valor padrão. Clique no campo para ver um cursor piscando no texto. Defina como valor **base_link** e pressione Enter. NÃO ESTARÁ disponível como um menu suspenso; você precisa DIGITAR o valor de base_link.
2. Modelo do Robô: Neste caso, clique no triângulo preto à esquerda de Modelo do Robô para abrir os detalhes. Role para baixo e confirme se a Origem da Descrição está definida como **Topic**. Em seguida, clique no espaço em branco à esquerda de **Description Topic** e DIGITE **/robot_description**. Em seguida, clique no triângulo preto à esquerda de Description Topic para visualizar mais campos. Defina a **Reliability Policy** como **Reliable** e a **Durability Policy** como **Transient Local**.

Quando tiver concluído essas etapas, você deverá ter algo semelhante a isto:

![rviz2basic1](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/rviz2basic1.png)

A caixa vermelha na janela de exibição principal representa o modelo de robô que você construiu.