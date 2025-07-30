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

## O que é um LINK (Elo)
LINKS são as unidades individuais que, por meio da montagem, formam um robô.

Veja nesse exemplo a baixo um arquivo urdf que:

* Este URDF será apenas um único LINK.
* O nome do link é **base_link**.
* Dentro dele, defina uma geometria dentro do elemento visual.
* Uma geometria é o formato que você deseja dar a este link, neste caso, uma caixa simples.

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

</robot>
```

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

### Visualize arquivos URDF no RVIZ2
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

## O que é uma junta (joint)
A **CONEXÃO** entre DOIS LINKS é chamada de JUNTA.

Veja no exemplo a baixo um arquivo urdf que conecta dois links, **base_link** e **head_link**, com uma **JOINT**.

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_link_to_head_link_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0 0 0.11" />
        <parent link="base_link" />
        <child link="head_link" />
    </joint>

</robot>
```

Ao executar a [launch](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/exemplos/marcos_bot_description/launch/urdf_visualize.launch.py) e abrir o rviz novamente com as configurações feitas anteriormente, verá o seguinte:

![joint](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/rvizjoinst2v2.png)

* Você tem dois **links**:
    1. base_link: Este é o link raiz, o Parent de onde todos os links serão suspensos. Ele precisa ser ÚNICO.
    2. head_link: Este link representa a cabeça do seu robo, uma caixa de 0,1 metro de cada lado.
* Você tem uma **junta**:
    1. **base_link_to_head_link_joint**: Esta é uma junta fixa, o que significa que NÃO há movimento entre os dois elos. É como se você tivesse soldado os dois elos.
* No entanto, as articulações podem se mover.
* JOINTS são as articulações do robô.

```xml
<joint name="base_link_to_head_link_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.11" />
    <parent link="base_link" />
    <child link="head_link" />
</joint>
```

Os elementos básicos para definir uma junção são:

* parent e child: Aqui, você define quem está conectado ao seu link. Neste caso, base_link foi o Pai para head_link.
* Origem: Todas as coordenadas (X, Y e Z e R, P, Y) são referenciados ao eixo Pai, não ao eixo Filho: Neste caso, o quadro de origem do head_link está 0,11 metros acima no EIXO Z da origem do quadro do base_link.

### TF Frames e LINKS
Automaticamente, cada LINK que você define em um URDF tem um TF FRAME associado.

1. Adicione TF no elemento RVIZ2.
2. Defina também o **alfa** como 0,5 no elemento RobotModel para tornar os links transparentes.

O nó **Robot State Publisher** publica todos os quadros TF. Ele também publica os **quadros TF** de todos os links no tópico **/tf**, de onde o RVIZ2 está lendo.

Dê uma olhada no tópico /tf: `ros2 topic info /tf --verbose`

```shell
Type: tf2_msgs/msg/TFMessage                                                  
Publisher count: 1                                                           
Node name: robot_state_publisher_node                                          
Topic type: tf2_msgs/msg/TFMessage                                            
Endpoint type: PUBLISHER                                                      
GID: e6.22.10.01.18.5b.3c.f9.71.dd.e4.02.00.00.16.03.00.00.00.00.00.00.00.00 
    QoS profile:                                                                 
    Reliability: RELIABLE                                                      
    Durability: VOLATILE                                                       
    Lifespan: 9223372036854775807 nanoseconds                                  
    Deadline: 9223372036854775807 nanoseconds                                  
    Liveliness: AUTOMATIC                                                       
    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                               
```

* O CONCEITO MAIS IMPORTANTE AQUI é que **cada link possui um QUADRO associado**.
* Este quadro é independente do quadro de referência da forma geométrica visual.
* Para ilustrar esse conceito, você pode adicionar o deslocamento dos elementos visuais se desejar que a forma geométrica fique em uma posição diferente do quadro do link associado.
* Faça essa modificação para que a caixa visual head_link seja deslocada 0,1 metro no eixo Z do QUADRO head_link, usando o parametro **origin**.

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0.1" /> <!-- ORIGIN DISPLACEMENT ADDED -->
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_link_to_head_link_joint" type="fixed">
        <origin rpy="0 0 0" xyz="0 0 0.11" />
        <parent link="base_link" />
        <child link="head_link" />
    </joint>

</robot>
```

Você deverá ver algo assim:

![visualdisplacement](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/visualdisplacement.png)

* Observe que a caixa visual de geometria associada ao head_link não se origina no mesmo local que o **quadro head_link**.
* NOVAMENTE, isso serve para mostrar que uma coisa são os quadros LINK e outra são as formas geométricas associadas a esses links. Eles NÃO são a mesma coisa.

* Observe que no RVIZ2 agora você pode selecionar os diferentes links em um menu suspenso (base_link ou head_link).
* Por que não conseguimos fazer isso quando tínhamos apenas UM LINK?
* O motivo é que, com apenas UM LINK, os TFs NÃO são publicados.
* Precisamos de pelo menos dois links com uma junção para iniciar a publicação do TF.

## Tipos de Juntas (joints)
* Lembra da definição da junção urdf para o seu base_link_to_head_link_joint?
* Viu o parâmetro **type**? Neste caso, corrigido.

```xml
<joint name="base_link_to_head_link_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.11" />
    <parent link="base_link" />
    <child link="head_link" />
</joint>
```

Não existem apenas tipos de juntas fixas.

Estes são os tipos de juntas disponíveis:
* continuous
* revolute
* prismatic
* fixed 

Existem também dois outros tipos:
* flutuante (floating): OBSERVE que esta articulação não funciona corretamente no ROS2.
* planar: OBSERVE que esta articulação não funciona corretamente no ROS2.

Aqui está uma postagem sobre como reproduzir essas juntas planas e flutuantes com combinações de outros tipos de juntas: [Planar and Floating joints patch](https://robotics.stackexchange.com/questions/28609/how-to-achieve-planar-joint).

Veja alguns exemplos de como você teria que alterar seu **base_link_to_head_link_joint** para todos os outros tipos de juntas:

**continuous**
```xml
<joint name="base_link_to_head_link_joint" type="continuous">
  <origin xyz="0 0 0.11" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="head_link"/>
  <axis xyz="0 0 1"/>
</joint>
```
---

**revolute**
```xml
<joint name="base_link_to_head_link_joint" type="revolute">
  <origin xyz="0 0 0.11" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="head_link"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
</joint>
```
---

**prismatic**
```xml
<joint name="base_link_to_head_link_joint" type="prismatic">
  <origin xyz="0 0 0.11" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="head_link"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="0.1" lower="-0.1" upper="0.1"/>
</joint>
```

### Mover as juntas com Publisher Joint State 
DOIS nós se confundem bastante:

* Robot State Publisher
* Joint State Publisher

O Robot State Publisher publica os tópicos **/robot_description** e **/tf**.

A principal função do Robot State Publisher é publicar os **QUADROS** DE **LINK**.

Com links conectados por JUNTAS FIXAS, não há problema. Você já viu. MAS, e se as JUNTAS SE MOVIMENTAREM?

Como o Robot State Publisher pode saber a orientação de um elo se a junta à qual ele está conectado se moveu?

Alguém deve publicar esses valores de junta (ângulos em juntas contínuas ou de revolução, metros em juntas prismáticas ou planas). É aqui que entra o **Joint State Publisher**:
* Você pode publicar esses joint states por meio de dois métodos:
    1. Os codificadores (**encoders**) de hardware do robô (simulados ou não)
    2. Manualmente usando o comando **joint_state_publisher_gui**: `ros2 run joint_state_publisher_gui joint_state_publisher_gui`.

* Este nó (joint_state_publisher_gui) permite que você defina o estado da junta **manualmente**, permitindo que o Robot State Publisher saiba os valores da junta e, portanto, consiga publicar os quadros TF.

Mude sua junta para uma junta contínua:

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0.1" />
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_link_to_head_link_joint" type="continuous">
        <origin xyz="0 0 0.11" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
    </joint>

</robot>
```

Execute o joint_state_publisher_gui: `ros2 run joint_state_publisher_gui joint_state_publisher_gui`.

Ao compilar e executar a launch novamente, aparece um erro no RobotModel. Além disso, o head_link aparece em BRANCO e a TF desaparece.

Isso é completamente compreensível, pois o valor da junção que conecta base_link com head_link agora é desconhecido. O ERobot State Publisher ainda não sabe e, portanto, não pode publicar o quadro TF e posicionar a caixa visual do head_link.

Publique esse valor usando a interface gráfica joint_state_publisher_gui.

Agora, o head_link não está mais branco e todos os quadros do TF são renderizados.
Se você mover o controle deslizante, a caixa do head_link deverá girar.

O head_link está girando em torno do eixo Z, AZUL. Mas, qual eixo Z? O eixo inicial do quadro base_link (Pai) ou do quadro head_link (Filho).

Neste caso, onde a junta tem um eixo de rotação, ele sempre se refere ao E**IXO DO QUADRO FILHO**. Neste caso, o eixo Z do quadro head_link.
Este não é o mesmo caso no elemento de origem da junta, onde tudo se refere ao QUADRO PARENTES. O motivo é que você definiu esse eixo como **ativo** no marcos_bot_simple.urdf.

```xml
<axis xyz="0 0 1"/>
```

Altere-o para girar em torno do eixo X, (VERMELHO). Ele gira em torno do eixo X do QUADRO head_link, não do eixo X do QUADRO base_link:


```xml
<axis xyz="1 0 0"/>
```

## Materiais URDF
Você pode atribuir **materiais** aos elementos visuais dos seus **links**. Materiais são essencialmente cores.

Altere os materiais dos seus elementos visuais dos links, coloque o material depois do elemento **geometry**, dentro dos elementos **visual**:

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
    <material name="blue">
        <color rgba="0.006 0.151 0.581 1"/>
    </material>

    <material name="white">
        <color rgba="1.0 0.91 0.827 1"/>
    </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
       <material name="white"/>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0.1" />
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

    <joint name="base_link_to_head_link_joint" type="revolute">
        <origin xyz="0 0 0.11" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>    

</robot>
```

Os nomes dos materiais são inventados. Você pode dar o nome que quiser. É um elemento. Neste caso, você criou dois, branco e azul.
URDF Meshes

## Malhas (meshes) URDF
Além de formas geométricas básicas como CAIXA (Cubo), CILINDRO e ESFERA, você também pode usar malhas 3D. Os formatos suportados são **.DAE, .STL e OBJ**.

Aqui, os materiais não afetarão DAE ou OBJ, pois já possuem seus próprios materiais e texturas atribuídos.

Nesse exemplos a malha (URDF) serão baixadas deste Git:

```shell
cd ~/ros2_ws/src/
git clone https://bitbucket.org/theconstructcore/urdf_meshes.git
cp -r ~/ros2_ws/src/urdf_meshes/meshes ~/ros2_ws/src/urdfbot_description/
```

[Veja aqui](https://github.com/marcospontoexe/ROS_2/tree/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/exemplos/marcos_bot_description/meshes) os arquivos meshes.

Lembre de incluir o diretório no arquivo CMakeLists.txt:

```txt
install(
  DIRECTORY
    urdf
    rviz
    launch
    meshes
  DESTINATION
    share/${PROJECT_NAME}/
)
```

Adicione as modificações no arquivo urdf para adicionar malhas em vez de formas geométricas no elemento **visual**:

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
    <material name="blue">
        <color rgba="0.006 0.151 0.581 1"/>
    </material>

    <material name="white">
        <color rgba="1.0 0.91 0.827 1"/>
    </material>

  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_body.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

    <joint name="base_link_to_head_link_joint" type="revolute">
        <origin xyz="0 0 0.07" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>    

</robot>
```

Vamos ver oque foi alterado no arquivo urdf:

```xml
<geometry>
    <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
</geometry>
```

* Indica onde o pacote onde as malhas estão. Neste caso, urdfbot_description.
* Indica a escala. Isso é usado para ajustar as malhas ao tamanho desejado. Neste caso, você precisava dimensionar todos os modelos em 10; portanto, 1/10 = 0,1.

```xml
<link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
```

Observe que a origem agora é '0,0' no deslocamento do eixo Z. O motivo é que a origem da sua malha não está no centro da geometria como a caixa anterior.

Aqui está uma comparação de onde cada uma das origens está:

![headaxis](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/headaxis.png)

![boxaxis](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/boxaxis.png)

É por isso que você não precisa do deslocamento para a cabeça, porque ela já tem um para posicionar melhor a malha visual.

## Elementos especiais de juntas

### <limit> (necessário apenas para juntas revolutas e prismáticas)

Esse elemento pode conter os seguintes atributos:

* lower (opcional, padrão 0): Um atributo que especifica o limite inferior da junta (em radianos para juntas de revolução, em metros para juntas prismáticas). Omita se a junta for contínua.
* upper (opcional, padrão 0): Um atributo que especifica o limite superior da junta (em radianos para juntas de revolução, em metros para juntas prismáticas). Omita se a junta for contínua.
* effort (**obrigatório**): Um atributo para impor o esforço máximo da junta (|esforço aplicado| < |esforço|). Consulte os limites de segurança.
* velocity (**obrigatório**): Um atributo para impor a velocidade máxima da junta (em radianos por segundo [rad/s] para juntas de revolução, em metros por segundo [m/s] para juntas prismáticas). Consulte os limites de segurança.

Observe que em URDFs sem simulação, os elementos de esforço e velocidade são irrelevantes. Esses elementos só são relevantes quando você usa a URDF para simulação.

### <mimic> (opcional)
Este elemento é usado para especificar que a junta definida imita outra junta existente. O valor desta junta pode ser calculado como:

valor = multiplicador * valor_da_outra_junta + offset

Atributos esperados e opcionais:

* valor_da_outra_junta (obrigatório): Especifica o nome da junta a ser imitada.
* multiplicador (opcional): Especifica o fator multiplicativo na fórmula acima.
* offset (opcional): Especifica o deslocamento a ser adicionado na fórmula acima. O padrão é 0 (radianos para juntas de revolução, metros para juntas prismáticas).

Veja um exemplo de como usar este elemento de imitação:
Você vai adicionar braços e pernas ao seu robô.

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_body.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

    <joint name="base_link_to_head_link_joint" type="revolute">
        <origin xyz="0 0 0.07" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>    

    <!-- Right Arm -->

    <link name="upper_arm_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_limb.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="base_link_to_upper_arm_r_link_joint" type="revolute">
        <origin xyz="0 -0.03272 0.0279895" rpy="-1.57 0 0"/>
        <parent link="base_link"/>
        <child link="upper_arm_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.0" upper="1.0"/>
    </joint>

</robot>
```

OBSERVE como você definiu como eixo de ROTAÇÃO a nova JUNTA base_link_to_upper_arm_r_link_joint.

* O eixo de rotação é o eixo Z do upper_arm_r_link, NÃO o eixo Z do quadro base_link.
* OBSERVE também que você rotacionou o upper_arm_r_link em relação ao base_link a partir do eixo de rotação (eixo X).

RESUMO:
* O eixo de rotação da articulação é feito em relação ao referencial FILHO.
* A orientação e translação da JUNTA são feitas em relação ao EIXO do referencial PAI.

Agora adicione o resto do braço:

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_body.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

    <joint name="base_link_to_head_link_joint" type="revolute">
        <origin xyz="0 0 0.07" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>    

    <!-- Right Arm -->

     <link name="upper_arm_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_limb.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="base_link_to_upper_arm_r_link_joint" type="revolute">
        <origin xyz="0 -0.03272 0.0279895" rpy="-1.57 0 0"/>
        <parent link="base_link"/>
        <child link="upper_arm_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.0" upper="1.0"/>
    </joint>

    <link name="lower_arm_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_limb.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_lower_arm_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.0568" rpy="0 0 0"/>
        <parent link="upper_arm_r_link"/>
        <child link="lower_arm_r_link"/>
        <axis xyz="1 0 0"/>
        <limit effort="100" velocity="1.0" lower="-1.0" upper="1.0"/>
    </joint>


    <link name="claw_a_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_claw.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_claw_a_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.065" rpy="-1.57 0 -1.57"/>
        <parent link="lower_arm_r_link"/>
        <child link="claw_a_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-0.7" upper="0.7"/>
    </joint>

    <link name="claw_b_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_claw.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_claw_b_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.065" rpy="-1.57 0 1.57"/>
        <parent link="lower_arm_r_link"/>
        <child link="claw_b_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-0.7" upper="0.7"/>
    </joint>

</robot>
```

Antes de adicionar o elemento mímico, diminua o tamanho dos quadros RVIZ TF para visualizar melhor o robô e suas articulações.
Reduza a ESCALA DO MARCADOR dos quadros TF para 0,3.
Você deve ter algo assim agora:

![smallertfs](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/smallertfs.png)

Você quer que ambas as garras se movam a mesma quantidade de radianos com um único controle deslizante, como em um robô com um único atuador para fechar e abrir as garras. É por isso que você usa o elemento **mimic**.

Você deve posicionar o elemento mimic na junta que deseja simular. No seu caso, você quer que a junta **upper_arm_r_link_to_claw_B_r_link_joint** imite a **upper_arm_r_link_to_claw_A_r_link_joint**. Portanto, você posicionará o elemento mimic dentro da upper_arm_r_link_to_claw_B_r_link_joint da seguinte forma:

```xml
<?xml version="1.0"?>
<robot name="marcos_bot">
        
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_body.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="head_link">
    <visual>
    <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://marcos_bot_description/meshes/urdfbot_head.dae" scale="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

    <joint name="base_link_to_head_link_joint" type="revolute">
        <origin xyz="0 0 0.07" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="head_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>    

    <!-- Right Arm -->

     <link name="upper_arm_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_limb.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="base_link_to_upper_arm_r_link_joint" type="revolute">
        <origin xyz="0 -0.03272 0.0279895" rpy="-1.57 0 0"/>
        <parent link="base_link"/>
        <child link="upper_arm_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.0" upper="1.0"/>
    </joint>

    <link name="lower_arm_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_limb.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_lower_arm_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.0568" rpy="0 0 0"/>
        <parent link="upper_arm_r_link"/>
        <child link="lower_arm_r_link"/>
        <axis xyz="1 0 0"/>
        <limit effort="100" velocity="1.0" lower="-1.0" upper="1.0"/>
    </joint>


    <link name="claw_a_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_claw.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_claw_a_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.065" rpy="-1.57 0 -1.57"/>
        <parent link="lower_arm_r_link"/>
        <child link="claw_a_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-0.7" upper="0.7"/>
    </joint>

    <link name="claw_b_r_link">
        <visual>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <geometry>
            <mesh filename="package://marcos_bot_description/meshes/urdfbot_claw.dae" scale="0.1 0.1 0.1"/>
        </geometry>
        </visual>
    </link>

    <joint name="upper_arm_r_link_to_claw_b_r_link_joint" type="revolute">
        <origin xyz="0 0 -0.065" rpy="-1.57 0 1.57"/>
        <parent link="lower_arm_r_link"/>
        <child link="claw_b_r_link"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="0.0" upper="0.0"/>
        <mimic joint="upper_arm_r_link_to_claw_a_r_link_joint" multiplier="1.0" offset="0.0"/>
    </joint>
</robot>
```

Defina a articulação mimética como "upper_arm_r_link_to_claw_a_r_link_joint". Defina o multiplicador como 1,0. Isso significa que para cada radiante ALFA, girado por upper_arm_r_link_to_claw_A_r_link_joint, sua articulação mimética upper_arm_r_link_to_claw_B_r_link_joint se moverá ALFA * 1,0, Ela se moverá na mesma proporção.

Se você alterar o multiplicador, fará com que a junta mimetizadora se mova assimetricamente em relação à junta mimetizada original. Altere-o para 0,1, isso significa que ela se moverá DEZ VEZES MENOS radianos em relação à junta mimetizada original.

A junta que possui o elemento mimic usará os mesmos limites da junta que está imitando. Então, por que deixá-lo se não é necessário?

O elemento **limit** deve ser adicionado dentro da junta que possui o elemento mimic, mesmo que não seja levado em consideração, pois, caso contrário, o analisador URDF gerará um erro parecido com esse:

```shell
Error:   Joint [upper_arm_r_link_to_claw_b_r_link_joint] is of type REVOLUTE but it does not specify limits
         at line 572 in /tmp/binarydeb/ros-galactic-urdfdom-2.3.5/urdf_parser/src/joint.cpp
Error:   joint xml is not initialized correctly
         at line 206 in /tmp/binarydeb/ros-galactic-urdfdom-2.3.5/urdf_parser/src/model.cpp
ERROR: Model Parsing the xml failed
```

Verifique a integridade de um arquivo URDF usando o comando check_urdf, seguido pelo caminho do arquivo URDF: `check_urdf src/marcos_bot_description/urdf/marcos_bot_simple.urdf`

## Elementos de simulação

### <dynamics> (optional)
Especifica as propriedades físicas da junta. Esses valores são usados para especificar as propriedades de modelagem da junta, usadas para simulação.

* damping (opcional, padrão 0): O valor de **amortecimento** físico da junta (em newtons-segundos por metro [N∙s/m] para juntas prismáticas, em newtons-metros-segundos por radiano [N∙m∙s/rad] para juntas revolutas).
* friction (opcional, padrão 0): O valor de **atrito estático** físico da junta (em newtons [N] para juntas prismáticas, em newtons-metros [N∙m] para juntas revolutas).

```xml
<dynamics damping="0.1" friction="0.05"/>
```

### <safety_controller> (optional)
Pode conter os seguintes atributos:

* soft_lower_limit (opcional, padrão 0): Um atributo que especifica o limite inferior da junta onde o controlador de segurança começa a limitar a posição da junta. Este limite deve ser maior que o limite inferior da junta (veja acima). Consulte os limites de segurança para obter mais detalhes.
* soft_upper_limit (opcional, padrão 0): Um atributo que especifica o limite superior da junta onde o controlador de segurança começa a limitar a posição da junta. Este limite deve ser menor que o limite superior da junta (veja acima). Consulte os limites de segurança para obter mais detalhes.
* k_position (opcional, padrão 0): Um atributo que especifica a relação entre os limites de posição e velocidade. Consulte os limites de segurança para obter mais detalhes.
* k_velocity (obrigatório): Um atributo que especifica a relação entre os limites de esforço e velocidade. Consulte os limites de segurança para obter mais detalhes.

### <calibration> (optional)
As posições de referência da junta são usadas para calibrar a posição absoluta da junta.

* rising (opcional): Quando a junta se move em uma direção positiva, esta posição de referência acionará uma borda ascendente.
* falling (opcional): Quando a junta se move em uma direção positiva, esta posição de referência acionará uma borda descendente.

Esses sistemas são usados para calibrar juntas com codificadores relativos e precisam saber onde estão os limites da amplitude de movimento para posicionar com precisão, antes de iniciar a operação.

```xml
<joint name="base_link_to_head_link_joint" type="revolute">
    <origin xyz="0 0 0.07" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="head_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    <!-- calibration definition -->
    <calibration rising="1.5" falling="-1.5" reference_position="0.0"/>
</joint>
```

* rising="1.5" significa que um interruptor de limite é ativado quando a junta se move para uma posição de 1,5 radianos (ou seja, passa de desligado para ligado).
* falling="-1.5" indica que quando a junta se move para -1,5 radianos, um interruptor de limite previamente ativado é desativado (ou seja, passa de ligado para desligado).

Aqui está uma representação conceitual:

```html
|          switch
|        deactivated
|      ___________________             <- Falling edge (-1.5 radians)
|     /
|    /
|   /
|  /
| /_______________________             <- Joint movement direction
|                          \
|                           \
|                            \
|                             \        <- Rising edge (1.5 radians)
|                              -----------------------
|                                        switch
|                                      activated
```

# Inicie o RVIZ2 por meio de um arquivo de inicialização
Agora você pode modificar o **urdf_visualize.launch.py**, criando uma versão que inicia o RVIZ2, que você chamará de **urdf_visualize_rviz.launch.py.**
Dessa forma, você não precisará iniciar o RVIZ2 manualmente.

**urdf_visualize_rviz.launch.py:**

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'marcos_bot_simple.urdf'
    #xacro_file = "urdfbot.xacro"
    package_description = "marcos_bot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node
        ]
    )
```

# Criando um robô de duas rodas
Para este Micro Projeto, você terá que construir o seguinte robô:

![robot](https://github.com/marcospontoexe/ROS_2/blob/main/URDF%20for%20Robot%20Modeling%20in%20ROS2/imagens/urdf_ros2_nottransparent.png)

Como você pode ver, este robô possui as seguintes partes:

* Corpo: É a caixa que, neste caso, você vê com uma carinha sorridente em forma de malha.
* Rodas direita e esquerda: São aquelas com motores.
* rodas giratórias dianteiras/traseiras: Essas rodas são livres e evitam que o corpo colida com o chão durante o movimento.

Revise o funcionamento das diferentes peças, especialmente as rodas giratórias:

* DUAS juntas contínuas, uma para cada roda.
* Você precisa que elas girem em uma direção mais de 360 graus.
* Essas juntas devem girar em torno do eixo perpendicular à roda.
* Defina as rodas giratórias.
* Essas esferas devem girar sem um limite de 360 graus, portanto, precisam ser contínuas.
* Além disso, você precisa que as rodas giratórias girem em todos os três eixos, não apenas em um.
* Em arquivos URDF, não há suporte para juntas ESFÉRICAS (em formatos SDF tem).
* Isso significa que você deve definir três juntas diferentes que girem em cada eixo.
* No final, a roda giratória se comportará como uma junta esférica.

Crie os seguintes novos arquivos:

* Crie um pacote chamado **marcos_box_bot_description**.
* Crie o arquivo URDF, **box_bot_geometric.urdf**, com os novos links (corpo e rodas) e elementos descritos.
* Crie o arquivo URDF, **box_bot_geometric_meshe.urdf**, que terá uma versão em malha.
* Crie o arquivo de inicialização **urdf_visualize_geometric.launch.py**, um novo lançamento que publica na **robot_description** o conteúdo de **box_bot_geometric.urdf**.
* Crie o arquivo de inicialização **urdf_visualize_meshes.launch.py**, um novo lançamento publicado na **robot_description** o conteúdo de **box_bot_geometric_meshes.urdf**.
* Ambas as inicializações devem iniciar o RVIZ2 com seu próprio arquivo RVIZ, urdf_vis.rviz.

NOVO CONCEITO:
* Links podem estar vazios, sem nenhum elemento visual dentro. Isso publicará apenas o frame do LINK.
* Isso é usado para links auxiliares a outros links, facilitando a construção do robô.

Exemplo para o FRAME base_link:

```xml
<link name="base_link">
</link>
```

## Links

* base_link, sem nenhum elemento visual.
* chassis, com uma caixa **box_bot_blue** com geometria básica de tamanho size="0.1 0.1 0.1".
* left_wheel, com um cilindro preto com forma geométrica básica de dimensões length="0.001" radius="0.035".
* right_wheel, com um cilindro branco com forma geométrica básica de dimensões length="0.001" radius="0.035".
* front_yaw_link, back_yaw_link, com um cilindro azul com forma geométrica básica de dimensões length="0.005" radius="0.010".
* front_roll_link, back_roll_link, com um cilindro vermelho com forma geométrica básica de dimensões length="0.005" radius="0.010".
* front_pitch_link, com uma esfera green_dark com forma geométrica básica de dimensões radius="0.010".
* back_pitch_link, com uma esfera green_light com forma geométrica básica de dimensões radius="0.010".

## JOINTS

* Todos os tipos de juntas que precisam se mover devem ser do tipo continuous.
* Front_pitch_joint: Conecta o front_roll_link com front_pitch_link
  * Origem rpy="0 0 0" xyz="0 0 0"
* Back_pitch_joint: Conecta o back_roll_link com back_pitch_link
  * Origem rpy="0 0 0" xyz="0 0 0"
* Front_roll_joint: Conecta o front_yaw_link com front_roll_link
  * Origem rpy="0 0 0" xyz="0 0 0"
* Back_roll_joint: Conecta o back_yaw_link com back_roll_link
  * Origem rpy="0 0 0" xyz="0 0 0"
* Front_yaw_joint: Conecta o chassis com o front_yaw_link
  * Origem rpy="0 0 0" xyz="0.04 0 -0,05"
* back_yaw_joint: Conecta o chassis com o back_yaw_link
  * Origem rpy="0 0 0" xyz="-0,04 0 -0,05"
* joint_right_wheel: Conecta o chassis com o right_wheel
  * Origem rpy="0 0 0" xyz="0 -0,05 -0,025"
* joint_left_wheel: Conecta o chassis com o left_wheel
  * Origem rpy="0 0 0" xyz="0 0,05 -0,025"
* base_link_joint: Conecta o base_link com o chassis
  * Origem rpy="0 0 0" xyz="0 0 0"