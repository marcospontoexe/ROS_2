# Launch
No ROS 2, os arquivos de inicialização podem ser criados usando diferentes formatos, incluindo **Python, XML e YAML**. Embora o Python seja comumente usado por sua flexibilidade, XML e YAML oferecem alternativas mais estruturadas e declarativas, tornando os arquivos de inicialização mais fáceis de ler e compartilhar.

Observe que TODOS eles serão iniciados da MESMA MANEIRA.

Por exemplo, a maneira de iniciar um programa ROS para qualquer tipo de arquivo de inicialização é:
* `ros2 launch <package_name> <launch_file_name>`
* `ros2 launch <path_to_launch_file>`

Se você quiser passar um argumento para um arquivo de inicialização (todos os formatos):
* `ros2 launch <package_name> <launch_file_name> argument1:=WHATEVER_VALUE`
* `ros2 launch <path_to_launch_file> argument2:=WHATEVER_VALUE`

**Nota**: lembre-se de que para iniciar nós com ros2, a passagem de parâmetros é um pouco diferente:
* `ros2 run <package_name> <executable_name> --ros-args argument1:=WHATEVER_VALUE`

Se você quiser obter os argumentos que podem ser fornecidos a um arquivo de inicialização (todos os formatos):
* `ros2 launch <package_name> <launch_file_name> --show-args`
* `ros2 run <package_name> <executable_name> --ros-args argument1:=WHATEVER_VALUE`

## arquivos de launch usando python
[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/launch/python_main.launch.py) a launch **python_main.launch.py** usando python.

Ao executar o comando `ros2 launch launch_tests_pkg python_main.launch.py turning_speed:=6.0 forward_speed:=0.2`. Você deve fazer com que o robô comece a se mover para frente e girar em pequenos intervalos, com a velocidade de giro e avanço definida por você nesta execução de linha de comando.

Alguns pontos a serem comentados:
* TextSubstitution(text="0.2"): O que é este TextSubstitution? Este método tem duas funções:
    * A primeira é que todas essas **launch.substitutions** são projetadas para NÃO avaliar os valores até que você execute o programa.
    * Em teoria, isso permite uma melhor concatenação de literais no Python 2 e Python 3.
    * Na realidade, você provavelmente não o usará muito e, em vez disso, usará uma string diretamente.

```python
turning_speed_f = LaunchConfiguration('turning_speed')
```

* O LaunchConfiguration() criará este objeto chamado turning_speed_f, que representa o valor futuro daquele argumento. Isso significa que você não pode usá-lo com operações padrão do Python. Sempre use estes sistemas de substituição.

```python
PathJoinSubstitution([
    FindPackageShare(package_description),
    'launch',
    'start_rviz_with_arguments.launch.py'
])
```

* Você fez isso da maneira padrão do Python antes:

```python
PythonLaunchDescriptionSource(
    os.path.join(pkg_box_bot_gazebo, 'launch', 'start_rviz.launch.py'),
)
```

No entanto, FindPackageShare é oficialmente recomendado pela API ROS.

```python
launch_arguments={'rviz_config_file_name': rviz_config_file_name_f}.items()
```

Os argumentos de inicialização são passados ​​assim. Observe que você sempre precisa passar uma versão em lista do dicionário. É por isso que você usa o método items().

```python
GroupAction(
    actions=[
        # push-ros-namespace to set namespace of included nodes
        PushRosNamespace(custom_namespace_f),
```

Ao usar esta ação de grupo, basicamente, você pode, dentro desta matriz, colocar inclusões para iniciar arquivos, outras ações como executar comandos ou, neste caso, definir um namespace para todos os nós internos.
        
## Arquivos de launch usando XML
Veja a baixo a mesma execução da launch **python_main.launch.py**, porém usando XML.

```xml
<launch>

    <arg name="turning_speed" default="0.0"/>
    <arg name="forward_speed" default="0.0"/>
    <arg name="rviz_config_file_name" default="launch_part.rviz"/>
    <arg name="custom_namespace" default="gypsi_danger"/>


    <include file="$(find-pkg-share launch_tests_pkg)/launch/start_rviz_with_arguments.launch.py">
        <arg name="rviz_config_file_name" value="$(var rviz_config_file_name)"/>
    </include>


  <group>
    <push-ros-namespace namespace="$(var custom_namespace)"/>
    <include file="$(find-pkg-share launch_tests_pkg)/launch/move_with_arguments.launch.xml">
        <arg name="turning_speed" value="$(var turning_speed)"/>
        <arg name="forward_speed" value="$(var forward_speed)"/>
    </include>
  </group>

</launch>
```

A sintaxe é semelhante à que você encontrou no ROS1. No entanto, há algo diferente que vale a pena observar:

args agora são acessados ​​usando a sintaxe:

```xml
$(var argument_name)
```

Veja a baixo a mesma execução da launch **move_with_arguments.launch.py**, porém usando XML.

```xml
<launch>

    <arg name="turning_speed" default="0.0"/>
    <arg name="forward_speed" default="0.0"/>

    <node pkg="launch_tests_pkg" exec="move_robot_with_params_exe" name="move_robot_node">
        <param name="turning_speed" value="$(var turning_speed)"/>
        <param name="forward_speed" value="$(var forward_speed)"/>
    </node>

</launch>
```

Observe que aqui, como entrada do nó, você não está usando argumentos, mas parâmetros. Portanto, você precisa criar uma versão do **move_robot_with_arguments.cpp** (que é executado com a luanch.py) que funcione com parâmetros. [Veja as modificações nesse executável **move_robot_with_params.cpp**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/src/move_robot_with_params.cpp)

Vamos fazer uma rápida revisão do código:

```cpp
this->declare_parameter("turning_speed", 0.0);
this->declare_parameter("forward_speed", 0.0);
```

Declare os parâmetros que serão as entradas deste nó. Você deve declarar um valor padrão; caso contrário, você receberá avisos agora e erros em versões futuras do ROS2.

```cpp
turning_speed = this->get_parameter("turning_speed").get_parameter_value().get<float>();
forward_speed = this->get_parameter("forward_speed").get_parameter_value().get<float>();
```

Para obter os valores dos parâmetros, você deve seguir esta sintaxe:

```cpp
get_parameter('parameter_name').get_parameter_value().get<type_of_value>()
```

Aqui está um guia de migração para portar seu lançamento do ROS1 em XML para versões XML do ROS2: [GUIA DE MIGRAÇÃO DE LANÇAMENTO DO ROS1 PARA ROS2 XML](https://docs.ros.org/en/galactic/How-To-Guides/Launch-files-migration-guide.html).


Execute com o comando: `ros2 launch launch_tests_pkg xml_main.launch.xml turning_speed:=0.1 forward_speed:=10.0`

## Arquivos de launch usando YAML
[Veja uma launch **yaml_main.launch.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/launch/yaml_main.launch.yaml).

Veja a [**move_with_arguments.launch.yaml**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/launch/move_with_arguments.launch.yaml) que executa **move_robot_with_params.cpp**. 

Execute com o comando `ros2 launch launch_tests_pkg yaml_main.launch.yaml turning_speed:=1.0 forward_speed:=1.0 rviz_config_file_name:=launch_part_2.rviz`.

## LogInfo
O que é **launch.actions.LogInfo()** ?

No arquivo **launch** podemos  usar um recurso de ação de inicialização chamado **LogInfo**:

```python
# This is to publish messages inside Launch files.
message_info = launch.actions.LogInfo(msg=str(rviz_config_dir))

...

return LaunchDescription(
        [
            ...,
            message_info
        ]
    )
```

O LogInfo permite que você publique uma mensagem dentro do sistema de registro ROS2 a partir de um arquivo de inicialização. 

Neste caso, a mensagem a ser impressa é o caminho completo para o arquivo de configuração do Rviz. Isso pode ser útil para verificar se estamos carregando o arquivo de configuração correto.

Para imprimir uma mensagem durante a inicialização:

* crie uma variável LogInfo, com o argumento (msg='a mensagem em string que você deseja imprimir').
* e então forneça essa variável para LaunchDescription.

## Executando várias launch a partir de uma unica launch
No exemplo a seguir, o arquivo de inicialização é responsável por iniciar os outros dois arquivos de inicialização. Para isso, usamos a ação **IncludeLaunchDescription**, bem como a função **PythonLaunchDescriptionSource**.

Para iniciar dois arquivos de inicialização dentro de um único arquivo de inicialização é com a seguinte estrutura:

```python
name_of_launch_object = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MY_LAUNCH_FILE_PATH)
        )
    )

name_of_launch_object_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MY_LAUNCH_FILE_PATH_2)
        )
    )
    
...


return LaunchDescription([
        name_of_launch_object,
        name_of_launch_object_2,

    ])
```

* A função **PythonLaunchDescriptionSource** deve conter o caminho completo para o arquivo de inicialização.
* A ação **IncludeLaunchDescription** deve conter um objeto PythonLaunchDescriptionSource com o caminho para o arquivo de inicialização.

[Veja essa launch (main.launch.py)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/launch/main.launch.py) que executa outras duas launchs (move.launch.py e start_rviz.launch.py)

## Passando parâmetros em arquivos de inicialização
Outro conceito importante a ser entendido é como passar argumentos entre arquivos de inicialização e nós no ROS 2. 

Veja a seguir uma launch que imprime os valores dos argumentos que você passa pelo sistema de registro ROS2:

```python
import launch
# These log actions are: https://github.com/ros2/launch/blob/master/launch/launch/actions 
# Logging, for example: https://github.com/ros2/launch/blob/master/launch/launch/actions/log_info.py

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('extra_msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('important_msg'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('extra_msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('important_msg')),
    ])
```

Este arquivo de inicialização faz duas coisas:

* Declara dois argumentos como argumentos para este arquivo de inicialização. Um deles (**msg**) tem um valor padrão que não é obrigatório. O outro argumento é obrigatório porque não tem valor padrão.
* Use **launch.actions.LogInfo** para registrar algumas mensagens.

Ao executar a launch (**ros2 launch launch_args_example_pkg start_with_arguments_dummy.launch.py extra_msg:="ROS2 is Great!" important_msg:="Execute Order 66"**) será mostrado:

```shell
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2022-01-18-14-21-31-913612-1_xterm-3124
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: ROS2 is Great!
[INFO] [launch.user]: Execute Order 66
```

Veja a seguir uma launch que recebe como argumentos: 
* duas strings que serão impressas alternadamente
* um float que será usado para imprimir essa mensagem em um determinado período.

E fornece esses argumentos para um nó executado pela launch.

```python
import launch
from launch_ros.actions import Node

# How to use Example:
# ros2 launch execution_and_callbacks_examples start_with_arguments.launch.py timer_period:=0.5


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg_A', default_value='Tick'),
        launch.actions.DeclareLaunchArgument('msg_B', default_value='Tack'),
        launch.actions.DeclareLaunchArgument('timer_period'),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('msg_A')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('msg_B')),
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('timer_period')),
        # All the arguments have to be strings. Floats will give an error of NonItreable.
        Node(
            package='launch_args_example_pkg',
            executable='arguments_examples_demo',
            output='screen',
            emulate_tty=True,
            arguments=["-timer_period_message", launch.substitutions.LaunchConfiguration(
                'msg_A'),
                launch.substitutions.LaunchConfiguration('msg_B'),
                "-timer_period", launch.substitutions.LaunchConfiguration(
                'timer_period')
            ]
        ),
    ])
```

[No node (**move_robot_node**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/launch_tests_pkg/src/move_robot.cpp) observe que passamos os argumentos para o construtor da classe:

```cpp
DummyArgumetsExample(int &argc, char **argv)
```

Obtemos os argumentos dentro do construtor da classe:

```cpp
message1 = argv[2];
message2 = argv[3];
timer_period = argv[5];
```

Observe como convertemos o argumento timer_period para o tipo std::chrono::duration, que é exigido pelo método create_wall_timer.

```cpp
timer_period = argv[5];
float tp1 = std::stof(timer_period);
auto tp2 = std::chrono::duration<double>(tp1);

timer_ = this->create_wall_timer(
    tp2, std::bind(&DummyArgumetsExample::timer_callback, this));
```

Ao executar a launch com o comando `ros2 launch launch_args_example_pkg start_with_arguments.launch.py timer_period:=0.5` sera exibido:

```shel
user:~/ros2_ws$ ros2 launch launch_args_example_pkg start_with_arguments.launch.py timer_period:=0.5
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2025-08-20-20-55-36-581281-1_xterm-9514
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Tick
[INFO] [launch.user]: Tack
[INFO] [launch.user]: 0.5
[INFO] [arguments_examples_demo-1]: process started with pid [9524]
[arguments_examples_demo-1] [INFO] [1755723337.325793230] [dummy_arguments_example]: --- Tick ---
[arguments_examples_demo-1] [INFO] [1755723337.825862063] [dummy_arguments_example]: --- Tack ---
[arguments_examples_demo-1] [INFO] [1755723338.325762353] [dummy_arguments_example]: --- Tick ---
[arguments_examples_demo-1] [INFO] [1755723338.825801782] [dummy_arguments_example]: --- Tack ---
[arguments_examples_demo-1] [INFO] [1755723339.325830814] [dummy_arguments_example]: --- Tick ---
```

Ao executar a launch com o comando `ros2 launch launch_args_example_pkg start_with_arguments.launch.py msg_A:="Sith" msg_B:="Jedi" timer_period:=1.0` sera exibido:

```shell
ros2 launch launch_args_example_pkg start_with_arguments.launch.py msg_A:="Sith" msg_B:="Jedi" timer_period:=1.0
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2023-03-28-22-32-38-417407-1_xterm-27177
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Sith
[INFO] [launch.user]: Jedi
[INFO] [launch.user]: 1.0
[INFO] [arguments_examples_demo-1]: process started with pid [27179]
[arguments_examples_demo-1] [INFO] [1680042759.572392362] [dummy_arguments_example]: --- Sith ---
[arguments_examples_demo-1] [INFO] [1680042760.572349491] [dummy_arguments_example]: --- Jedi ---
[arguments_examples_demo-1] [INFO] [1680042761.572317100] [dummy_arguments_example]: --- Sith ---
[arguments_examples_demo-1] [INFO] [1680042762.572280166] [dummy_arguments_example]: --- Jedi ---
[arguments_examples_demo-1] [INFO] [1680042763.572246556] [dummy_arguments_example]: --- Sith ---
```

# Parametros de nodes
No ROS 2, os parâmetros fornecem uma maneira de configurar e personalizar nós em tempo de execução, tornando seus robôs mais flexíveis e adaptáveis ​​a diferentes ambientes e tarefas. Os parâmetros permitem definir valores para variáveis ​​que podem afetar o comportamento dos nós sem a necessidade de modificar o código em si. Isso facilita a adaptação a mudanças nas condições, o ajuste do comportamento dos robôs ou o ajuste de configurações em diferentes sistemas.

Nesta unidade, você aprenderá a trabalhar com parâmetros no ROS 2, incluindo como definir, configurar, obter e atualizar parâmetros em seus nós. Você também explorará os diferentes tipos de parâmetros, seu uso e como eles ajudam a criar aplicações robóticas mais dinâmicas e reutilizáveis.

Ao final desta unidade, você poderá trabalhar com parâmetros no ROS 2 com confiança para melhorar a flexibilidade e o desempenho do seu robô.

**alguns comando uteis**:

* Obter o valor de um parâmetro específico: `ros2 param get <node_name> <parameter_name>`
* Defina o valor de um parâmetro específico: `ros2 param set <node_name> <parameter_name> <parameter_value>`
* Generate a YAML file with the current parameters of a node: `ros2 param dump <node_name>`
* Carregue os parâmetros de um arquivo YAML em um nó em execução: `ros2 param load <node_name> <parameter_file>`

## Como os parâmetros funcionam no ROS 2?
Parâmetros são valores usados ​​para configurar nós no ROS 2. Diferentemente do ROS 1, onde os parâmetros são compartilhados entre todos os nós por meio de um **roscore central**, no ROS 2 os parâmetros são específicos para cada nó individual. Essa é uma diferença fundamental que permite maior flexibilidade e modularidade na configuração de sistemas robóticos.

No ROS 2, quando um nó é desligado, seus parâmetros associados também são apagados. Os parâmetros podem ser definidos no momento da inicialização do nó ou dinamicamente enquanto o nó está em execução, permitindo ajustes em tempo real no comportamento de um robô sem a necessidade de reiniciá-lo.

Ao executar [este nó (**parameter_tests_node.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/parameter_tests/src/parameter_tests_node.cpp) com o comando: `ros2 run parameter_tests param_vel_node`.

Será exibido: 

```shell
[INFO] [1645178156.488937533] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645178158.454250656] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645178160.454262152] [param_vel_node]: Velocity parameter is: 0.000000
        
...
```

Parece que seu nó contém um parâmetro para configurar a velocidade.

Vamos analisar o código:

```cpp
VelParam()
  : Node("param_vel_node")
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Sets the velocity (in m/s) of the robot.";
  this->declare_parameter<std::double_t>("velocity", 0.0, param_desc);
  timer_ = this->create_wall_timer(
  1000ms, std::bind(&VelParam::timer_callback, this));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}
```

A primeira coisa a fazer é definir um **ParameterDescriptor** para fornecer alguns dados extras sobre ele.

```cpp
auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
param_desc.description = "Sets the velocity (in m/s) of the robot.";
```

Não é obrigatório criar um descritor para seus parâmetros. No entanto, é sempre útil fornecer dados sobre eles, especialmente para nós complexos com muitos parâmetros.

Em seguida, declare seu parâmetro e forneça:

* O nome do parâmetro: velocidade
* Um valor padrão para o parâmetro: 0,0.
* O descritor de parâmetro criado anteriormente

```cpp
this->declare_parameter<std::double_t>("velocity", 0.0, param_desc);
```

Em seguida, crie um objeto timer. Este objeto timer é anexado a um **timer_callback** que será executado 1 vez por segundo:

```cpp
timer_ = this->create_wall_timer(1000ms, std::bind(&VelParam::timer_callback, this));
```

Por fim, crie um objeto publicador para publicar mensagens no tópico **/cmd_vel** para mover o robô:

```cpp
publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
```

Certo, você encontra o timer_callback:

```cpp
void timer_callback()
{
  this->get_parameter("velocity", vel_parameter_);
  RCLCPP_INFO(this->get_logger(), "Velocity parameter is: %f", vel_parameter_);
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = vel_parameter_;
  publisher_->publish(message);
}
```

A linha mais importante aqui é onde você obtém o valor deste parâmetro.

```cpp
this->get_parameter("velocity", vel_parameter_);
```

Em seguida, defina o valor do parâmetro como a velocidade do robô e envie esta mensagem ao robô:

```cpp
message.linear.x = vel_parameter_;
publisher_->publish(message);
```





## Interaja com parâmetros das ferramentas de linha de comando
Primeiro, obtenha uma lista de todos os parâmetros disponíveis atualmente com o comando `ros2 param list`:

```shell
...
/param_vel_node:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
  velocity
        
...
```

Como você pode ver, cada nó (gazebo, moving_service, param_vel_node...) contém seus próprios parâmetros. Se você pesquisá-los, encontrará o nó que você iniciou, param_vel_node, com um parâmetro chamado velocity.

Também é possível listar os parâmetros de um nó específico `ros2 param list /param_vel_node`:

```shell
qos_overrides./parameter_events.publisher.depth
qos_overrides./parameter_events.publisher.durability
qos_overrides./parameter_events.publisher.history
qos_overrides./parameter_events.publisher.reliability
use_sim_time
velocity
```

Cada nó possui o parâmetro **use_sim_time**. Ele é gerado automaticamente e usado para gerenciar o Tempo ROS: https://design.ros2.org/articles/clock_and_time.html

Além disso, você pode ver um conjunto de parâmetros no namespace **qos_overrides**:

```shell
qos_overrides./parameter_events.publisher.depth
qos_overrides./parameter_events.publisher.durability
qos_overrides./parameter_events.publisher.history
qos_overrides./parameter_events.publisher.reliability
```

Esses parâmetros definem as **configurações de QoS** do tópico **parameter_events**. Este tópico é usado para monitorar alterações de parâmetros. Esses parâmetros são **somente leitura, portanto, não podem ser alterados**.

Agora obtenha mais dados sobre este parâmetro de velocidade: `ros2 param describe /param_vel_node velocity`

```shell
Parameter name: velocity
  Type: double
  Description: Sets the velocity (in m/s) of the robot.
  Constraints:
```

Você pode ver que o parâmetro de velocidade usa um tipo double, que parece configurar a velocidade do robô. Legal!

Você também pode obter o valor atual deste parâmetro de velocidade com o seguinte comando: `ros2 param get /param_vel_node velocity`

Para ter acesso a um parâmetro, você deve primeiro especificar o nó ao qual ele pertence.

```shell
Double value is: 0.0
```

E se você quiser alterar o valor deste parâmetro?

Você pode alterar o valor de um parâmetro em tempo de execução com o comando ros2 param set: `ros2 param set /param_vel_node velocity 0.1`

Observe que se você usar o valor **inteiro** 0 : `ros2 param set /param_vel_node velocity 0`

você obterá um erro como o abaixo:

```shell
Setting parameter failed: Wrong parameter type, parameter {velocity} is of type {double}, setting it to {integer} is not allowed.
```

## Carregando e despejando parâmetros de arquivos YAML
Às vezes, é útil obter uma cópia dos parâmetros específicos do nó. Isso permite que você execute um nó com uma configuração específica quando necessário, por exemplo. Você pode fazer isso com o seguinte comando: `ros2 param dump /param_vel_node`

Será exibido:
```shell
Saving to:  ./param_vel_node.yaml
```

Isso gerará um arquivo chamado param_vel_node.yaml no caminho onde você executa o comando com o seguinte conteúdo:

```yaml
/param_vel_node:
  ros__parameters:
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
    velocity: 0.1
```

Como você pode ver, os parâmetros são armazenados no formato de arquivo YAML. É claro que carregar esse arquivo YAML em um nó em execução também é possível. 

Observe que você removeu os parâmetros qos_overrides, pois eles são somente leitura e não podem ser modificados.

Agora, carregue o arquivo YAML com o seguinte comando: `ros2 param load /param_vel_node param_vel_node.yaml`

Também é possível carregar parâmetros de um arquivo ao iniciar um nó: `ros2 run parameter_tests param_vel_node --ros-args --params-file /home/user/param_vel_node.yaml`

## Configurando parâmetros via linha de comando na inicialização do nó
Por fim, também é possível definir parâmetros na linha de comando ao iniciar um nó: `ros2 run parameter_tests param_vel_node --ros-args -p velocity:=0.1`

## Carregando parametros no arquivos launch
Também é possível definir parâmetros a partir de um arquivo de inicialização. Nesse caso, os parâmetros serão definidos na inicialização, não na execução. Siga as instruções descritas abaixo para saber como.

[Veja essa launch (**test_parameters.launch.py**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/parameter_tests/launch/test_parameters.launch.py) como exemplo.

## Parâmetros de callBack
Como você pode ver, no ROS2, você pode interagir e modificar parâmetros a qualquer momento. Sempre que um parâmetro de nó for atualizado, você pode notificá-lo sobre essa alteração para que ele possa tomar as medidas necessárias, se necessário.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/parameter_tests/src/parameter_tests_callback.cpp) um nó (**parameter_tests_callback.cpp**) que chama a função **parameter_callback**, sempre que você definir um novo valor para o parâmetro **velocity**. 

Ao definir uma velocidade de 0.1 m/s: `ros2 param set /param_vel_node velocity 0.1`:

```shell
...
[INFO] [1646392513.493424804] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1646392514.493439021] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1646392515.139746729] [param_vel_node]: Parameter 'velocity' changed!
[INFO] [1646392515.493460144] [param_vel_node]: Velocity parameter is: 0.100000
[INFO] [1646392516.493485543] [param_vel_node]: Velocity parameter is: 0.100000
...
```

Ao definir uma velocidade de 0.3 m/s: `ros2 param set /param_vel_node velocity 0.3`:

```shell
Setting parameter failed: Parameter 'velocity' cannot be higher than 0.2
```

Vamos dar uma olhada no código:

```cpp
callback_handle_ = this->add_on_set_parameters_callback(std::bind(&VelParam::parametersCallback, this, std::placeholders::_1));
```

Esta linha indica que, quando um novo parâmetro é definido para este nó, a função **parametersCallback** deve ser acionada. Portanto, você precisará usar o método **add_on_set_parameters_callback** se quiser adicionar um retorno de chamada de parâmetro ao seu nó.

A resposta desta função parametersCallback será armazenada em **callback_handle_**.

É claro que você precisa definir esta variável de identificador de chamada de retorno:

```cpp
OnSetParametersCallbackHandle::SharedPtr callback_handle_;
```

Por fim, você tem a implementação desta função **parametersCallback**:

```cpp
rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    for (const auto &parameter : parameters)
    {
        if (parameter.get_name() == "velocity" &&
            parameter.as_double() > 0.2)
        {
            RCLCPP_INFO(this->get_logger(), "Parameter 'velocity' not changed!");
            result.reason = "Parameter 'velocity' cannot be higher than 0.2";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Parameter 'velocity' changed!");
            result.successful = true;
            result.reason = "Parameter 'velocity' is lower than 0.2";
        }
    }
    return result;
}
```

Esta função de retorno de chamada retornará um objeto **SetParametersResult**:

```cpp
rcl_interfaces::msg::SetParametersResult result;
result.successful = false;
result.reason = "";
```

Como você pode ver, este objeto SetParametersResult contém duas variáveis:

* **successful**: um sinalizador booleano para indicar se o parâmetro foi atualizado (true) ou não (false).
* **reason**: uma string para fornecer mais informações sobre o motivo da alteração do parâmetro.

Em seguida, verifique se o parâmetro velocity possui um valor maior que 0,2:

```cpp
if (parameter.get_name() == "velocity" && parameter.as_double() > 0.2)
{
    RCLCPP_INFO(this->get_logger(), "Parameter 'velocity' not changed!");
    result.reason = "Parameter 'velocity' cannot be higher than 0.2";
}
```

Se for maior, deixe a variável "sucesso" como falsa; portanto, você não atualizará o parâmetro. Além disso, atualize a variável "reason" para indicar por que você não está atualizando o parâmetro.

Se o parâmetro "velocity" não tiver um valor maior que 0,2, atualize a variável "sucesso" para verdadeira, permitindo assim que o parâmetro seja atualizado:

```cpp
else
{
    RCLCPP_INFO(this->get_logger(), "Parameter 'velocity' changed!");
    result.successful = true;
    result.reason = "Parameter 'velocity' is lower than 0.2";
}
```

# Gerenciando nós complexos com C++
Aplicações robóticas modernas frequentemente exigem que múltiplas tarefas sejam executadas simultaneamente, como processamento de dados de sensores, execução de planejamento de movimento e comunicação com outros sistemas. É aqui que o **multithreading** se torna essencial. O multithreading permite que diferentes partes de um programa sejam executadas simultaneamente, melhorando o desempenho e a responsividade.

No ROS 2, **executores**, grupos de retorno de chamada (**callback groups**) e **WaitSet** são mecanismos-chave para gerenciar a execução multithread. 

O **WaitSet** permite aguardar múltiplos eventos simultaneamente, possibilitando um tratamento mais eficiente de tarefas assíncronas. Ao utilizar multithreading, executores, grupos de retorno de chamada e WaitSet, você aumentará a eficiência e a responsividade de suas aplicações ROS 2, garantindo que elas possam lidar com tarefas complexas em tempo real de forma eficaz.

## Uso de Executores
O rclcpp fornece três tipos de Executores, derivados de uma classe pai compartilhada:

* Executor **Single-Threaded**: Ele manipula apenas uma thread no nó.
* Executor **Multi-Threaded**: Ele cria um número variável de threads que permite que múltiplas mensagens/eventos sejam processados ​​em paralelo.

Em ambos os Executores **Single-Threaded/Multi-Threaded**, se você adicionar/remover/alterar assinaturas, temporizadores, servidores de serviço e servidores de ação, ele **se adaptará a essas alterações** escaneando o nó.

Se você não precisar disso e tiver APENAS uma thread, você pode usar:

* Executor **Static Single-Threaded**: Ele executa uma varredura de nó apenas uma vez quando o nó é adicionado ao Executor. Use-o apenas com nós que criam todos os Callbacks relacionados durante a inicialização.

Este é o primeiro exemplo do problema que você terá se usar SINGLE THREAD EXECUTOR. Ao executar `ros2 run executors_exercises_pkg executor_example_3_static_node` desse [pacote](https://github.com/marcospontoexe/ROS_2/tree/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg) verá o seguinte:

```shell
[INFO] [1649179947.273255770] [slow_timer_subscriber]: slow_timer_node INFO...
[INFO] [1649179950.762544006] [slow_timer_subscriber]: TICK
[INFO] [1649179953.762827503] [slow_timer_subscriber]: TICK
[INFO] [1649179953.762957992] [odom_subscriber]: Odometry=['0.275937','-0.902710','0.109371']
[INFO] [1649179956.763160708] [slow_timer_subscriber]: TICK
[INFO] [1649179956.763276857] [odom_subscriber]: Odometry=['0.275936','-0.902710','0.109371']
[INFO] [1649179959.763499246] [slow_timer_subscriber]: TICK
[INFO] [1649179959.763618150] [odom_subscriber]: Odometry=['0.275935','-0.902710','0.109371']
```

Então, por que isso acontece?

A resposta é que você tem apenas UMA THREAD no EXECUTOR porque usa o Executor Single-Threaded ou o Executor Estático Single-Threaded.

Portanto, a solução lógica seria usar o [Executor Multi-Threaded](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_4.cpp). 

Neste primeiro exemplo, você está alterando o tipo de Executor para Multithread.
Como você tem apenas UM RETORNO DE CHAMADA por Nó, não haverá problemas.
Isso ocorre porque o Executor Multithread inicia UMA THREAD POR NÓ ADICIONADO.

Ao executar `touch executors_exercises_pkg/src/executor_example_4_singlethreaded.cpp` verá o seguinte:

```shell
[INFO] [1649239417.277080517] [odom_subscriber]: Odometry=['0.210631','1.008857','0.109481']
[INFO] [1649239417.296441313] [odom_subscriber]: Odometry=['0.210631','1.008857','0.109481']
[INFO] [1649239417.311212740] [odom_subscriber]: Odometry=['0.210631','1.008857','0.109481']
[INFO] [1649239417.331764518] [odom_subscriber]: Odometry=['0.210630','1.008857','0.109481']
[INFO] [1649239417.341036044] [odom_subscriber]: Odometry=['0.210630','1.008857','0.109481']
[INFO] [1649239417.350275506] [odom_subscriber]: Odometry=['0.210629','1.008857','0.109481']
[INFO] [1649239417.373895865] [odom_subscriber]: Odometry=['0.210629','1.008857','0.109481']
[INFO] [1649239417.383497167] [odom_subscriber]: Odometry=['0.210628','1.008857','0.109481']
[INFO] [1649239417.392826479] [odom_subscriber]: Odometry=['0.210628','1.008857','0.109481']
[INFO] [1649239417.415700319] [odom_subscriber]: Odometry=['0.210627','1.008857','0.109481']
[INFO] [1649239417.428317525] [odom_subscriber]: Odometry=['0.210627','1.008857','0.109481']
[INFO] [1649239417.440443096] [odom_subscriber]: Odometry=['0.210626','1.008857','0.109481']
[INFO] [1649239417.457971920] [slow_timer_subscriber]: TICK
[INFO] [1649239417.461313452] [odom_subscriber]: Odometry=['0.210626','1.008857','0.109481']
[INFO] [1649239417.482754192] [odom_subscriber]: Odometry=['0.210626','1.008857','0.109481']
```

Você pode ver um TICK a cada três segundos, mas a odometria não para. Isso significa que ambos os Callbacks estão funcionando simultaneamente.

Como exemplo adicional, veja como você acha que poderia ter o mesmo comportamento, mas usando [Executores de thread única e múltiplos](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_4_singlethreaded.cpp).

Como você pode ver, você está criando dois Executores Single Threaded e, em seguida, atribui a cada um deles apenas UM Nó.
Em termos de desempenho, Single Threaded consome muito menos CPU do que Executores Multi-Threaded. Portanto, é sempre preferível.
Outra opção seria criar binários diferentes juntos. Seria o mesmo que neste exemplo, mas em binários separados.

execute `ros2 run executors_exercises_pkg executor_example_4_singlethreaded_node`:


```shell
[INFO] [1649253811.051878378] [odom_subscriber]: Odometry=['0.395768','1.000205','0.109481']
[INFO] [1649253811.061593590] [odom_subscriber]: Odometry=['0.395768','1.000205','0.109481']
[INFO] [1649253811.073167454] [odom_subscriber]: Odometry=['0.395767','1.000205','0.109481']
[INFO] [1649253811.082616228] [odom_subscriber]: Odometry=['0.395767','1.000205','0.109481']
[INFO] [1649253811.105497169] [odom_subscriber]: Odometry=['0.395767','1.000205','0.109481']
[INFO] [1649253811.121808076] [odom_subscriber]: Odometry=['0.395766','1.000205','0.109481']
[INFO] [1649253811.135817744] [odom_subscriber]: Odometry=['0.395766','1.000205','0.109481']
[INFO] [1649253811.158670391] [odom_subscriber]: Odometry=['0.395765','1.000205','0.109481']
[INFO] [1649253811.180828865] [odom_subscriber]: Odometry=['0.395765','1.000205','0.109481']
```

Como você pode ver, isso não funciona. Isso ocorre porque você só pode ter UM ÚNICO EXECUTOR por binário.

## Uso de Callback Groups
Até agora, você aprendeu a trabalhar com executores. No entanto, para lidar adequadamente com múltiplos retornos de chamada (Callback), muitas vezes você precisará de ferramentas de gerenciamento adicionais. É aqui que os grupos de retorno de chamada (Callback Groups) entram em ação!

[Veja neese exemplo (**executor_example_5.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_5.cpp):

* Aqui, você tem uma classe Node com MAIS de um Callback, neste caso, DOIS.
* Veja o que acontece, porque, com base no que você explicou anteriormente, o Executor **Multi-Threaded** inicia apenas UMA THREAD por Node, a menos que você especifique o contrário.

Após executar: `ros2 run executors_exercises_pkg executor_example_5_node`, verá o seguinte:

```shell
[INFO] [1649243091.314707167] [slow_timer_subscriber]: timer_1_node INFO...
[INFO] [1649243092.774541260] [slow_timer_subscriber]: TIMER CALLBACK 1
[INFO] [1649243093.776366590] [slow_timer_subscriber]: TIMER CALLBACK 1
[INFO] [1649243094.776625513] [slow_timer_subscriber]: TIMER CALLBACK 1
[INFO] [1649243095.776895726] [slow_timer_subscriber]: TIMER CALLBACK 1
[INFO] [1649243096.777163350] [slow_timer_subscriber]: TIMER CALLBACK 1
```

* Você vê que SOMENTE o PRIMEIRO RETORNO DE CHAMADA é executado.
* O motivo pelo qual apenas um Retorno de Chamada é executado é que o Executor criou apenas uma thread por Nó. Portanto, você tem apenas uma thread para executar tudo; portanto, não é possível executar vários Retornos de Chamada em paralelo.
* E o motivo pelo qual o timer_callback_1 é executado é que ele foi instanciado primeiro no construtor.

**O que você pode fazer?**

Introduzir um novo conceito chamado **Grupos de Callback**.

Grupos de Callback permitem agrupar diferentes Callbacks para que alguns sejam executados em paralelo e outros não. Existem DOIS tipos:

* **Reentrant**: Todos os Callbacks nesses tipos de grupos poderão ser executados simultaneamente. O Executor gerará quantas threads forem necessárias para isso.
* **MutualyExclusive**: Todos os Callbacks dentro desses tipos de grupos podem ser executados **UM de cada vez**. Isso é útil para Callbacks que, por algum motivo, afetam o mesmo sistema ou usam os mesmos recursos que outro. Além disso, oferece melhor controle sobre o fluxo de Callback.

* [Veja um exemplo de **Reentrant**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_5_reentrant.cpp).

A única mudança real é adicionar esses callback_groups aos assinantes, serviços ou, neste caso, temporizadores.

* Neste caso, ambos usam o mesmo Grupo de Callback do tipo Reentrante.

```cpp
callback_group_ =this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

...

timer1_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_1, this), callback_group_);
timer2_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_2, this), callback_group_);
```

Ao executar: `ros2 run executors_exercises_pkg executor_example_5_reentrant_node`, verá o seguinte:

```shell
[INFO] [1649245219.694108351] [slow_timer_subscriber]: Wating...TIMER CALLBACK 2
[INFO] [1649245219.698429480] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245219.698525651] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245220.699197731] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245220.699308479] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245221.699431638] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245221.699605561] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245222.694242393] [slow_timer_subscriber]: End TIMER CALLBACK 2
```

Você pode ver que funciona perfeitamente. Porque o Callback 1 é chamado a cada um segundo e o Callback 2 a cada três segundos. Você vê exatamente esse comportamento, então eles funcionam em paralelo.

* [Veja um exemplo de **MutualyExclusive**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_5_mutualyexclusive.cpp).

Você alterou o tipo de Grupo de retorno de chamada para Mutuamente exclusivo:

```cpp
callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

ao executar: `ros2 run executors_exercises_pkg executor_example_5_mutualyexclusive_node` verá o seguinte:

```shell
[INFO] [1649245342.195628849] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245343.195787567] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245343.195980444] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245344.196122558] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245344.196328147] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245345.196444581] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245345.196621215] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
```

Aqui, o comportamento é exatamente o mesmo sem Grupos de Callback. Isso ocorre porque, por padrão, os Callbacks em um Nó estão TODOS dentro do mesmo Grupo de Callback do tipo Mutuamente Exclusivo.

* [Veja um exemplo de **mutualyexclusive_multiple**](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/executor_example_5_mutualyexclusive_multiple.cpp).

E, neste exemplo, você está dando a cada um dos temporizadores seu próprio Grupo de Retorno de Chamada. Isso o torna igual à versão Reentrante.

```cpp
callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

...

timer1_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_1, this), callback_group_);
timer2_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_2, this), callback_group_2);
```

Ao executar: `ros2 run executors_exercises_pkg executor_example_5_mutualyexclusive_multiple_node` verá o seguinte: 

```shell
[INFO] [1649245460.033089561] [slow_timer_subscriber]: Wating...TIMER CALLBACK 2
[INFO] [1649245460.033469544] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245460.033586291] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245461.033712353] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245461.033941159] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245462.034098058] [slow_timer_subscriber]: End TIMER CALLBACK 1
[INFO] [1649245462.034267823] [slow_timer_subscriber]: Wating...TIMER CALLBACK 1
[INFO] [1649245463.033231549] [slow_timer_subscriber]: End TIMER CALLBACK 2
[INFO] [1649245463.033400761] [slow_timer_subscriber]: Wating...TIMER CALLBACK 2
```

Aqui, como você tem um Grupo de Callback por Callback, o Executor cria uma thread para cada Grupo de Callback. Portanto, o comportamento é o mesmo do Grupo de Callback único Reentrante.

## Uso de WaitSet
A maioria dos aplicativos de robótica que você criará com o ROS2 Multi-Threading já são suficientes para o que você acabou de ver. No entanto, há algumas ocasiões em que isso não é suficiente.

É aqui que entra a estrutura WaitSet. Ela permite controlar tópicos, temporizadores, condições de disparo e muito mais de forma controlada.

[Veja um exemplo (**wait_for_box_bots_arrive.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/wait_for_box_bots_arrive.cpp).

Alguns ponto importantes:

``cpp
// Minimum 2 {} {} to avoid problems in declaration.
rclcpp::WaitSet wait_set({}, {});

wait_set.add_subscription(sub1);
wait_set.add_subscription(sub2);
wait_set.add_subscription(sub3);
```

* Aqui, inicialize o objeto wait_set.
* Você precisa adicionar os `{}` vazios para que o construtor funcione corretamente. O motivo é a maneira como o construtor e os modelos funcionam. Existem outras maneiras de fazer isso, mas esta é a mais simples.
* Após a inicialização do objeto, você pode adicionar diferentes funções invocáveis, como:
    * add_subscritions
    * add_guard_condition
    * add_client
    * add_waitable
    * add_service
    * add_timer
* Nesta unidade, você verá apenas exemplos de add_subscritions e add_guard_condition, pois são os básicos.

```cpp
const auto wait_result = wait_set.wait(3s);
```

* É assim que você define o tempo limite.

```cpp
wait_result.kind() == rclcpp::WaitResultKind::Ready
```

* Este tipo de função fornece o status do WaitSet. Os básicos são:
    * **Timeout**: Nada aconteceu no tempo limite especificado.
    * **Empty**: Aqui é onde você não adicionou nada ao wait_set. É útil quando você usa a função remove_elements do wait_set. Não é mostrado aqui.
    * **Ready**: Algo aconteceu. Um dos elementos internos foi acionado.

```cpp
wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U];
```

* É assim que você verifica se há novas mensagens no objeto de assinatura.
* Observe que você não está usando **int** como índice, mas sim o tipo **size_t**. Portanto, é o {INDEXNUMBER}{U}, ou você pode inicializá-lo assim:

```cpp
// For index 0, instead of using dirrectly 0U
size_t guard_index = 0;
```

E é assim que você obtém os dados da mensagem, um de cada vez.

```cpp
sub1->take(topic_msg, msg_info);
```

### Otimizando o uso do CPU
Se quiser otimizar o impacto na CPU, você pode usar StaticWaitSet. Este é um melhor desempenho porque, após a inicialização, os elementos não podem ser adicionados ou removidos.

[Aqui (**wait_for_box_bots_arrive_static.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/wait_for_box_bots_arrive_static.cpp), DEFINA EXPLICITAMENTE TODOS OS ELEMENTOS NECESSÁRIOS para o WaitSet, mesmo que NÃO HAJA NENHUM:

Como você pode ver, esta é a única parte que mudou:

Indique através de TEMPLATES os elementos dentro do StaticWaitSet

```cpp
rclcpp::StaticWaitSet<3, 0, 0, 0, 0, 0>
```

Sempre coloque os elementos dentro desta estrutura:

```
{{{ELEMENT_1}, {ELEMENT_2}, ..., {ELEMENT_N}}}
// If empty
{}
```

Aqui, você está adicionando TRÊS assinantes.

O comportamento é EXATAMENTE O MESMO que o WaitSet normal.

### GUARD CONDITIONS
Veja um possível uso de CONDIÇÕES DE GUARDA.
É uma boa maneira de controlar seu fluxo em assinaturas e outros elementos.
Neste caso, use UMA CONDIÇÃO DE GUARDA para detectar se houve TRÊS TIMEOUTS sem nenhuma publicação do box_bot nos tópicos atingidos. Em caso afirmativo, o Node será encerrado.

[Veja um exemplo (**wait_for_box_bots_arrive_condition.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/executors_exercises_pkg/src/wait_for_box_bots_arrive_condition.cpp).

Alguns ponto importantes:

```cpp
auto guard_condition1 = std::make_shared<rclcpp::GuardCondition>();
...
wait_set.add_guard_condition(guard_condition1);
```

* Aqui, inicialize a condição de guarda.
* E adicione-a ao objeto wait_set, usando, neste caso, o método add_guard_condition.

```cpp
bool condition1_triggered = wait_result.get_wait_set().get_rcl_wait_set().guard_conditions[guard_index];
```

Isso é o equivalente a guard_conditions para esperar por um gatilho.

```cpp
if (condition1_triggered) {
        RCLCPP_INFO(node->get_logger(), "TOO MANY TIMEOUTS CONDITION TRIGGERED...ENDING NODE");
        break;
    }
```

Se esta condição FOR DISPARADA, interrompa o loop while.

```cpp
timeout_counter += 1;
if (timeout_counter > 3) {
  RCLCPP_INFO(node->get_logger(), "Action: Trigger Guard condition 1");
  guard_condition1->trigger();
}
```

Acione-o usando o método guard_condition1->trigger().

Ao executar: `ros2 run executors_exercises_pkg wait_for_box_bots_arrive_condition_node` verá o seguinte:

Quando alguém está publicando trabalhos, **o mesmo que nos exemplos acima.
No entanto, se houver mais de TRÊS TEMPOS LIMITE, você deverá obter algo assim:

```shell
[ERROR] [1649439358.883992389] [wait_set_listener]: Wait-set failed with timeout
[ERROR] [1649439361.884370644] [wait_set_listener]: Wait-set failed with timeout
[ERROR] [1649439364.884603913] [wait_set_listener]: Wait-set failed with timeout
[ERROR] [1649439367.884840660] [wait_set_listener]: Wait-set failed with timeout
[INFO] [1649439367.884938357] [wait_set_listener]: Action: Trigger Guard condition 1
[INFO] [1649439367.884988920] [wait_set_listener]: TOO MANY TIMEOUT CONDITIONS TRIGGERED, ENDING THE NODE
```

# Qualidade de serviço (QoS)
Nesta unidade, você será apresentado à Qualidade de Serviço (QoS) no ROS 2, um conceito crítico para garantir a comunicação confiável entre nós em sistemas robóticos. As configurações de QoS determinam como os dados são transmitidos entre nós, impactando fatores como confiabilidade, latência e uso de largura de banda.

Compreender a QoS é essencial para ajustar suas aplicações ROS 2 a requisitos específicos, especialmente em ambientes robóticos em tempo real, distribuídos e críticos para a segurança. Ao final desta unidade, você terá uma compreensão clara do que é QoS e como ela pode ser usada para otimizar a comunicação em sistemas ROS 2.

Vamos nos aprofundar nos detalhes da QoS e explorar como você pode aplicá-la para melhorar o desempenho e a robustez do seu robô.

No ROS 1, o TCP era usado para transporte de mensagens. No entanto, o TCP não é ideal para redes com perdas, como Wi-Fi ou sistemas não confiáveis. Essa limitação dificultou o uso do ROS 1 por engenheiros em aplicações críticas ao sistema ou em ambientes reais.

Tudo isso mudou com o ROS 2, que usa o **UDP** como **protocolo de transporte** e utiliza o **DDS** (Serviço de Distribuição de Dados). Com essa mudança, agora você pode controlar a confiabilidade dos nós e personalizar a comunicação com base em necessidades específicas.

O ROS 2 oferece controle detalhado sobre as configurações de QoS, permitindo a configuração de parâmetros como **confiabilidade, histórico e durabilidade** para publicadores e assinantes de tópicos.

No entanto, para uma comunicação eficaz entre nodes, suas **configurações de QoS devem ser compatíveis** — isso às vezes pode ser uma fonte de problemas em sistemas ROS 2.

## Entendendo a compatibilidade QoS
[Veja esse exemplo (**subscriber_custom_minimal_qos.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/subscriber_custom_minimal_qos.cpp).

Aqui, defina **reliability=ReliabilityPolicy.RELIABLE**. Isso significa que você está forçando este assinante a receber todas as mensagens enviadas pelo publicador.

[Veja esse exemplo (**publisher_custom_minimal_qos.cpp**)](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/publisher_custom_minimal_qos.cpp).

Observe que você está DEFININDO a versão do DDS explicitamente. Isso ocorre porque cada implementação do DDS possui algumas QoS que suporta e outras que não. Dessa forma, você garante que está usando o Cyclone, pois é a versão padrão no Galactic:

```shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg publisher_custom_minimal_qos_exe -reliability reliable
```

```shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg subscriber_custom_minimal_qos_exe
```

Portanto, não há problemas ao usar uma QoS COMPATÍVEL. No entanto, o que acontece se você usar uma QoS INCOMPATÍVEL?

```shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg publisher_custom_minimal_qos_exe -reliability best_effort
```

```shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg subscriber_custom_minimal_qos_exe
```

será exibido: 

```shell
[WARN] [1680119365.479235041] [subscriber_qos_obj]: New publisher discovered on topic '/q
os_test', offering incompatible QoS. No messages will be sent to it. Last incompatible po
licy: RELIABILITY_QOS_POLICY
```

Aqui, DOIS eventos acontecem:

* No publicador, o evento **incompatible_qos** é DISPARADO, exibindo a mensagem dentro do Callback incompatible_qos_clb.

```shell
[WARN] [1680119365.479235041] [subscriber_qos_obj]: New publisher discovered on topic '/q
os_test', offering incompatible QoS. No messages will be sent to it. Last incompatible po
licy: RELIABILITY_QOS_POLICY
```

* No assinante, você recebe o aviso de QoS Incompatível.

```shell
user:~$ ros2 run qos_tests_pkg subscriber_custom_minimal_qos_exe
[WARN] [1756303912.388105403] [subscriber_qos_obj]: New publisher discovered on topic '/qos_test', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

O motivo é que a QoS do publicador e do assinante são INCOMPATÍVEIS:

* Confiabilidade da QoS do publicador = Melhor_Esforço
* Confiabilidade da QoS do assinante = Confiável

Também indica a QoS incompatível, pelo menos a última:

* Última política incompatível: CONFIABILIDADE

Esta configuração é INCOMPATÍVEL.

No entanto, como saber quais configurações são compatíveis?

Aqui você tem uma tabela para facilitar sua vida:

![qos](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/imagens/table_qos_compatibility.png)

Este curso não abordará como definir a QoS para Serviços e Ações. Se você consultar os links a seguir, perceberá que a forma como a QoS é definida para Serviços e Ações é bastante semelhante.

* [Serviços](https://docs.ros2.org/galactic/api/rclpy/api/services.html)
* [Ações](https://docs.ros2.org/galactic/api/rclpy/api/actions.html)

## Criando um subscriber
Crie um assinante para o tópico **/scan**. No entanto, agora você pode alterar todos os elementos da QoS do seu assinante.

Leve em consideração os seguintes elementos:

* O Middleware é usado pelo tópico /scan (cycloneDDS).
* A QoS deve ser compatível com o tópico /scan.
* Crie um novo script chamado **subscriber_scan_qos.py**.
* Você precisa editar as seguintes configurações de QoS:
    * history
    * reliability
    * lifespan
    * deadline
    * liveliness
    * liveliness_lease_duration
    * depth

**Observações**:

Para obter todas as informações de QoS de um tópico específico, adicione --verbose às informações do tópico ROS2: `ros2 topic info /scan --verbose`:

```shell
$ ros2 topic info /scan --verbose
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: lidar_1
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 6f.e3.10.01.07.32.a2.b4.82.0f.5c.f2.00.00.3e.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  Durability: VOLATILE
  Lifespan: 9223372036854775807 nanoseconds
  Deadline: 9223372036854775807 nanoseconds
  Liveliness: AUTOMATIC
  Liveliness lease duration: 9223372036854775807 nanoseconds

Subscription count: 0
```

Como você pode ver, /scan tem:

* Reliability = BEST_EFFORT. Esta é a maneira padrão de defini-la em sensores, pois você está interessado em obter um fluxo de dados. Não importa se você perder uma ou duas mensagens.
* Durability = Volatile. Também é a maneira padrão para sensores, especialmente sensores com alto volume de geração de dados. Você não precisa salvar mensagens antigas para assinantes que se inscreveram tardiamente. Se você perder mensagens antigas, isso é irrelevante.
ALiveliness = Automatic. Esta é a maneira padrão, especialmente para sensores. Considere que o publicador está ativo se QUALQUER um de seus tópicos publicados tiver publicado algo dentro do tempo definido pela duração do lease.
* Deadline = "9223372036,854775807" segundos (VALOR INFINITO). Isso significa que NÃO há Prazo. Você não está definindo nenhum requisito para o período entre cada mensagem publicada do tópico /scan. Um sensor real não deve ser definido dessa forma, pois, normalmente, os sensores têm uma taxa de publicação específica. No entanto, esta é uma simulação, então você pode ser menos rigoroso.
* Lifespan = "9223372036,854775807" segundos (VALOR INFINITO). Novamente, isso significa que NÃO há limitação. Não importa a idade de uma mensagem, ela é válida. Isso, novamente, normalmente não é feito em sistemas reais, pois dados antigos de sensores são inúteis. Você é menos rigoroso porque isso não acontece na simulação.

## Políticas de QoS padrão
Antes de nos aprofundarmos em políticas específicas de QoS, é importante entender as configurações padrão ao criar um publicador e um assinante no ROS 2. Essas políticas padrão são aplicadas, a menos que sejam explicitamente substituídas:

* History: "keep last" -> Esta configuração garante que o publicador mantenha apenas as mensagens mais recentes, descartando as mais antigas. É útil para tópicos em que apenas os dados mais recentes são relevantes.
* Queue Size: 10 -> Define o tamanho da fila de mensagens para o assinante. Se o assinante não conseguir processar uma mensagem com rapidez suficiente, as mensagens mais antigas serão descartadas quando a fila estiver cheia. Um tamanho de 10 é um padrão razoável para muitos casos de uso.
* Reliability: reliable -> Esta política garante que as mensagens sejam entregues de forma confiável, o que significa que, se uma mensagem for perdida, ela será retransmitida. Isso é importante para comunicações críticas, onde a perda de dados não pode ser tolerada.
* Durability: volatile -> Isso significa que as mensagens não são retidas após o envio. Os dados não são armazenados para novos assinantes que se inscrevem posteriormente. Para dados persistentes que precisam estar disponíveis para assinantes futuros, essa configuração precisa ser alterada para **transient**.
* Liveliness: system default -> Isso determina como o sistema verifica a atividade de um publicador. O "padrão do sistema" significa que o ROS 2 usará o mecanismo definido pelo sistema para determinar se o publicador ainda está ativo.
* Deadline, Lifespan, and Lease Durations: definidos com valores altos (9223372036854775807 nanossegundos) -> Essas configurações são usadas para especificar restrições de tempo. Por exemplo, o Prazo define o período máximo esperado entre mensagens. Quando definido com um valor tão alto, essas restrições se tornam essencialmente não restritivas.

Essas políticas de QoS padrão são aplicadas ao criar publicadores e assinantes, a menos que você especifique explicitamente outros valores.

## Durability
A durabilidade regula se uma mensagem publicada pelo publicador permanece lá para sempre, de modo que os assinantes possam lê-la mesmo após a assinatura da mensagem. É assim, por exemplo, que o robot_description ou os parâmetros globais devem funcionar.

Você tem duas configurações:

* Transient local: O publicador é responsável por fazer com que as mensagens persistam no tempo para os assinantes que se cadastram após a publicação da mensagem pelo publicador.
* Volatile: Você não fará nada para que as mensagens persistam.

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/subscriber_durability.cpp) (**subscriber_durability.cpp**). Aqui, você define **reliability=ReliabilityPolicy.RELIABLE**. Isso significa que você está forçando este assinante a receber cada mensagem enviada pelo publicador.

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/publisher_durability.cpp) (**publisher_durability.cpp**).

Ao executar `ros2 run qos_tests_pkg publisher_durability_exe -durability transient_local` sera mostrado:

```shell
[INFO] [1680120446.045756631] [publisher_qos_obj]: Publishing: 0:1680120446.045713,168012
0446045713462
```

Agora, esse assinante deve receber a mensagem mesmo que ela tenha sido iniciada depois que o publicador a publicou: `ros2 run qos_tests_pkg subscriber_durability_exe -durability transient_local`:

```shell
[INFO] [1680120490.024011041] [subscriber_qos_obj]: Data Received = '0:1680120446.045713,1680120446045713462'
```

Você também pode usar um comando de terminal simples. Defina o comando echo com o QoS apropriado: `ros2 topic echo --qos-durability transient_local /qos_test`

Defina a confiabilidade (**reliability**) além da durabilidade; caso contrário, não funcionará. Este problema no Galactic será resolvido na próxima versão do ROS2: `ros2 topic echo --qos-durability transient_local --qos-reliability reliable /qos_test`

## Deadline
Agora, considere o máximo que deve transcorrer entre a publicação de uma mensagem e a próxima para considerar o tópico saudável. Isso é útil para comandos de controle, como cmd_vel, ou outros comandos mais críticos que precisam ser regulares, como detecção de obstáculos, por exemplo.

A variável regula isso:

* **Duration**: Tempo entre a publicação de mensagens subsequentes.

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/subscriber_deadline.cpp) (**subscriber_deadline.cpp**)

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/publisher_deadline.cpp) (**publisher_deadline.cpp**) 

Execute os scripts em condições de funcionamento, por exemplo. Condição de funcionamento significa que:

* O publicador tem um prazo que excede o tempo que leva para publicar suas mensagens.
* O assinante tem um prazo que excede o tempo do publicador para publicar normalmente E durante a fase de pausa no código.
* Ambos têm valores de QoS compatíveis (consulte a tabela mostrada anteriormente).

```shell
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 10.0
```


```shell
deadline==Duration(nanoseconds=10000000000)
[INFO] [1644936043.662228672] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936043,647624987')"
[INFO] [1644936044.648765369] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936044,648061429')"
[INFO] [1644936044.649647590] [publisher_qos_obj]: Counter =1
[INFO] [1644936045.648750958] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936045,648067137')"
[INFO] [1644936045.649631783] [publisher_qos_obj]: Counter =2
[INFO] [1644936046.648815753] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936046,647949835')"
[INFO] [1644936046.649476575] [publisher_qos_obj]: Counter =3
[INFO] [1644936047.648672771] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936047,647940313')"
[INFO] [1644936047.649307089] [publisher_qos_obj]: Counter =4
[INFO] [1644936048.648772383] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936048,648042781')"
[INFO] [1644936048.649380699] [publisher_qos_obj]: Counter =5
[INFO] [1644936049.648826664] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936049,648102316')"
[INFO] [1644936049.649409283] [publisher_qos_obj]: Counter =6
[INFO] [1644936050.748734377] [publisher_qos_obj]: Paused =0.0/2.0
[INFO] [1644936050.849589211] [publisher_qos_obj]: Paused =0.1/2.0
[INFO] [1644936050.950404186] [publisher_qos_obj]: Paused =0.2/2.0
[INFO] [1644936051.051182319] [publisher_qos_obj]: Paused =0.30000000000000004/2.0
[INFO] [1644936051.152053497] [publisher_qos_obj]: Paused =0.4/2.0
[INFO] [1644936051.252723450] [publisher_qos_obj]: Paused =0.5/2.0
[INFO] [1644936051.353412801] [publisher_qos_obj]: Paused =0.6000000000000001/2.0
[INFO] [1644936051.454263050] [publisher_qos_obj]: Paused =0.7000000000000001/2.0
[INFO] [1644936051.555149310] [publisher_qos_obj]: Paused =0.8/2.0
[INFO] [1644936051.655982300] [publisher_qos_obj]: Paused =0.9/2.0
[INFO] [1644936051.756735299] [publisher_qos_obj]: Paused =1.0/2.0
[INFO] [1644936051.857541979] [publisher_qos_obj]: Paused =1.1/2.0
[INFO] [1644936051.958319656] [publisher_qos_obj]: Paused =1.2000000000000002/2.0
[INFO] [1644936052.059184485] [publisher_qos_obj]: Paused =1.3/2.0
[INFO] [1644936052.159936629] [publisher_qos_obj]: Paused =1.4000000000000001/2.0
[INFO] [1644936052.260631850] [publisher_qos_obj]: Paused =1.5/2.0
[INFO] [1644936052.361722072] [publisher_qos_obj]: Paused =1.6/2.0
[INFO] [1644936052.462475582] [publisher_qos_obj]: Paused =1.7000000000000002/2.0
[INFO] [1644936052.563326299] [publisher_qos_obj]: Paused =1.8/2.0
[INFO] [1644936052.664155447] [publisher_qos_obj]: Paused =1.9000000000000001/2.0
[INFO] [1644936052.665534631] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936052,664860538')"
[INFO] [1644936052.666173215] [publisher_qos_obj]: Counter =1
[INFO] [1644936053.648618315] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936053,647928872')"
[INFO] [1644936053.649236053] [publisher_qos_obj]: Counter =2
```

```shell
ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 10.0
```

```shell
deadline==Duration(nanoseconds=10000000000)
[INFO] [1644936043.662486552] [subscriber_qos_obj]: Data Received =1644936043,647624987
[INFO] [1644936044.649069145] [subscriber_qos_obj]: Data Received =1644936044,648061429
[INFO] [1644936045.649059219] [subscriber_qos_obj]: Data Received =1644936045,648067137
[INFO] [1644936046.649305336] [subscriber_qos_obj]: Data Received =1644936046,647949835
[INFO] [1644936047.649032866] [subscriber_qos_obj]: Data Received =1644936047,647940313
[INFO] [1644936048.648978854] [subscriber_qos_obj]: Data Received =1644936048,648042781
[INFO] [1644936049.649103841] [subscriber_qos_obj]: Data Received =1644936049,648102316
[INFO] [1644936052.665971148] [subscriber_qos_obj]: Data Received =1644936052,664860538
[INFO] [1644936053.648887735] [subscriber_qos_obj]: Data Received =1644936053,647928872
```

Você pode ver que o assinante não apresenta erro, mesmo quando o editor não publica nada por aproximadamente 2 segundos. Isso ocorre porque o assinante tinha 10 segundos de deadline, que está dentro da margem.

E o editor também não apresenta erro porque está publicando algo com um período inferior ao seu deadline de 10 segundos.

### Teste diferentes valores de prazo para observar os diferentes comportamentos:

1. O publicador não cumpre seu prazo: 
    * Aqui, você deve definir um prazo menor que o tempo que o publicador leva para publicar sua mensagem.
2. O assinante não recebe a mensagem do publicador a tempo: 
    * Defina o prazo para o assinante ser menor que o tempo do publicador para publicar cada mensagem.
    * Se o prazo do assinante for pequeno o suficiente, isso acionará o evento de retorno de chamada no loop de pausa ou mesmo no loop de publicação normal.
3. Os assinantes e os publicadores têm valores de QoS incompatíveis.
    * Aqui, você deve consultar a tabela de compatibilidades.

## Lifespan
É definido pela duração, o tempo entre a publicação e o recebimento da mensagem, após o qual a mensagem se torna obsoleta. Isso é importante para dados de sensores, como varreduras a laser ou câmeras, porque, normalmente, você está interessado em ter os valores mais recentes, e dados que chegam depois são inúteis.

**Primeiro**, a duração do tempo de vida só é útil se você a usar com **reliable e transient_local**. O motivo é que, se você não garantir o recebimento da mensagem (**Reliable**) e não esperar que os assinantes atrasados ​​a recebam (**transient_local**), não será possível avaliar a duração da mensagem publicada.

É por isso que você define a QoS no publicador e nos assinantes:

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/subscriber_lifespan.cpp) (**subscriber_lifespan.cpp**)

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/publisher_lifespan.cpp) (**publisher_lifespan.cpp**)

Como funciona o tempo de vida?

Defina uma data de expiração para as mensagens publicadas pelo publicador. Se o assinante receber uma mensagem mais antiga que o tempo de vida definido no publicador, ele nem mesmo processará esses dados. É como se a mensagem não tivesse chegado ao assinante.

No código do assinante, você tem o seguinte:

```cpp
void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
    // The data has the format Seconds,NanoSeconds

    raw_data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Data Received = %s seconds",
                raw_data.c_str());
    delimiter = ",";
    last = 0;
    next = 0;
    while ((next = raw_data.find(delimiter, last)) != std::string::npos) {
      seconds = std::stod(raw_data.substr(last, next - last));
      last = next + 1;
    }
    nanoseconds = std::stod(raw_data.substr(last));
    RCLCPP_INFO(this->get_logger(), "SPLIT = [%lf,%lf]", seconds, nanoseconds);
    RCLCPP_INFO(this->get_logger(), "seconds = %lf, nseconds = %lf", seconds,
                nanoseconds);

    total_seconds = seconds + nanoseconds;
    RCLCPP_INFO(this->get_logger(), "total_seconds = %lf", total_seconds);

    rclcpp::Time time_now_obj = this->now();
    total_current_time = time_now_obj.seconds() + time_now_obj.nanoseconds();
    RCLCPP_INFO(this->get_logger(), "total_current_time = %lf",
                total_current_time);
    delta = total_current_time - total_seconds;
    RCLCPP_INFO(this->get_logger(), "Message Age = %lf seconds", delta);
}
```

Aqui, calcule o tempo decorrido entre o momento em que a mensagem foi publicada e o momento em que o assinante a recebeu.

Lembre-se de que, se esse tempo exceder o tempo de vida útil do publicador, o Retorno de Chamada do Assinante não será acionado. Isso é feito internamente pela estrutura DDS ROS2.

Ao executar: `ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 20.0`:

```shell
[INFO] [1680129694.728612919] [publisher_qos_obj]: Publishing: 1680129694.728510,1680129694728509652
```

Isso deve ser lançado antes do editor ou nos próximos 20 segundos.

```shell
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0
```

```shell
[INFO] [1680131593.313753596] [subscriber_qos_obj]: Data Received = 1680131588.319890,1680131588319890176.000000 seconds
[INFO] [1680131593.313841995] [subscriber_qos_obj]: SPLIT = [1680131588.319890,1680131588319890176.000000]
[INFO] [1680131593.313881163] [subscriber_qos_obj]: seconds = 1680131588.319890, nseconds = 1680131588319890176.000000
[INFO] [1680131593.313921224] [subscriber_qos_obj]: total_seconds = 1680131588.319890
[INFO] [1680131593.313943456] [subscriber_qos_obj]: total_current_time = 1680131593.313942
[INFO] [1680131593.313964152] [subscriber_qos_obj]: Message Age = 4.994052 seconds
```

Observe que a Idade da Mensagem mostrará o número de segundos decorridos entre o início do publicador e o início do assinante.

### Vida útil do Publisher muito restritiva
Para fazer este teste, você deve iniciar o assinante primeiro; caso contrário, você não poderá medir o tempo corretamente.

```shell
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0
```

Inicie uma vida útil rápida, mas factível pelo seu sistema

```shell
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 0.00001
```

Como você pode ver, a mensagem "Age" tem 0,000424 segundos, então, em teoria, NÃO DEVERIA ter funcionado. No entanto, o assinante foi muito mais rápido do que o tempo que o Callback levou para fazer os cálculos e se conectar ao Callback. Ele recebeu a mensagem em 0,00001 segundos, o tempo de vida do publicador.

```shell
[INFO] [1680132077.273447403] [subscriber_qos_obj]: Data Received = 1680132077.273251,1680132077273251072.000000 seconds
[INFO] [1680132077.273569591] [subscriber_qos_obj]: SPLIT = [1680132077.273251,1680132077273251072.000000]
[INFO] [1680132077.273608618] [subscriber_qos_obj]: seconds = 1680132077.273251, nseconds = 1680132077273251072.000000
[INFO] [1680132077.273652540] [subscriber_qos_obj]: total_seconds = 1680132077.273251
[INFO] [1680132077.273676718] [subscriber_qos_obj]: total_current_time = 1680132077.273675
[INFO] [1680132077.273697039] [subscriber_qos_obj]: Message Age = 0.000424 seconds
```

## Liveliness and LeaseDuration
A vivacidade é usada para saber se um publicador ou nó está ativo. Isso significa que ele é publicado a uma taxa regular.

A duração do lease é o período em que você considera que o publicador deve fornecer alguma mensagem ou sinal de que está ativo; caso contrário, considere-o inativo.

[Veja esse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/subscriber_liveliness.cpp) (**subscriber_liveliness.cpp**)

[Veja esse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/Intermediate%20ROS2%20(C%2B%2B)/exemplos/qos_tests_pkg/src/publisher_liveliness.cpp) (**publisher_liveliness.cpp**)

Precisamos comentar vários aspectos do código:

* O fato de você agora estar usando grupos MutuallyExclusiveCallbackGroup

```cpp
self.group_timer_publisher = MutuallyExclusiveCallbackGroup()
self.group_alive_timer = MutuallyExclusiveCallbackGroup()
self.group_events_clb = MutuallyExclusiveCallbackGroup()
```

Os motivos são os seguintes: Como você executa apenas dois Callbacks regulares (timer_callback e alive_callback), se você usasse RentrantCallbackGroups, o sistema executaria timer_callback ou alive_callback várias vezes simultaneamente. Você não quer isso. Você quer que esses Callbacks sejam executados apenas uma instância de cada. É por isso que você usa mutuamente exclusivos.

* O fato de você estar usando agora três Grupos de Retorno de Chamada

```cpp
event_callbacks = PublisherEventCallbacks(incompatible_qos=self.incompatible_qos_clb,liveliness=self.liveliness_clb)
self.publisher_ = self.create_publisher(msg_type=String, topic='/qos_test', qos_profile=qos_profile, event_callbacks=event_callbacks, callback_group=self.group_events_clb)

...

self.create_timer(self.publish_period, self.timer_callback, callback_group=self.group_timer_publisher)

self.create_timer(self.pub_topic_assert_period, self.alive_callback, callback_group=self.group_alive_timer)
```

Os motivos são os seguintes:

Você deseja que pelo menos três Callbacks possam ser executados simultaneamente:
* timer_callback
* alive_callback
* event_callbacks, que são o liveliness_clb ou o incompatible_qos_clb

* O fato de você agora estar usando o executor multithread

```cpp
executor = MultiThreadedExecutor(num_threads=3)
```

Os motivos são os seguintes: Você precisa de pelo menos três threads para processar três Callbacks em paralelo.

### Usando AUTOMATIC LIVELINESS
Você não publica a mensagem Alive; você publica a cada três segundos, com leaseDuration de dois segundos. Em teoria, isso deveria acionar o erro de retorno de chamada de liveliness. No entanto, do lado do assinante e do publicador, nada acontece.

```shell
ros2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 2000 -publish_period 3000 -topic_assert_period 3000 --publish_assert no --policy AUTOMATIC
```

Veja o assinante: Se você definir um LeaseDuration menor que o do publicador, a QoS será incompatível.

```shell
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 1000 --policy AUTOMATIC
```

Entretanto, se você definir a liveliness para um valor MAIOR que o do publicador, o Callback será acionado pelo assinante para nos informar que você tem um publicador ativo:

```shell
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 3000 --policy AUTOMATIC
```

```shell
[ERROR] [1645025787.812470945] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645025787.813001553] [subscriber_liveliness]: alive_count=1
[ERROR] [1645025787.813461276] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645025787.813915481] [subscriber_liveliness]: alive_count_change=1
[ERROR] [1645025787.814364720] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645025787.814814320] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

* alive_count_change=1, significa que você ganhou um publisher.
* alive_count=1, agora você vê que tem um publisher ativo.

Tente interromper o PUBLISHER agora, enquanto o assinante ainda estiver ouvindo. Neste caso, o assinante está contando o tempo definido na Duração do Contrato de Assinante. E então o sistema será acionado, informando que você perdeu um publicador:

```shell
[ERROR] [1645025805.018105382] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645025805.018746115] [subscriber_liveliness]: alive_count=0
[ERROR] [1645025805.019210407] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645025805.019749429] [subscriber_liveliness]: alive_count_change=-1
[ERROR] [1645025805.020284825] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645025805.020858597] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

* alive_count_change=-1, significa que você perdeu um publicador.
* alive_count=0, agora você vê que não tem nenhum publisher ativo.

**A conclusão é que:**

* O valor LeaseDuration SOMENTE é usado ao usar MANUAL_BY_TOPIC.
* Um dos casos em AUTOMATIC, onde o valor LeaseDuration é usado, é para uma verificação de compatibilidade de QoS.
* Outro caso em que você pode acionar o retorno de chamada de atividade é quando você fecha o nó do publicador enquanto o assinante ainda está funcionando.

Observe que o acionamento no assinante é instantâneo quando o publicador morre. Portanto, a duração do lease não é usada.

### Usando o método ALIVE para o MANUAL_BY_TOPIC

**Exemplo de uma mensagem de editor rápido, mas Alive lenta:**

Neste caso, o tempo de publicação do sinal Alive é maior que o LeaseDuration. Isso significa que ele deve gerar um erro. O único momento em que isso não ocorre é quando você publica, pois a publicação tem prioridade sobre o sinal Alive.

Se eu puder falar com você, é óbvio que você está ativo. Não preciso de nenhum sinal especial para me dizer.

Neste exemplo, você vê que ele gera um gatilho de atividade quando o tópico publicado é pausado, porque:

Você parou de publicar; portanto, não recebe nenhuma mensagem.
O sinal Alive publica a cada três segundos, o que é maior que os dois segundos da duração do lease.

```shell
ros2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 2000 -publish_period 1000 -topic_assert_period 3000 --publish_assert yes --policy MANUAL_BY_TOPIC
```

```shell
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 3000 --policy AUTOMATIC
```

```shell
[INFO] [1645026603.247400671] [subscriber_liveliness]: Data Received =1645026603,246501420
[INFO] [1645026604.247414913] [subscriber_liveliness]: Data Received =1645026604,246504289
[ERROR] [1645026607.247663506] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645026607.248263356] [subscriber_liveliness]: alive_count=0
[ERROR] [1645026607.248822345] [subscriber_liveliness]: not_alive_count=1
[ERROR] [1645026607.249401287] [subscriber_liveliness]: alive_count_change=-1
[ERROR] [1645026607.250044846] [subscriber_liveliness]: not_alive_count_change=1
[ERROR] [1645026607.250583916] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
[ERROR] [1645026610.285465126] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645026610.286062735] [subscriber_liveliness]: alive_count=1
[ERROR] [1645026610.286643833] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645026610.287334065] [subscriber_liveliness]: alive_count_change=1
[ERROR] [1645026610.287854927] [subscriber_liveliness]: not_alive_count_change=-1
[ERROR] [1645026610.288318913] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
[INFO] [1645026610.288989710] [subscriber_liveliness]: Data Received =1645026610,284628135
[INFO] [1645026611.247577412] [subscriber_liveliness]: Data Received =1645026611,246611642
```

