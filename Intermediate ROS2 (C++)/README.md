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

Este é o [primeiro exemplo]() do problema que você terá se usar SINGLE THREAD EXECUTOR.

Se você usar **STATIC SINGLE THREADED** EXECUTOR, a única alteração será substituir a instanciação do Executor único por esta:
```cpp
rclcpp::executors::StaticSingleThreadedExecutor executor;
```

