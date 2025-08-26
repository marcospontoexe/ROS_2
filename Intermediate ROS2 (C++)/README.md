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


## Arquivos de launch usando YAML
[Veja uma launch **yaml_main.launch.yaml**]().

Veja a [**move_with_arguments.launch.yaml**]() que executa **move_robot_with_params.cpp**






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
