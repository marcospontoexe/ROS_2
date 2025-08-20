# Launch

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

[No node]() observe que passamos os argumentos para o construtor da classe:

```cpp
DummyArgumetsExample(int &argc, char **argv)
```

Obtemos os argumentos dentro do construtor da classe:

