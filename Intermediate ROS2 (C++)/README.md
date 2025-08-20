# LogInfo
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

# Executando várias launch a partir de uma unica launch
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