# Pacotes
No ROS2, você pode criar dois tipos de pacotes: pacotes CMake (C++) e pacotes Python. Neste curso, porém, você se concentrará nos primeiros.

Cada pacote CMake terá a seguinte estrutura de arquivos e pastas:
* Pasta launch: Contém os arquivos de inicialização
* Pasta src: Contém os arquivos de código-fonte (CPP, Python)
* CMakeLists.txt: Lista de regras do CMake para compilação
* pacote.xml: Metadados e dependências do pacote

## Criando um pacote
Para criar pacotes, você precisa trabalhar em um espaço de trabalho específico conhecido como espaço de trabalho ROS2. O espaço de trabalho ROS2 é o diretório no seu disco rígido onde os pacotes ROS2 residem para serem usados pelo ROS2. Normalmente, o diretório do espaço de trabalho ROS2 é chamado **ros2_ws**.

Dentro deste espaço de trabalho, há um diretório chamado **src**. Esta pasta contém todos os pacotes do espaço de trabalho. Portanto, sempre que você quiser criar um pacote, deverá estar neste diretório src de um espaço de trabalho.

Neste ponto, você finalmente está pronto para criar seu próprio pacote! Para isso, digite o seguinte no seu terminal:
* `cd ~/ros2_ws/src`
* `ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp`

**my_package** é o nome do pacote que você deseja criar, e **rclcpp** são os nomes de outros pacotes ROS2 dos quais seu pacote depende.

Observe também que você está especificando **ament_cmake** como o tipo de compilação. Isso indica que você está **criando um pacote CMake**.

É uma boa ideia **compilar** seu pacote após sua criação. É a maneira mais rápida de determinar se as dependências listadas podem ser resolvidas e verificar se não há erros nos dados inseridos:
* `cd ~/ros2_ws/`
* `colcon build`
* `source install/setup.bash`

Para confirmar que seu pacote foi criado com sucesso, use alguns comandos ROS relacionados a pacotes:
* `ros2 pkg list`: Fornece uma lista com todos os pacotes no seu sistema ROS2.
* `ros2 pkg list | grep my_package`: Filtra, de todos os pacotes localizados no sistema ROS2, o pacote é chamado my_package.

## Compilando pacotes
Ao criar um pacote, você precisa compilá-lo para que ele funcione.

O comando a seguir compilará **todo o seu diretório src** e precisa ser executado dentro do diretório home de um espaço de trabalho para funcionar (**ros2_ws**):
* `cd ~/ros2_ws/`
* `colcon build`
* `source install/setup.bash`

Às vezes (para projetos grandes), você não vai querer compilar todos os seus pacotes. Isso levaria muito tempo. Então, você pode usar o seguinte comando para compilar apenas os pacotes nos quais você fez alterações:
* `colcon build --packages-select <package_name>`

Compile sempre que alterar qualquer arquivo, mesmo arquivos Python ou de inicialização que não precisem de compilação. 

# Arquivos Launch (arquivo de inicialização)
Para usar um arquivo de inicialização, a estrutura do comando seria a seguinte: `ros2 launch <package_name> <launch_file>`

Veja um exemplo. Se você quiser usar um arquivo de inicialização para iniciar o executável **teleop_keyboard**, precisará escrever algo semelhante ao script Python abaixo:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True),
    ])
```

Como você pode ver, a estrutura do arquivo de inicialização é bem simples. Primeiro, você importa alguns módulos dos pacotes **launch** e **launch_ros**.:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

Em seguida, defina uma função que retornará um objeto **LaunchDescription**:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True),
    ])
```

Dentro do objeto **LaunchDescription**, gere um **nó** onde você fornecerá os seguintes parâmetros:

1. **package**='nome_do_pacote' é o nome do pacote que contém o código do programa ROS a ser executado
2. **executable**='nome_executável_binário' é o nome do arquivo executável binário que você deseja executar
3. **output**='tipo_de_saída' é o canal onde você imprimirá a saída do programa
4. **emulate_tty**=True|False permite que arquivos de inicialização produzam mensagens de log coloridas: Verde=DEBUG, Branco=INFO, Laranja=Aviso e Vermelho=ERRO|Fatal.
