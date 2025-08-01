# Pacotes
O ROS 2 utiliza pacotes para organizar seus programas. Você pode pensar em um pacote como uma coleção de todos os arquivos relacionados a um programa ROS 2 específico; todos os seus arquivos CPP, arquivos Python, arquivos de configuração, arquivos de compilação, arquivos de inicialização e arquivos de parâmetros. Além disso, organizar seus programas ROS 2 em pacotes facilita muito o compartilhamento com outros desenvolvedores/usuários.

No ROS2, você pode criar dois tipos de pacotes:

* Pacotes Python
* Pacotes CMake (C++)

Neste curso, vamos nos concentrar no primeiro tipo. Os pacotes Python conterão executáveis Python.

Cada pacote Python terá a seguinte estrutura de arquivos e pastas:

* **package.xml** - Arquivo contendo meta-informações sobre o pacote (mantenedor do pacote, dependências, etc.).
* **setup.py** - Arquivo contendo instruções sobre como compilar o pacote.
* **setup.cfg** - Contém instruções sobre como instalar o pacote.
* src/\<nome_do_pacote> - Este diretório recebe o nome do seu pacote. Você colocará todos os seus scripts Python dentro desta pasta. Ela já contém um arquivo __init__.py vazio por padrão.

![2_3_ros2_package_no_title](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(Python)/imagens/2_3_ros2_package_no_title.svg)

## Criando um pacote
Antes de criar um pacote, vamos revisar alguns pontos-chave relacionados aos pacotes ROS 2:

* Ao criar pacotes, você precisa trabalhar em um espaço de trabalho específico chamado espaço de trabalho ROS 2. Este espaço de trabalho é um diretório no seu disco rígido onde seus pacotes ROS 2 são armazenados e disponibilizados para o ROS 2. Normalmente, o diretório de um espaço de trabalho ROS 2 é chamado **ros2_ws**.
* Para diferentes projetos ROS 2, você pode optar por criar espaços de trabalho separados para manter cada projeto isolado dos outros.

Crie o ambiente ROS2 no seu Terminal para usar as ferramentas de linha de comando do ROS2. Forneça (Sourcing) as configurações do ambiente ROS2 para que você possa usar suas ferramentas de linha de comando nesse terminal: `source /opt/ros/humble/setup.bash`

Agora, vá para o ros2_ws no seu Terminal: `cd ~/ros2_ws/`

Dentro deste espaço de trabalho, há um diretório chamado **src**. Esta pasta contém todos os pacotes criados. Sempre que quiser criar um pacote, você precisa estar neste diretório **ros2_ws/src**. 

1. Digite o seguinte comando no seu Terminal para navegar até o diretório **scr**: `cd ~/ros2_ws/src`.
2. Degite o comando para criar um pacote: `ros2 pkg create --build-type ament_python <package_name> --dependencies package_dependency_1 package_dependency_2`, por exemplo `ros2 pkg create --build-type ament_python mars_rover_systems --dependencies rclpy`

**package_name** é o nome do pacote que você deseja criar, e **package_dependency_X** são os nomes de outros pacotes ROS2 dos quais o seu pacote depende. Observe também que estamos especificando **ament_python** como o tipo de compilação. Isso indica que estamos criando um pacote Python.

3. **Compilar** seu pacote após criá-lo é uma boa prática. Isso permite verificar rapidamente se as dependências listadas podem ser resolvidas e ajuda a identificar quaisquer erros nas informações fornecidas:

```shell
cd ~/ros2_ws/
colcon build
source install/setup.bash
```

ou

```shell
colcon build --packages-select package_name
```

4. Para **confirmar** que seu pacote foi criado com sucesso, use os comandos ROS relacionados a pacotes. Por exemplo, digite o seguinte: `ros2 pkg list`.

Assim como em outras tarefas de codificação, você precisa **compilar seu código para gerar executáveis**. Abordaremos como fazer isso na próxima seção.


[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(Python)/exemplos/mars_rover_systems) um programa básico chamado **heartbeat.py**, que exibirá periodicamente uma mensagem de status. Colocaremos heartbeat.py dentro do pacote **mars_rover_systems** para que ele possa ser executado como parte das verificações de sistema do Mars Rover.

### Criando um arquivo Python
Para este exemplo Vamos criar um pacote chamado **mars_rover_systems**.

1. Crie um arquivo Python no diretório **mars_rover_systems**, que está dentro do pacote **mars_rover_systems**. Para este exercício, criaremos um arquivo Python simples chamado **heartbeat.py**.

**heartbeat.py:**

```python
#!/usr/bin/env python
import rclpy

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Mars rover 1 is alive...")
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function
```

para testar o executável: 
```shell
cd ~/ros2_ws/src/mars_rover_systems/mars_rover_systems/
python3 heartbeat.py 
```

### Modificando o arquivo setup.py para executar programas Python
O arquivo **setup.py** em módulos Python do ROS2, define o pacote, manipula dependências e gerencia a distribuição. Isso é comum em muitos projetos baseados em Python para essa finalidade.

Nos pacotes Python do ROS2, além do mencionado acima, o arquivo setup.py integra-se ao ecossistema ROS2, garantindo a compatibilidade com as ferramentas de compilação, o gerenciamento de dependências e a execução de nós do ROS2.

Dê uma olhada no arquivo setup.py, especificamente em console_scripts. Ele está localizado dentro de um dicionário chamado entry_points.

#### **Custom Entry Points**
Setup.py define o que é conhecido como pontos de entrada (Entry Points). Esses pontos de entrada permitem que o ROS2 reconheça e execute scripts baseados em Python.

Os pontos de entrada têm duas partes:
* **entry_point_name**: Este é um nome arbitrário que você fornece, que será usado no ROS2.
* **entry_point_script_path**: Especifica o caminho para o MÉTODO que você deseja executar ao chamar o entry point name.

```python
'console_scripts': [
      'entry_point_name = entry_point_script_path',
      'another_entry_point_name = another_entry_point_script_path',
      ...
      'still_another_entry_point_name = still_another_entry_point_script_path'
  ],
```

Como você pode ver, você pode incluir quantos quiser. Vamos dar uma olhada no que precisamos incluir para executar o método **main()** em nosso **heartbeat.py**.

```python
'console_scripts': [
            'heartbeat_executable = mars_rover_systems.heartbeat:main'
        ],
```

entry_point_script_path: mars_rover_systems.heartbeat:main:

1. Isso informa ao ROS2 que queremos acessar uma pasta dentro do pacote chamado **mars_rover_systems**. Por padrão, quando criamos um pacote, uma pasta com EXATAMENTE o MESMO NOME do pacote é gerada dentro dele.
2) Em seguida, especifica qual script dentro da pasta mars_rover_systems devemos procurar, neste caso, **heartbeat.py**.
3) Dentro de heartbeat.py (observe o uso de : em vez de .), especificamos o método a ser executado, neste caso, o método chamado **main()**.

Seu arquivo **setup.py** final deve ficar assim:

```python
from setuptools import find_packages, setup

package_name = 'mars_rover_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_executable = mars_rover_systems.heartbeat:main'
        ],
    },
)
```

Isso nos permitirá executar o script usando o comando ros2 run, sem precisar saber o caminho exato onde o script está localizado:
```python
ros2 run mars_rover_systems heartbeat_executable
```

### Criando um novo método e modificando o setup.py para executa-lo
Adicione um novo método a heartbeat.py semelhante a main(), chamado **main_shutdown()**. Este método deve exibir uma mensagem dizendo Shutting down Mars rover 1...

**heartbeat.py:**

```python
#!/usr/bin/env python
import rclpy

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Mars rover 1 is alive...")
    # shutdown the ROS communication
    rclpy.shutdown()

def main_shutdown(args=None):
    rclpy.init(args=args)
    print("Shutting down Mars rover 1...")
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function
```

**setup.py:**

```python
from setuptools import find_packages, setup

package_name = 'mars_rover_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_executable = mars_rover_systems.heartbeat:main',
            'heartbeat_executable_shutdown = mars_rover_systems.heartbeat:main_shutdown'
        ],
    },
)
```

compile:
```shell
cd ~/ros2_wsv
colcon build
source ~/ros2_ws/install/setup.bash
```

execute: `ros2 run mars_rover_systems heartbeat_executable_shutdown`

### Crie um novo método e entry point para que possamos iniciar ao mesmo tempo dois nós diferentes
Crie um novo método dentro de heartbeat.py semelhante a main(), chamado main2(). A única diferença deve ser o nome do robô.

Em seguida, adicione um novo ponto de entrada para permitir a execução de main2().

**heartbeat.py:**

```python
#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self, rover_name, timer_period=0.2):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        self._rover_name = rover_name
        super().__init__(self._rover_name)
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        self.get_logger().info(self._rover_name +" is alive..."+str(ros_time_stamp))

    def main_shutdown(args=None):
      rclpy.init(args=args)
      print("Shutting down Mars rover 1...")
      rclpy.shutdown()

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="mars_rover_1", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

def main2(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="mars_rover_2", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**setup.py:**

```python
from setuptools import find_packages, setup

package_name = 'mars_rover_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [            
            'heartbeat_executable = mars_rover_systems.heartbeat:main',
            'heartbeat_executable2 = mars_rover_systems.heartbeat:main2',
            'heartbeat_executable_shutdown = mars_rover_systems.heartbeat:main_shutdown'
        ],
    },
)
```

Compile:
```shell
cd ~/ros2_wsv
colcon build
source ~/ros2_ws/install/setup.bash
```

Agora você pode e executar main e main2 simultaneamente, um em cada terminal:
```shell
ros2 run mars_rover_systems heartbeat_executable
```

```shell
ros2 run mars_rover_systems heartbeat_executabl2
```


### Criando um arquivo launch
Nesta exemplo, você aprenderá a usar arquivos launch para iniciar várias instâncias do programa **Heartbeat** do Rover com um único comando. Isso ajudará você a entender como gerenciar com eficiência vários nós ROS 2 em um sistema.

1. Crie uma pasta dentro do pacote **mars_rover_systems** chamada **launch**.
2. Dentro da pasta de launch, crie um novo arquivo chamado **two_rover_heartbeat.launch.py**