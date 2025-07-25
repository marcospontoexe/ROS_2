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
* `ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp std_msgs`

**my_package** é o nome do pacote que você deseja criar. **rclcpp** e **std_msgs** são os nomes de outros pacotes ROS2 dos quais seu pacote depende.

Observe também que você está especificando **ament_cmake** como o tipo de compilação. Isso indica que você está **criando um pacote CMake**.

É uma boa ideia **compilar** seu pacote após sua criação. É a maneira mais rápida de determinar se as dependências listadas podem ser resolvidas e verificar se não há erros nos dados inseridos:
* `cd ~/ros2_ws/`
* `colcon build`
* `source install/setup.bash` ou `source /opt/ros/humble/setup.bash`.

Para confirmar que seu pacote foi criado com sucesso, use alguns comandos ROS relacionados a pacotes:
* `ros2 pkg list`: Fornece uma lista com todos os pacotes no seu sistema ROS2.
* `ros2 pkg list | grep my_package`: Filtra, de todos os pacotes localizados no sistema ROS2, o pacote é chamado my_package.

[Veja nesse diretório](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos) o pacote **marcos** criado como exemplo.

## Arquivos Launch (arquivo de inicialização)
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
2. **executable**='nome_executável_binário' é o nome do arquivo executável binário que você deseja executar. Esse arquivo é gerado a partir da compilação do código .cpp através do compilador *ament*. O nome do arquivo binário é definido no arquivo **CMakeLists.txt**.
3. **output**='tipo_de_saída' é o canal onde você imprimirá a saída do programa
4. **emulate_tty**=True|False permite que arquivos de inicialização produzam mensagens de log coloridas: Verde=DEBUG, Branco=INFO, Laranja=Aviso e Vermelho=ERRO|Fatal.

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos/launch/my_package_launch_file.launch.py).

## Criando um arquivo de programa
Os arquivos de programa (.cpp e .h) dever ser criados dentro do diretório **src**.

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos/src/simple.cpp).

## CMakeLists.txt
Ao programar em C++, você precisa **criar binários (executáveis)** dos seus programas para executá-los. Para isso, você precisará modificar o arquivo CMakeLists.txt do seu pacote para indicar que deseja criar um executável do seu arquivo C++ (.cpp).

Para isso, você precisa adicionar algumas linhas ao seu arquivo CMakeLists.txt.

No exempplo a baixo:

```txt
# para gerar o executável simple_publisher_node
add_executable(simple_publisher_node src/simple_topic_publisher.cpp)    # Quando o "simple_topic_publisher.cpp" é compilado, é gerado um executável (simple_publisher_node) a partir do arquivo simple_topic_publisher.cpp, que está na pasta src do seu pacote. 
add_executable(simple_publisher_composable_node src/simple_topic_publisher_composable.cpp)


# para incluir dependências 
ament_target_dependencies(simple_publisher_node rclcpp std_msgs)    # Esta linha adiciona todas as dependências ament de destino do executável.
ament_target_dependencies(simple_publisher_composable_node rclcpp std_msgs geometry_msgs)


#Este snippet instalará nosso nó executável (simple_node) em nosso espaço de instalação dentro do espaço de trabalho do ROS2. 
#Portanto, este executável será colocado no diretório de pacotes do seu espaço de instalação, que está localizado, por padrão, em
# ~/ros2_ws/install/my_package/lib/my_package/.
install(TARGETS
   simple_publisher_node
   simple_publisher_composable_node
   DESTINATION lib/${PROJECT_NAME}
 )

# O objetivo deste código é instalar os arquivos de inicialização. 
# Por exemplo, com o pacote chamado my_package, isso instalará todos os arquivos de inicialização da pasta launch/ em
# ~/ros2_ws/install/my_package/share/my_package/launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

Além de adicionar as linhas relacionadas aos executáveis e launch, é necessário incluir pacotes ao usar mensagens (interfaces) que não foram definidos como dependências no momento da criação do pacote. Esses pacotes são incluidos em **find_package()**:

```txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
```

[Veja um exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_publisher/CMakeLists.txt).

## Compilando pacotes
Ao criar um pacote, você precisa compilá-lo para que ele funcione.

O comando a seguir compilará **todo o seu diretório src** e precisa ser executado dentro do diretório home de um espaço de trabalho para funcionar (**ros2_ws**):
* `cd ~/ros2_ws/`
* `colcon build`
* `source install/setup.bash` ou `source /opt/ros/humble/setup.bash`.

Às vezes (para projetos grandes), você não vai querer compilar todos os seus pacotes. Isso levaria muito tempo. Então, você pode usar o seguinte comando para compilar apenas os pacotes nos quais você fez alterações:
* `colcon build --packages-select <package_name>`

Compile sempre que alterar qualquer arquivo, mesmo arquivos Python ou de inicialização que não precisem de compilação. 

# Node (nós)
No ROS2, um único executável (um programa C++ ou Python, etc.) pode conter um ou mais nós.

* Para observar nós em execução no espaço de trabalho, use o comando ROS: `ros2 node list`.
* Para ver informações sobre seu nó, você pode usar o seguinte comando: `ros2 node info node_name`.

Para encerrar seu nó, você pode pressionar **Ctrl + C** no terminal em que ele está sendo executado.

## Node Composition (composição do nós)
No ROS2, como uma diferença notável em relação ao ROS1, o conceito de Composição é introduzido. Isso significa que você pode compor (executar) **vários nós em um único processo**. Você aprenderá mais sobre composição de nós nas próximas unidades.

Para usar a composição de nós, você deve programar seus scripts de uma forma mais orientada a objetos. 

[Veja um exemplo de uso de composição do nós](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_publisher/src/simple_topic_publisher_composable.cpp).

### Vantagens de usar múltiplos nós no mesmo executável (mesmo processo)

1. Comunicação mais eficiente (Intra‑Process Communication): Nós executados no mesmo processo podem compartilhar dados diretamente pela memória, sem a necessidade de serialização ou transporte via DDS. Isso reduz **latência, overhead de CPU e uso de memória**, especialmente útil para mensagens grandes ou alta frequência.

2. Menor consumo de recursos: Executar múltiplos nós dentro de um único processo reduz o overhead associado a processos diferentes (contêineres de threads, alocação de memória, etc.). Ideal para sistemas embarcados ou robôs com recursos limitados.

# Tópics
Os **tópicos** formam a espinha dorsal de como os nós se comunicam em um sistema baseado em ROS.

Um Tópico é como um tubo. Os nós usam Tópicos para publicar informações para outros nós, permitindo a comunicação entre eles.
* A qualquer momento, você pode verificar o número de Tópicos no sistema por meio de uma lista de tópicos ros2: `ros2 topic list`.
* Você também pode verificar informações de um tópico específico: `ros2 topic info name_of_topic` ou `ros2 topic info -- verbose name_of_topic` para ver informações sobre QoS.
* Para ler as informações que estão sendo publicadas sobre um tópico: `ros2 topic echo name_of_topic`.
* Para publicar mensagens (interface) em um tópico: `ros2 topic pub topic_name message_type value`. Por exemplo: `ros2 topic pub /counter std_msgs/Int32 "{data: '5'}"`.
* verificar as diferentes opções que o comando **rostopic** possui usando o seguinte comando: `ros2 topic -h` ou pressione duas vezes rapidamente a tecla **TAB** após o digitar **ros2 topic**.

## Publisher
Um Publicador é um nó que fica publicando uma mensagem em um Tópico

[Veja no pacote **marcos_publisher**](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_publisher). O nó **simple_publisher** publica um int32 incrementado a cada 2 Hz no tópico **/counter**.

Para verificar a saida do tópico /counter, use: `ros2 topic echo /counter`.

## Subscribers (Assinante)
Um Assinante é um nó que lê informações de um Tópico.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_subscriber/src/simple_topic_subscriber.cpp). Um nó Assinante que escuta o tópico **/counter** e, cada vez que lê algo, chama uma função que imprime a mensagem. Para que algo seja mostrado no terminal é necessário publicar no tópcio /counter.

# Messages (interfaces)
Tópicos manipulam informações por meio de mensagens (interfaces). Existem diferentes tipos de mensagens. No ROS1, você as conhece como mensagens. No entanto, no ROS2, essas mensagens são conhecidas como **interfaces**.

No caso do código de exemplo usado na seção publisehr o tipo de interface era std_msgs/Int32, mas o ROS2 fornece interfaces diferentes. Você pode até criar suas próprias interfaces, mas é recomendável usar as interfaces padrão do ROS2 sempre que possível.

As interfaces para tópicos são definidas em arquivos .msg, que estão localizados dentro de um diretório **msg** de um pacote.

Para obter informações sobre uma interface, use o seguinte comando: `ros2 interface show package_name/msg/message_type` ou `ros2 interface proto package_name/msg/message_type`, por exemplo `ros2 interface show std_msgs/msg/Int32`.

## Criando uma interface (mensagem) customizada
Você pode estar se perguntando se precisa publicar alguns dados que não sejam Int32 e que tipo de mensagem deve usar. É claro que você pode usar todas as mensagens definidas pelo ROS2 (**ros2 interface list**). No entanto, se nenhuma atender às suas necessidades, você pode criar uma nova.

Para criar uma nova mensagem, siga os seguintes passos:

1. Crie um diretório chamado '**msg**' dentro do seu pacote
2. Dentro deste diretório **msg**, crie um arquivo chamado **Nome_da_sua_mensagem.msg** (mais informações abaixo)
3. Modifique o arquivo **CMakeLists.txt** (mais informações abaixo)
4. Modifique o arquivo **package.xml** (mais informações abaixo)
5. **Compile** e crie o código-fonte
6. Use no código

[Veja o pacote marcos_custom_interfaces](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C++)/exemplos/marcos_custom_interfaces) uma interface (Age.msg) que indica a idade, com anos, meses e dias, criada dentro do diretório **msg**, e os arquivos **CMakeLists.txt** e  **package.xml** alterados de acordo com a nova interface (Age.msg) criada.

### Criando o arquivo de interface
O arquivo de interface deve ser criado dentro do diretório **msg** do pacote em questão.

No exemplo a baixo a nova interface se chamara **Age**:

```txt
int32 years
int32 months
int32 days
```

### Modificabdo o arquivo CMakeLists.txt
Edite duas funções dentro de CMakeLists.txt:
* find_package()
* rosidl_generate_interfaces()

#### find_package()
É aqui que vão todos os pacotes necessários para COMPILAR as mensagens dos Tópicos, Serviços e Ações. Em **package.xml**, defina-os como **build_depend** e **exec_depend**.

```txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```

#### rosidl_generate_interfaces()
Esta função inclui todas as mensagens do pacote (na pasta msg) a serem compiladas. 

No exemplo a baixo o arquivo de mensegem se chama **Age**. A função deve ter a seguinte aparência:

```txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)
```

### Modificabdo o arquivo package.xml
Adicione as seguintes linhas ao arquivo package.xml.

```txt
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

Após compilar o pacote **marcos_custom_interfaces** use o comando para verificar se a interface foi criada: `ros2 interface show marcos_custom_interfaces/msg/Age`.

## Usando uma interface (mensagem) customizada
Para usar a interface criada (Age.msg do pacote marcos_custom_interfaces) em um novo pacote, adicione ao **CMakeLists.txt** do novo pacote criado as seguintes linhas extras para compilar e vincular seu executável (neste exemplo, ele é chamado publish_age.cpp):

```txt
#-----Usando a interface em um nó-------------------
find_package(marcos_custom_interfaces REQUIRED) # This is the package that contains the custom interface
#---------------------------------------------------

#------para gerar o executável--------------------
add_executable(publish_age_node src/publish_age.cpp)
#------adiciona as dependencias ao nó--------------
ament_target_dependencies(publish_age_node rclcpp std_msgs marcos_custom_interfaces) # Note that we are also adding the package which contains the custom interface as a dependency of the node that will be using this custom interface
#---------------------------------------------------

#------ Arquivos de inicialização do nó---------
install(TARGETS
   publish_age_node
   DESTINATION lib/${PROJECT_NAME}
 )
#-----------------------------------------------

```

Você também precisará adicionar ao novo pacote criado como uma dependência no seu arquivo **package.xml**:

```txt
  <!-- para usar a mensagem criada em um nó-->
  <depend>marcos_custom_interfaces</depend>
  <!-- ################################# -->
```

**Importante**: não esqueçe da importar o pacote e interface no axecutável do novo pacote:

```c++
#include "marcos_custom_interfaces/msg/age.hpp"
```

[Veja o pacote marcos_using_custom_interfaces](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_using_custom_interfaces) um nó publicador que indica a idade do robô. Para isso é usada a interface criada **Age.msg** do pacote **marcos_custom_interfaces**. Para executar o publisher use `ros2 run marcos_using_custom_interfaces publish_age_node` e também, `ros2 topic echo /age` em outro terminal.

# Executors and Callback Groups
Aplicações robóticas modernas frequentemente exigem que múltiplas tarefas sejam executadas simultaneamente, como processamento de dados de sensores, execução de planejamento de movimento e comunicação com outros sistemas. É aqui que o multithreading se torna essencial. O **multithreading** permite que diferentes partes de um programa sejam executadas simultaneamente, melhorando o desempenho e a responsividade.

No ROS 2, **executors** e **Callback Groups** são mecanismos essenciais para gerenciar a execução multithread. Ao aproveitar o multithreading, os executors e os callback Groups, você aumentará a eficiência e a responsividade de suas aplicações ROS 2, garantindo que elas possam lidar com tarefas complexas em tempo real com eficácia.

## Executor
O objetivo de um Executor é coordenar diferentes nós e retornos de chamada executando-os em vários threads para execução em paralelo, de modo que os retornos de chamada (Callback) não bloqueiem a execução da parte restante do programa, o que pode melhorar significativamente o fluxo de execução e o desempenho.

### Tipos de Executors
O **rclcpp** fornece três tipos de Executor, derivados de uma classe pai compartilhada:

1. O Executor **Single-threaded** usa uma thread para executar todas as instruções do Node. 
2. O Executor **Multi-threaded** cria um número variável de threads que permite que múltiplas mensagens/eventos sejam processados em paralelo.
3. O Executor **Static Single-threaded** executa uma varredura de Node apenas uma vez, quando o Node é adicionado ao Executor. Portanto, use-o apenas com Nodes que criam todos os Callbacks relacionados durante a inicialização.

Nas duas primeiras implementações, o Executor se adapta a alterações como adicionar, remover ou alterar assinaturas, temporizadores, servidores de serviço e servidores de ação durante o tempo de execução. Use a última opção da lista acima se você não precisar desse recurso e não se importar com processos single-threaded configurados no momento da inicialização.

### Executor executando nó mínimo
[Esse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_1.cpp) demonstra como adicionar um Nó a um Executor. São necessárias três instruções:

1. Inicializar um objeto Executor.
2. Adicionar um Nó ao Executor.
3. Executar o Executor com o Spin() para procurar trabalho disponível e concluí-lo.

### Executor executando nó mínimo com callback
[Neste exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_2.cpp), adicionaremos uma função de retorno de chamada (callback) que assina o tópico /box_bot_1/odom.

O resultado da execução é mostrado a baixo:

![executor_example_2_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_2_node.png)

### Single-Threaded Executor executando um nó com duas funções de Callback
O objetivo principal deste exemplo é demonstrar o problema que pode surgir ao usar um EXECUTOR SINGLE-THREADED.

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_3.cpp) é definido dois nós ROS2:
1. OdomSubscriber
2. SlowTimer

Ambos os nós são instanciados e adicionados a um **SingleThreadedExecutor** dentro de main().

O resultado da execução é mostrado a baixo:

![executor_example_3_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_3_node.png)

Em comparação com o exemplo anterior, a odometria não está sendo registrada periodicamente, em intervalos regulares. Então, por que isso acontece?

O problema é que a função de Callback odometry não é executada mesmo quando o tópico /box_bot_1/odom possui valores publicados. Sua execução é bloqueada pela função de Callback do timer e ainda não finalizou seu trabalho.

Isso ocorre porque você tem apenas UMA THREAD no Executor, como é o caso dos tipos Executor Single-Threaded e Executor Static Single-Threaded.

Portanto, a solução lógica é usar o Executor Multi-Threaded.

### Multi-Threaded Executor executando dois nós
[Neste exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_4.cpp), mantenha tudo igual ao exemplo anterior, exceto que o Executor Multi-Threaded será usado.

Você pode ver uma mensagem "TICK" a cada 100 mili segundos, e a odometria flui em uma taxa contínua. Isso significa que ambos os Callbacks estão trabalhando simultaneamente em paralelo e sem interferir um no outro.

O resultado da execução é mostrado a baixo:

![executor_example_4_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_4_node.png)

**Observação**: neste exemplo, há um Callback por nó, então ter uma thread por nó funciona perfeitamente.

### Dois nós em execução em dois executors diferentes
[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_4_singlethreaded.cpp), é explorado o que acontece se você usar duas instâncias Single-ThreadedExecutor no mesmo programa ROS 2. Você está criando dois Static Single-Threaded Executors e, em seguida, atribuindo a um deles apenas UM Nó.

A função de Callback de odometria funciona bem, mas onde está a mensagem de log gerada pelo Callback do temporizador? Como você pode ver, o Callback do temporizador não está sendo executado e não há mensagem de erro.

O resultado da execução é mostrado a baixo:

![executor_example_4_singlethreaded_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_4_singlethreaded_node.png)

Isso ocorre porque você só pode ter **UM ÚNICO EXECUTOR** por arquivo binário.

Por isso, se você preferir continuar usando SingleThreadedExecutors, sua única opção é criar dois programas ROS2 diferentes, compilados como dois arquivos binários independentes.

**Observação**: Em termos de desempenho, SingleThreadedExecutors consomem menos CPU do que MultiThreadedExecutors. Portanto, se atender a todas as suas necessidades, é preferível.

Agora que você já entendeu os conceitos básicos do Executor, está pronto para abordar os aspectos mais avançados para lidar adequadamente com múltiplos Callbacks (Callback Groups).

## Callback Groups
[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_5.cpp) é declarada uma classe Node com mais de um Callback. Neste caso, dois Callbacks. Dentro da função principal, você verá que ela carrega o Node em um MultiThreadedExecutor.

O resultado da execução é mostrado a baixo:

![executor_example_5_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_5_node.png)

Ao executar o nó **executor_example_5_node**, somente o retorno de chamada do timer 1 é executado. É por isso que apenas uma thread é criada e apenas um Callback pode ser executado. Isso ocorre porque o Executor Multi-Threaded cria uma thread por Nó, a menos que você especifique o contrário. Portanto, você tem apenas uma thread para todos os Callbacks neste Nó, portanto, ele não pode executar Callbacks em paralelo.

O timer1_callback é executado porque foi instanciado primeiro no construtor.

Então, como resolver esse problema? A solução é implementar **CALLBACK GROUPS**!

### Tipos de Callback Groups
Grupos de Callbacks (Callback Groups) permitem agrupar diferentes Callbacks para que alguns sejam executados em paralelo e outros não. Existem dois tipos:

1. **Reentrant**: Todos os Callbacks nesses grupos podem ser executados simultaneamente, sem restrições. O Executor gerará quantas threads forem necessárias para isso.
2. **MutuallyExclusive**: Todos os Callbacks dentro desses grupos podem ser executados um de cada vez. Isso é útil para Callbacks que, por algum motivo, afetam o mesmo sistema ou usam os mesmos recursos que outros. Além disso, esse tipo de Callback oferece melhor controle sobre o fluxo de Callbacks.

### Reentrant Callback Groups
ReentrantCallbackGroup permite que o Executor agende e execute os Callbacks do grupo da maneira que achar melhor, sem restrições. Isso significa que, além de diferentes Callbacks serem executados simultaneamente, o Executor também pode executar diferentes instâncias do mesmo Callback simultaneamente. Assim, por exemplo, a execução de um Callback de timer pode levar mais tempo do que o período de disparo do timer (embora esse caso específico deva ser evitado a todo custo no rclpy, isso é outra história).

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_5_reentrant.cpp) a única mudança é que foi criado um grupo de retorno de chamada (Callback Groups) com a seguinte linha de código:

```c++
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
```

Em seguida, é passado **callback_group_** como um argumento adicional para **create_wall_timer()**. Nesse caso, ambos usam o mesmo grupo de retorno de chamada do tipo Reentrant:

```c++
  timer1_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer1_callback, this), callback_group_);
  timer2_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer2_callback, this), callback_group_);
```

Da mesma forma, é passado um grupo de retorno de chamada para assinantes, serviços ou ações.

O resultado da execução é mostrado a baixo:

![executor_example_5_reentrant_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_5_reentrant_node.png)

Como você pode ver, os retornos de chamada estão sendo executados sem nenhuma restrição. Isso significa que até mesmo várias instâncias do mesmo retorno de chamada estão sendo executadas em paralelo. Esse comportamento pode ser detectado, por exemplo, porque o retorno de chamada do Timer 2, que deveria ser executado apenas uma vez a cada 3 segundos, está sendo executado muito mais vezes.

### Mutually Exclusive Callback Groups
MutuallyExclusiveCallbackGroup permite que o Executor execute apenas um de seus Callbacks simultaneamente, essencialmente como se os Callbacks fossem executados por um SingleThreadedExecutor. Portanto, é uma boa opção colocar quaisquer Callbacks que acessam recursos críticos e potencialmente não seguros para threads no mesmo MutuallyExclusiveCallbackGroup.

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_5_mutualyexclusive.cpp) foi modificado o código de forma que ele criasse um grupo de retorno de chamada MutuallyExclusive:

```c++
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

O resultado da execução é mostrado a baixo:

![executor_example_5_mutualyexclusive_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_5_mutualyexclusive_node.png)

Aqui, o comportamento é exatamente o mesmo sem Callback Groups (como no exemplo tópico **Callback Groups**). Isso ocorre porque, por padrão, todos os retornos de chamada em um nó estão dentro do mesmo grupo de retorno de chamada do tipo mutuamente exclusivo.

### Multiplos Mutually Exclusive Callback Groups
MutuallyExclusiveCallbackGroup permite que o Executor execute apenas um de seus Callbacks simultaneamente, essencialmente como se os Callbacks fossem executados por um SingleThreadedExecutor. Portanto, é uma boa opção colocar quaisquer Callbacks que acessam recursos críticos e potencialmente não seguros para threads no mesmo MutuallyExclusiveCallbackGroup.

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_executors/src/executor_example_5_mutualyexclusive_multiple.cpp) foi criado dois grupos de retorno de chamada:

```c++
  callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
```

Em seguida, atribua um a cada temporizador:

```c++
  timer1_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_1, this), callback_group_1);
  timer2_ = this->create_wall_timer(500ms, std::bind(&TwoTimer::timer_callback_2, this), callback_group_2);
```

E, neste exemplo, você está atribuindo a cada um dos temporizadores seu próprio Grupo de Retorno de Chamada.

O resultado da execução é mostrado a baixo:

![executor_example_5_mutualyexclusive_multiple_node](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/executor_example_5_mutualyexclusive_multiple_node.png)

Aqui, como você está usando dois Callback Groups MutuallyExclusive separados, o Executor cria uma thread para cada um deles. Portanto, os retornos de chamada podem ser executados em paralelo, obtendo o comportamento esperado correto:

* O Callback do Timer 1 é executado uma vez a cada segundo.
* O Callback do Timer 2 é executado uma vez a cada 3 segundos.

# Services
Assim como os Tópicos, os Serviços também são um método de comunicação entre nós no ROS 2. Para entendê-los melhor, vamos compará-los com o que você já conhece, os tópicos. 

Os Tópicos usam o modelo Publicador-Assinante (**Publisher-Subscriber**), permitindo que os nós recebam atualizações contínuas. Em contraste, os Serviços seguem um modelo Solicitação-Resposta (**Request-Response**), o que significa que eles só fornecem dados quando explicitamente solicitados por um Cliente.

Ao trabalhar com Serviços, existem duas funções principais: **Clientes e Servidores**. Vários Clientes podem usar o mesmo Servidor de Serviço, mas cada Serviço pode ter apenas um Servidor para processar as solicitações.

Alguns comandos básicos:
* Este comando **listará** todos os serviços atualmente disponíveis no seu sistema ROS2: `ros2 service list`.
* Este comando é usado para **chamar** um serviço (enviar uma solicitação): `ros2 service call service_name service_type value`.
* Comando para saber qual é o **tipo de interface** que um serviço usa: `ros2 service type service_name`. Um exemplo de retorno `std_srvs/srv/Empty`, que são os dados de **request**.
É retornado algo como:

![interface_server](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/interface_server.png)

Isso ocorre porque você está lidando com o tipo de serviço "Empty Service", que não contém nenhum dado (nem Request nem Response), neste caso, você não precisa enviar um valor ao chamar o Serviço.

Vamos analisar outro tipo de interface de serviço: `ros2 interface show std_srvs/srv/SetBool`:

![interface_server2](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/interface_server2.png)

Nesse caso a interface se serviço **std_srvs/srv/SetBool**, contem
* uma variável de **request** do tipo bool chamada data
* e duas variáveis de **response**, uma do tipo bool e outra do tipo string.

## Service Client
[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_client_service/src/service_client.cpp) um nó Cliente que interage com um nó Servidor de serviços. Considere que o Cliente que você criou será usado apenas para chamar o Serviço /moving e iniciar o movimento do robô.

Nesse exemplo você vera como chamar um serviço de um nó básico, que não está em spin():

Esta é a linha onde você cria o Cliente:

```c++
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = node->create_client<std_srvs::srv::Empty>("moving");
```

Você pode ver que criou um Cliente que usa o tipo de Serviço Vazio e se conecta a um Serviço chamado /moving.

Este loop while é usado para garantir que o Service Server (neste caso, /moving) esteja ativo e em execução:

```c++
  while (!client->wait_for_service(1s))
```

Esta linha também é importante:

```c++
  auto result_future = client->async_send_request(request);
```

Envia uma solicitação assíncrona ao Servidor de Serviço usando o método async_send_request(). Em seguida, armazene a resposta do Servidor na variável result_future. Essa variável result_future conterá o que é conhecido como um objeto future. Após fazer a solicitação, o Servidor retornará imediatamente result_future, que indica se a chamada e a resposta foram concluídas (mas não contém o valor da resposta em si). 

Por fim, um spin() no nó até que este result_future seja concluído (o serviço seja concluído):

```c++
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
```

Se for concluído com sucesso, você obterá o valor da resposta do servidor e imprimirá uma mensagem indicando que o robô está se movendo:

```c++
  auto result = result_future.get();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
```

Caso contrário, você imprime uma mensagem indicando que a chamada para o serviço falhou:

```c++
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /moving");
```

Como o nó não está em spin(), você precisa usar o método rclcpp::spin_until_future_complete para por o nó em spin até que o resultado esteja disponível.

Mas o que acontece se o nó cliente já estiver em spin por padrão? Este caso é exatamente o que vamos reproduzir no exemplo a seguir.

### Chamando um serviço a partir de um spinning node
[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_client_service/src/service_client_v2.cpp) um resultado bastante semelhante ao do anterior. No entanto, o código apresenta diversas diferenças importantes que você analisará na seção seguinte.

Como você pode ver, agora empacotamos o código do cliente dentro de uma classe:

```c++
  class ServiceClient : public rclcpp::Node
```

Dentro da nossa classe, adicionamos um método timer que faz algumas coisas. Primeiro, ele verifica se o serviço já foi chamado ou não usando o sinalizador service_called_:

```c++
void timer_callback() {
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
}
```

* Se o serviço ainda não tiver sido chamado, ele executará o método send_async_request().
* Se o serviço já tiver sido chamado, ele apenas imprimirá uma mensagem de log.

Em seguida, temos o método send_async_request(). Aqui, fazemos várias coisas. Primeiro, verificamos se o serviço está disponível ou não:

```c++
while (!client_->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Client interrupted while waiting for service. Terminating...");
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "Service Unavailable. Waiting for Service...");
}
```

Se o serviço estiver disponível, ele enviará uma solicitação e definirá o sinalizador service_called_ como true:

```c++
auto request = std::make_shared<std_srvs::srv::Empty::Request>();
auto result_future = client_->async_send_request(request, std::bind(&ServiceClient::response_callback, this, std::placeholders::_1));
service_called_ = true;
```

Como você pode ver, neste caso especificamos na solicitação uma função de retorno de chamada para manipular a resposta do serviço:

```c++
&ServiceClient::response_callback
```

Voltaremos a esta função de retorno de chamada mais tarde.

Em seguida, logo após enviarmos a solicitação, aguardamos a resposta por 1 segundo.

```c++
// Now check for the response after a timeout of 1 second
auto status = result_future.wait_for(1s);

if (status != std::future_status::ready) {
  RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
}
```

Mas o que está acontecendo aqui? Um cliente assíncrono retornará imediatamente o future, um valor que indica se a chamada e a resposta foram concluídas (não o valor da resposta em si), após enviar uma solicitação a um serviço.

Portanto, você pode verificar esse future para ver se há uma resposta ou não do serviço.

```c++
if (status != std::future_status::ready)
```

Se a resposta estiver pronta dentro desse tempo limite (1s), o status terá o valor "pronto".

E aqui está o método de retorno de chamada da resposta:

```c++
void response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
  // Get response value
  // auto response = future.get();
  RCLCPP_INFO(this->get_logger(), "Response: success");
  service_done_ = true;
}
```

Este método será chamado quando o serviço retornar uma resposta. Aqui, imprimimos apenas um log mostrando o resultado e definimos o sinalizador service_done_ como true. Como estamos usando uma interface Empty para este serviço, não precisamos obter a resposta (pois não há valor de resposta). No entanto, é mostrado nos comentários como isso seria feito.

Como você pode ver, neste caso, não estamos usando o método spin_until_future_complete(). Isso ocorre porque nosso nó já está girando (spin()). Portanto, não precisamos girar o nó até que o resultado seja concluído, mas apenas verificar se o resultado está completo.

Na função main(), giramos o nó até que o serviço seja concluído:

```c++
while (!service_client->is_service_done()) {
  rclcpp::spin_some(service_client);
}
```

O método que define quando o serviço foi concluído é um booleano simples que verifica o valor de service_done_:

```c++
bool is_service_done() const { return this->service_done_; }
```

Para resumir, vamos verificar novamente a saída recebida da execução deste cliente:

![client-service](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/client-service.png)

O que acontece aqui é o seguinte:

Primeiro, como o serviço não foi chamado inicialmente, enviamos uma solicitação a ele:

```c++
if (!service_called_) {
  RCLCPP_INFO(this->get_logger(), "Send Async Request");
  send_async_request();
}
```

Após o envio da solicitação, a resposta não fica imediatamente pronta, por isso vemos a mensagem de log apropriada:

```c++
auto status = result_future.wait_for(1s);
if (status != std::future_status::ready) {
  RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
}
```

Na 2ª iteração, o serviço já foi chamado, portanto, apenas imprimimos a seguinte mensagem de log:

```c++
else {
  RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
}
```

Finalmente, a resposta do serviço está pronta, então o robô começa a se mover e o método response_callback é chamado:

```c++
void response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
  // Get response value
  // auto response = future.get();
  RCLCPP_INFO(this->get_logger(), "Response: success");
  service_done_ = true;
}
```

## Service Server
Lembre-se de que você pode habilitar a comunicação entre nós usando Serviços. O nó que envia uma solicitação a um Serviço é chamado de Cliente, enquanto o nó que responde a essa solicitação é o Servidor. Como você deve se lembrar, no início deste módulo, você trabalhou com o Serviço /moving, com um Servidor em execução por trás deles.

No próximo exemplo, mostraremos o código por trás dos Servidores com os quais você interagiu durante esta unidade. 

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_client_service/src/server.cpp) um código para implementar o serviço **/moving** usado para mover o robô.

Dê uma olhada nas partes mais importantes do código.

Esta é a linha onde o Servidor está sendo criado:

```c++
  srv_ = create_service<Empty>("moving", std::bind(&ServerNode::moving_callback, this, _1, _2));
```

* O Serviço usa um tipo Vazio.
* O nome do Serviço é moving.
* O retorno de chamada do Serviço é moving_callback.

Sempre que o Serviço /moving recebe uma solicitação de um Cliente, o método moving_callback é executado.

Analise também o código a seguir, que foi escrito para implementar o Serviço /stop que você usa para interromper o movimento do robô. Você não encontrará muitas outras diferenças além do nome com o qual o Serviço será iniciado e dos parâmetros de velocidade para o Tópico /cmd_vel necessários para interromper o robô.

```c++
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("service_stop"){
    srv_ = create_service<Empty>("stop", std::bind(&ServerNode::stop_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Empty>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void stop_callback( const std::shared_ptr<Empty::Request> request, const std::shared_ptr<Empty::Response>  response){
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
```

## Interfaces de serviço personalizadas
[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_custom_interfaces/srv/MyCustomServiceMessage.srv) uma interface de serviço criada. Essa interface receba como solicitação (request) três movimentos possíveis: "Turn Right," "Turn Left," and "Stop." O arquivo de interface **MyCustomServiceMessage.srv** contem as seguintes variáveis:

```txt
string move   # Signal to define the movement
              # "Turn right" to make the robot turn in the right direction.
              # "Turn left" to make the robot turn in the left direction. 
              # "Stop" to make the robot stop the movement.

---
bool success  # Did it achieve it?
```

Para criar uma nova interface de serviço (srv), siga os seguintes passos:

1. Crie um diretório chamado **srv** dentro do seu pacote
2. Dentro deste diretório, crie um arquivo chamado **MyCustomServiceMessage.srv** (mais informações abaixo)
3. Modifique o arquivo **CMakeLists.txt** (mais informações abaixo)
4. Modifique o arquivo **package.xml** (mais informações abaixo)
5. **Compile** e crie o código-fonte
6. **Use** no código

Para verificar se sua service_message foi criada com sucesso use: `ros2 interface show package_name/srv/MyCustomServiceMessage`.

### Editando o CMakeLists.txt

#### find_package()
É aqui que vão todos os pacotes necessários para COMPILAR as mensagens dos Tópicos, Serviços e Ações. Em **package.xml**, defina-os como **build_depend** e **exec_depend**.

```txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```

#### rosidl_generate_interfaces()
Esta função inclui todas as mensagens deste pacote (na pasta srv) a serem compiladas. A função deve ter a seguinte aparência:

```txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyCustomServiceMessage.srv"
)
```

### Editando o package.xml
Adicione as seguintes linhas ao arquivo package.xml:

```txt
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## Usando interfaces de serviço personalizadas
[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_movement) vamos ver como importar uma interface de serviço personalizada (criada no marcos_custom_interfaces), e usar em outro pacote (nesse exemplo é o pacote marcos_movement).

Para usar a interface de serviço customizada, que foi criada, em outros pacotes, é necessário:

1. Usar o pacote onde a interface foi criada como dependência no momento da criação do novo pacote (--dependencies rclcpp **marcos_custom_interfaces** std_msgs geometry_msgs sensor_msgs), ou incluir manualmente no **CMakeLists.txt**:

```txt
find_package(marcos_custom_interfaces REQUIRED)
```

2. Modifique o **CMakeLists.txt** para instalar o arquivo de inicialização que você criou e adicione os pontos de entrada aos scripts executáveis:


```txt
#--------------usa o pacote da interface criada como dependencia-------------------------------
ament_target_dependencies(movement_server_node rclcpp geometry_msgs marcos_custom_interfaces)
#----------------------------------------------------------------------------------------------

```

3. Importar a interface criada no executável:

```c++
#include "marcos_custom_interfaces/srv/my_custom_service_message.hpp"
```

Após ter importado a nova interface criada para o novo pacote (marcos_movement), inicie o nó do servidor de serviço: `ros2 launch marcos_movement movement_server_launch_file.launch.py`.

Em outro terminal envie um request para o servidor do serviço **movement**: `ros2 service call /movement marcos_custom_interfaces/srv/MyCustomServiceMessage move:\ 'Turn Left'` para fazer o robo se mover para a esquerda.

O seguinte log é retornado:

![serviço](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/servi%C3%A7o.png)

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_movement/src/movement_client.cpp) é criado um executável (movement_client.cpp) cliente para chamar o serviço **movement**.

Vamos olhar as partes mais importantes do código movement_client.cpp:

Essa linha cria um serviço ROS 2 no C++ usando o rclcpp, associando o nome do serviço a um callback que será chamado quando o serviço for solicitado.:
1. Cria um serviço chamado /movement
2. Usa o tipo MyCustomServiceMessage
3. Liga o serviço ao método moving_callback da classe ServerNode
4. Faz com que, ao receber uma chamada de serviço, o ROS 2 execute moving_callback(request, response)

```c++
srv_ = create_service<MyCustomServiceMessage>("movement", std::bind(&ServerNode::moving_callback, this, _1, _2));
```

* create_service<MyCustomServiceMessage>(...): Função do rclcpp::Node (ou herdada dele) que cria um servidor de serviço do tipo MyCustomServiceMessage.
    Espera dois argumentos:
    * O nome do serviço, que será o nome do tópico do serviço ("movement")
    * A função de callback que será chamada quando alguém requisitar esse serviço.

* std::bind(&ServerNode::moving_callback, this, _1, _2): Associa o método moving_callback da classe ServerNode como callback do serviço.
  * O this indica que o método pertence a uma instância da classe.
  * _1 e _2 são placeholders que indicam os argumentos que o ROS vai passar automaticamente para o callback (tipicamente o request e response do serviço).

No retorno de chamada do serviço, você decidirá como mover o robô dependendo da solicitação recebida:

```c++
if (request->move == "Turn Right")
{   
    // Send velocities to move the robot to the right
    message.linear.x = 0.2;
    message.angular.z = -0.2;
    publisher_->publish(message);

    // Set the response success variable to true
    response->success = true;
}
else if (request->move == "Turn Left")
{
    // Send velocities to stop the robot
    message.linear.x = 0.2;
    message.angular.z = 0.2;
    publisher_->publish(message);

    // Set the response success variable to false
    response->success = true;
}
else if (request->move == "Stop")
{
    // Send velocities to stop the robot
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);

    // Set the response success variable to false
    response->success = true;
}
else {
    response->success = false;
}
```

Portanto, para acessar a parte de resposta do tipo Serviço, use response->success:

```c++
response->success = true;
```

# Actions
Ações do ROS 2, um poderoso mecanismo de comunicação que permite que robôs executem tarefas de longa duração, fornecendo feedback e permitindo a preempção. Ao contrário dos serviços, que são síncronos, as ações permitem a execução assíncrona, tornando-as ideais para tarefas como navegação de robôs, planejamento de movimento e agarrar objetos.

Ações são semelhantes a Serviços. No entanto, ao chamar uma Ação, você está chamando uma funcionalidade que outro nó está fornecendo. Além disso, Ações são baseadas no modelo Cliente-Servidor – o mesmo que acontece com Serviços.

No entanto, existem duas diferenças principais entre Ações e Serviços:
1. Ações são preemptivas. Isso significa que você pode cancelar uma Ação enquanto ela está sendo executada.
2. Ações fornecem feedback. Isso significa que, enquanto a Ação está sendo executada, o Servidor pode enviar feedback ao Cliente.

Abaixo, você pode ver um diagrama que descreve o fluxo de trabalho de uma Ação.

![actions](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/actions.png)

Como você pode ver no gráfico acima, as Ações usam Serviços para lidar com a meta (goal) e o resultado (result), e usam Tópicos para lidar com o feedback.

Não se preocupe se você não entendeu completamente agora, pois ele contém muitos conceitos. Por fim, tenha em mente os dois pontos a seguir:

* O nó que fornece a funcionalidade de Ação precisa conter um Servidor de Ação. O Servidor de Ação permite que outros nós chamem essa funcionalidade de Ação.
* O nó que chama a funcionalidade de Ação precisa conter um Cliente de Ação. O Cliente de Ação permite que um nó se conecte ao Servidor de Ação de outro nó.

Resumindo, o fluxo de trabalho é assim:

1. O Cliente envia uma meta para o Servidor. Isso acionará o "início" da Ação.
2. O Servidor envia feedback para o Cliente enquanto a Ação está em andamento.
3. Assim que a Ação termina, o Servidor retorna uma resposta (response) para o Cliente.

Veja alguns comandos básicos:
* Para descobrir quais ações estão disponíveis em um robô: `ros2 action list`.
* Você também pode obter dados de uma Ação específica com o seguinte comando: `ros2 action info /action_name`.
* Além disso, se você adicionar o sufixo -t ao comando acima, obterá dados da interface de ação usada: `ros2 action info /action_name -t`.
* Para chamar um action: `ros2 action send_goal <action_name> <action_type> <values>`, por exemplo `ros2 action send_goal /move_robot_as t3_action_msg/action/Move "{secs: 5}"`.

Ações fornecem feedback. E você pode, de fato, visualizar esse feedback. No entanto, você precisa especificar que deseja visualizar o feedback ao chamar o Servidor de Ações: `ros2 action send_goal -f /move_robot_as t3_action_msg/action/Move "{secs: 5}"`.

Observe o argumento -f adicionado ao comando, que é a forma abreviada de --feedback. Você pode usar qualquer um como desejar.

* Você também pode obter mais dados sobre a interface de um action com o seguinte comando: `ros2 interface show package_name/action/interface_name`. Por exemplo, o comando `ros2 interface show t3_action_msg/action/Move` retorna o seguinte:

![message_action](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/message_action.png)

A mensagem é composta por três partes separadas por três hífens ---:
* int32 segs: Consiste em uma variável chamada secs do tipo int32. Essa variável é usada como a meta (**goal**) solicitada e indica o número de segundos em que o robô deve avançar.
* string status: Consiste em uma variável chamada status, que é do tipo string. Ela é usada para comunicar o resultado (**result**), pois a string indica o status final quando a Ação termina.
* string feedback: Consiste em uma variável chamada feedback do tipo string. Essa string é usada para indicar o **status** atual do robô.

## Chamando um action server
Chamar um Servidor de Ação significa enviar um objetivo a ele. Assim como Tópicos e Serviços, tudo funciona por meio da transmissão de mensagens.

* A mensagem de um Tópico é composta por uma única parte: as informações fornecidas pelo Tópico.
* A mensagem de um Serviço tem duas partes: a solicitação e a resposta.
* A mensagem de um Servidor de Ação é dividida em três partes: o objetivo, o resultado e o feedback.

Todas as mensagens de Ação utilizadas são definidas no diretório Action do respectivo pacote.

## Actions fornecendo feedback
Como chamar um Servidor de Ação não interrompe sua thread, os Servidores de Ação fornecem uma mensagem chamada feedback. O feedback é uma mensagem que o Servidor de Ação gera ocasionalmente para indicar o andamento da Ação (informando quem a chamou sobre o status da Ação solicitada). Ela é gerada enquanto a Ação está em andamento.

## Action Client
A maneira de chamar um Action Server é implementando um Action Client.

[Esse exemplo](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_action_client) mostra como implementar um Action Client que chama o Action Server **/move_robot_as** e comanda o robô para avançar por cinco segundos.

Como você deve ter notado, após importar algumas bibliotecas C++ nas primeiras quatro linhas, você não faz nada além de importar as bibliotecas de cliente C++ do ROS2 para trabalhar com ações (**rclcpp_action**) e (**rclcpp**). Você também pode ver que é aqui que você importará as interfaces com as quais trabalha, neste caso, (**t3_action_msg.action**).7

```c++
#include "t3_action_msg/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
```

Agora vamos pular para a função principal que usa executores:
* Aqui, você cria uma instância de **MyActionClient()**. Em seguida, inicializamos um objeto **MultiThreadedExecutor**.
* Enquanto a meta não for concluída, **while** (!action_client->is_goal_done()), você executará o executor **executor.spin_some()**;.

```c++
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}
```

Agora, vamos continuar analisando o construtor da classe **MyActionClient()**:

```c++
Node("my_action_client", node_options), goal_done_(false){
  this->client_ptr_ = rclcpp_action::create_client<Move>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "move_robot_as");

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MyActionClient::send_goal, this));
}
```

No construtor da classe, inicializamos um nó ROS2 chamado **my_action_client**. Observe também que inicializamos a variável **goal_done_** como false.
Em seguida, dentro do corpo do construtor, criamos um objeto **ActionClient** que se conecta ao Servidor de Ações **/move_robot_as**.
Finalmente, criamos um objeto timer, com um método de retorno de chamada chamado **send_goal**.

Para continuar, vamos analisar o método **send_goal()**:

```c++
void send_goal()
  {
    using namespace std::placeholders;
    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Move::Goal();
    goal_msg.secs = 5;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();                
    send_goal_options.goal_response_callback = std::bind(&MyActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&MyActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&MyActionClient::result_callback, this, _1);      
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
```
A linha abaixo cancela o temporizador para que ele seja executado apenas uma vez (neste exemplo, não queremos continuar enviando metas para o Action Server):

```c++
this->timer_->cancel();
```

Aqui, verificamos se o Servidor de Ações está ativo e funcionando. Caso contrário, imprimimos uma mensagem no log do nó:

```c++
if (!this->client_ptr_) {
  RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
}
```

Aqui, aguardamos a inicialização do Servidor de Ações por 10 segundos. Se ele não estiver pronto após 10 segundos, aguarde.

```c++
if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
  RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
  this->goal_done_ = true;
  return;
}
```

Em seguida, um objeto **Goal()** do tipo de ação **Move** é criado. Em seguida, acessamos a variável **secs** da meta **Action** e atribuímos a ela um valor numérico arbitrário em segundos. Neste exemplo, são 5 segundos.

```c++
auto goal_msg = Move::Goal();
goal_msg.secs = 5;
```

Aqui, defina os diferentes retornos de chamada para o Action Client:

```c++
auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();                
send_goal_options.goal_response_callback = std::bind(&MyActionClient::goal_response_callback, this, _1);            
send_goal_options.feedback_callback = std::bind(&MyActionClient::feedback_callback, this, _1, _2);            
send_goal_options.result_callback = std::bind(&MyActionClient::result_callback, this, _1);
```

A primeira linha acima inicializa o objeto usado para definir métodos de retorno de chamada para aceitação, feedback e conclusão de metas.

Como você pode ver, eles são definidos imediatamente após:

* goal_response_callback
* feedback_callback
* result_callback

Finalmente, enviamos a meta (goal) para o Servidor de Ações usando o método **async_send_goal**:

```c++
auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
```

Fornecemos dois argumentos para este método:

* Uma mensagem de meta, neste caso, goal_msg
* Os métodos de retorno de chamada do cliente

Este método **async_send_goal()** retorna um **future** para um identificador de meta. Este identificador de meta **future** será concluído quando o Servidor tiver processado a meta, seja ela aceita ou rejeitada pelo Servidor. Portanto, você deve atribuir um método de retorno de chamada para ser acionado quando o **future** for concluído (a meta tiver sido aceita ou rejeitada). Neste caso, este método é **goal_response_callback()**.

Agora, observe este método **goal_response_callback()**:

```c++
void goal_response_callback(const GoalHandleMove::SharedPtr & goal_handle){
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}
```

Portanto, esse método é acionado quando a meta é aceita ou rejeitada pelo servidor. Podemos saber isso verificando o valor **goal_handle**.

```c++
if (!goal_handle)
```

Abaixo, imprimimos uma mensagem caso a meta tenha sido rejeitada. Caso tenha sido aceita, aguardaremos o resultado.

```c++
if (!goal_handle) {
  RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
} else {
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
}
```

Semelhante ao envio da meta, este método retornará um **future** que será concluído quando o resultado estiver pronto. Portanto, você também deve atribuir um método de retorno de chamada para ser acionado quando esse **future** for concluído (o resultado estiver pronto). Neste caso, este método é **result_callback()**:

Agora, observe este método **result_callback()**:

```c++
void result_callback(const GoalHandleMove::WrappedResult & result){
  this->goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->status.c_str());
}
```

Este método é bem simples. Primeiro, verifique a variável **result.code** para ver o que aconteceu com o seu objetivo. Em seguida, imprima a variável status do nosso resultado.

Finalmente, você tem o método **feedback_callback**:


```c++
void feedback_callback(
  GoalHandleMove::SharedPtr,
  const std::shared_ptr<const Move::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Feedback received: %s", feedback->feedback.c_str());
}
```

Aqui, imprima a sequência de feedback no log do nó.

## Action Server
No ROS 2, as Ações facilitam a comunicação entre nós por meio de um processo orientado a objetivos (goal). O nó que envia um objetivo para uma Ação é chamado de Cliente, enquanto o nó que processa e responde ao objetivo é o Servidor. No tópico **Action Client**, você interagiu com a Ação /move_robot_as, que é suportada por um Servidor de Ações em execução.

[Nesse exemplo](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_action_server), foi criado um Servidor de Ações, com base no Servidor de Ações /move_robot_as (da sessão anterior). 

Vamos analisar o código com mais detalhe.

Comece com a primeira seção, onde você importará as bibliotecas necessárias.

```c++
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "t3_action_msg/action/move.hpp"
#include "geometry_msgs/msg/twist.hpp"
```

Em seguida, defina sua classe **MyActionServer** que herda de **Node**:

```c++
class MyActionServer : public rclcpp::Node{
public:
  using Move = t3_action_msg::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
}
```

Observe que você também está criando um "alias" para simplificar o código.

```c++
explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("my_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move_robot_as_2",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
```

No construtor da classe, inicialize um nó ROS2 chamado **my_action_server**. Além disso, e importante, crie um objeto **ActionServer** para o qual você especifique várias coisas:

1. O tipo de Ação: **Move**.
2. O nó ROS2 que contém o Servidor de Ação: neste caso, **this**.
3. Um método de retorno de chamada a ser executado quando a Ação recebe uma meta: **handle_goal**.
4. Um método de retorno de chamada a ser executado se o Cliente cancelar a meta atual: **handle_cancel**.
5. Um método de retorno de chamada a ser executado se a meta for aceita: **handle_accepted**.
6. Finalmente, defina um Publicador para o Tópico **/cmd_vel**.

Agora continue analisando o método **handle_goal()**:

```c++
rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Move::Goal> goal){
  RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->secs);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

Este método será chamado quando o Servidor de Ações receber uma meta. Nesse caso, você está aceitando todas as metas recebidas com **rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE**.

Em seguida, encontre o método **handle_cancel()**:

```c++
rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle){
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
```

Este método será chamado quando o Servidor de Ações receber uma solicitação de cancelamento do Cliente. Nesse caso, você estará aceitando todas as solicitações de cancelamento com **rclcpp_action::CancelResponse::ACCEPT**.

Finalmente, encontre o método **handle_accepted()**:

```c++
void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)  {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
}
```

Este método será chamado quando o Servidor de Ação aceitar o objetivo. Aqui, você está chamando a função **execute**, que contém a funcionalidade principal da Ação.

Finalmente, você tem o método **execute**:

```c++
void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
```

Este método é o que realmente fará o trabalho. Revise-o passo a passo. Primeiro, defina todas as variáveis que você usará:

```c++
const auto goal = goal_handle->get_goal();
auto feedback = std::make_shared<Move::Feedback>();
auto & message = feedback->feedback;
message = "Starting movement...";
auto result = std::make_shared<Move::Result>();
auto move = geometry_msgs::msg::Twist();
rclcpp::Rate loop_rate(1);
```

Você tem:

* A variável "**goal**", contendo a mensagem de meta enviada pelo Cliente.
* A variável "**feedback**", contendo a mensagem de feedback que você enviará de volta ao Cliente.
* A variável "**result**", contendo a mensagem de resultado que você enviará ao Cliente quando a Ação terminar.
* A variável "**mov**", contendo a mensagem "Twist" usada para enviar velocidades ao robô.
* Uma variável "**loop_rate**" de 1 Hz (1 segundo).

Em seguida, inicie um loop "for" que continuará em execução até que a variável "i" atinja o número de segundos especificado na mensagem de meta.

```c++
for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i)
```

Por exemplo, se você definir cinco segundos na mensagem de meta, este loop será executado uma vez por segundo, durante 5 segundos.

Em seguida, verifique se a Ação foi cancelada. Se for, encerre a Ação.

```c++
if (goal_handle->is_canceling()) {
  result->status = message;
  goal_handle->canceled(result);
  RCLCPP_INFO(this->get_logger(), "Goal canceled");
  return;
}
```

Se a Ação não for cancelada, você enviará uma mensagem Twist ao robô para movê-lo para frente a uma velocidade de 0,3 m/s, e enviará uma mensagem de feedback ao Cliente com a string "Moving forward...".


```c++
message = "Moving forward...";
move.linear.x = 0.3;
publisher_->publish(move);
goal_handle->publish_feedback(feedback);
RCLCPP_INFO(this->get_logger(), "Publish feedback");
loop_rate.sleep();
```

No final da função **execute**, verifique se o objetivo foi concluído:

```c++
if (rclcpp::ok()) {
  result->status = "Finished action server. Robot moved during 5 seconds";
  move.linear.x = 0.0;
  publisher_->publish(move);
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}
```

Se o objetivo for alcançado, você fará duas coisas:

* Parar o robô enviando uma velocidade de 0.
* Preencher a mensagem de resultado e enviá-la de volta ao Cliente.

Finalmente, encontrar o principal que se parece com se adicionarmos um **MultiThreadedExecutor** dentro dele:

```c++
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

Como você pode ver, é simples. **Spin** o nó do Servidor de Ações para mantê-lo ativo e em execução.

## Action Interface custumizada
Para criar sua própria Interface de Ação, você deve concluir as três etapas a seguir:

1. Crie um diretório de **actions** no seu no pacote.
2. Crie seu arquivo de Interface de Ação **Nome_da_interface.action** no diretório actions.

O nome do arquivo de Interface de Ação determinará posteriormente o nome das classes a serem usadas no Servidor de Ação e/ou no Cliente de Ação. A convenção ROS2 indica que o nome deve ser escrito em camelcase.

Lembre-se de que o arquivo de Interface de Ação deve conter três partes, cada uma separada por três hífens.

```txt
#goal
message_type goal_var_name
---
#result
message_type result_var_name
---
#feedback
message_type feedback_var_name
```

Você pode deixar qualquer uma dessas partes em branco se não precisar de uma parte da mensagem (por exemplo, se não precisar fornecer feedback). No entanto, você deve sempre especificar os separadores de hífen.

3. Modifique os arquivos **CMakeLists.txt** e **package.xml** para incluir a compilação da interface de ação. 

### Modificando o CMakeLists.txt
Chame a função **rosidl_generate_interfaces** no seu arquivo **CMakeLists.txt** para criar uma nova Interface de Ação:

```txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Nome_da_interface.action"
)
```

Para gerar Interfaces de Ação, certifique-se de ter acesso aos seguintes pacotes:

* rosidl_default_generators
* action_msgs

Para isso, adicione a linha abaixo ao seu arquivo CMakeLists.txt:

```txt
#-----Relacionado às mensagens de action criadas----
find_package(action_msgs REQUIRED)
#---------------------------------------------------

#-----Relacionado às mensagens criadas--------------
find_package(rosidl_default_generators REQUIRED)
#---------------------------------------------------
```

### Modificando o Cpackage.xml
No arquivo package.xml, certifique-se de que você tenha dependências para os seguintes pacotes:

* action_msgs
* rosidl_default_generators

```txt
  <!-- Relacionados à mensagem de actions criada-->
  <depend>action_msgs</depend>
  <depend>rosidl_default_generators</depend>
  <!-- ######################################## -->
```

Além disso, especifique o pacote **rosidl_interface_packages** como **member_of_group**:

```txt
  <member_of_group>rosidl_interface_packages</member_of_group>
```

Para verificar se sua Interface de Ação foi criada corretamente, use o seguinte comando: `ros2 interface show package_name/action/Interface_name`.

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_custom_interfaces/action/Move.action) uma interface de action criada.

## Usando uma action Interface custumizada
Para usar uma interface criada (nesse exemplo do pacote marcos_custom_interfaces) em um novo pacote criado;

1. Inclua sua interface de ação personalizada no arquivo **executável**:

```c++
#include "marcos_custom_interfaces/action/Interface_name.hpp"
```

2. No arquivo **CMakeLists.txt**, você precisará adicionar o seguinte código:

```txt
# Required to find the custom_interfaces package
find_package(marcos_custom_interfaces)

# Update the executable dependencies with the marcos_custom_interfaces package
ament_target_dependencies(action_server_node rclcpp rclcpp_action t3_action_msg marcos_custom_interfaces geometry_msgs)
```

3. No **package.xml** você precisará adicionar a dependência para o pacote marcos_custom_interfaces:

```txt
<depend>marcos_custom_interfaces</depend>
```

# Ferramentas de Debugging (Depuração)
Depuração e visualização são habilidades essenciais para qualquer desenvolvedor de robótica. 

## ROS2 Debugging Messages
Os logs podem ser exibidos na tela, mas também podem ser salvos no quadro ROS para classificação, filtragem e outros usos. Existem níveis de log nos sistemas, como visto na imagem abaixo. No caso dos logs ROS2, existem cinco níveis. Cada nível possui uma série de subníveis. Por exemplo, se você usar o nível **Erro**, o log exibirá as mensagens Erro e Fatal. Se o seu nível for **Warning**, você verá todas as mensagens dos níveis Warning, Erro e Fatal.

![níveis de log](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/log.png)

[Veja nesse exemplo](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_logs_test/src/logger_example.cpp) um nó criado para ficar imprimendo diferentes menssagens de logs.

## RVIZ2
RVIZ é uma ferramenta de visualização que permite visualizar imagens, nuvens de pontos, lasers, transformações cinemáticas, modelos de robôs e muito mais. A lista é infinita. É até possível criar seus próprios marcadores. É um dos motivos pelos quais o ROS foi tão bem recebido. Não era fácil descobrir o que o robô estava vendo antes do RVIZ. Esta é a ideia principal:

* RVIZ NÃO é uma simulação. Repito: NÃO é uma simulação.
* RVIZ representa o que está sendo publicado nos tópicos pela simulação ou pelo robô real.
* RVIZ é uma ferramenta complexa e levaria um curso inteiro para dominá-la. Aqui, você terá uma ideia do que ela pode lhe oferecer.

**Painel Central**: Aqui é onde toda a mágica acontece. É aqui que os dados serão exibidos. É um espaço 3D que você pode rotacionar (MANTENDO O BOTÃO ESQUERDO DO MOUSE PRESSIONADO), transladar (MANTENDO O BOTÃO CENTRAL DO MOUSE PRESSIONADO) e dar zoom in/out (MANTENDO O BOTÃO DIREITO DO MOUSE PRESSIONADO).

**Painel de Exibição à Esquerda**: Aqui você gerencia/configura todos os elementos que deseja visualizar no painel central. Você só precisa usar dois elementos:
* Em Opções Globais, você deve selecionar o Quadro Fixo (Fixed Frame) que se adequa à visualização dos dados. É o quadro de referência a partir do qual todos os dados serão referenciados.
* O botão Adicionar (Add). Clicando aqui, você obtém todos os tipos de elementos que podem ser representados no RVIZ.

## Vizualizar os frames do robo
Outra ferramenta útil é o programa **view_frames**. Este programa permite visualizar um diagrama de árvore mostrando a relação entre os diferentes quadros do seu robô.

Para gerar um pdf com o diagrama: `ros2 run tf2_tools view_frames`.

O arquivo será gerado no diretório atual do terminal.

Esta ferramenta é útil porque permite ver a conexão entre cada quadro e determinar se uma transformação não foi bem executada. Por exemplo, pode não haver relação entre um mapa gerado e o robô que deseja navegar, ou um sensor e sua base, etc.

## ROS2 Doctor
O ROS 2 Doctor (ou ros2 doctor) é uma ferramenta de diagnóstico incluída no ROS 2 que verifica se o seu ambiente está corretamente configurado para usar o ROS 2.

Esta poderosa ferramenta analisa toda a configuração do ROS2, incluindo plataforma, versões, rede, ambiente, etc., além de fornecer um diagnóstico preciso com precauções, erros e possíveis causas dos problemas.

Não se esqueça de que o ros2doctor pertence ao pacote **ros2cli**, portanto, você precisará instalá-lo para usá-lo.

### O que o `ros2 doctor` faz?

Ele **realiza uma série de testes automáticos** para detectar problemas comuns no ambiente de desenvolvimento ROS 2, como:

* Variáveis de ambiente (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, etc.)
* Conectividade entre nodes
* Comunicação via DDS
* Instalações faltando
* Conflitos entre workspaces
* Status da rede
* Detecção de múltiplas fontes de ROS (mistura de versões)

* Se `ROS_DISTRO` está definido corretamente
* Se o `rmw` (middleware DDS) está compatível
* Se o `ros2` está instalado corretamente
* Se há conflitos entre múltiplos workspaces
* Se há problemas de DNS que podem afetar a descoberta de nodes
* Se há pacotes ausentes

### Quando usar o `ros2 doctor`?

Use sempre que:

* O ROS 2 não estiver se comportando como esperado
* Nodes não estiverem se comunicando
* Você configurar um novo ambiente
* Houver dúvidas sobre configuração de rede, DDS ou variáveis de ambiente


### Verifique toda a sua configuração
Este é o primeiro e mais significativo nível de verificação da ferramenta ros2doctor. Você só precisa executar o ros2 doctor: `ros2 doctor`.

Se a configuração do ROS2 estiver em perfeitas condições, ela passará em todas as verificações, apresentando **All 5 checks passed** no terminal.

Se algo estiver errado, como será a saída? Bem, depende do que está errado. Você pode receber uma saída semelhante a essa: 

```bash
1/4 checks failed

Failed modules:  network
```

### Obtenha um relatório completo com o ROS2 Doctor
Se você quiser saber mais detalhes para analisar problemas, --report é a melhor opção: `ros2 doctor --report`.

Você verá que o relatório está categorizado em 7 seções:

* NETWORK CONFIGURATION: Você pode ver todas as informações relacionadas à configuração de rede do sistema.
* PACKAGE VERSION: Esses são pacotes que podem ser atualizados para a última versão.
* PLATFORM CONFIGURATION: Estas são informações sobre a plataforma que você está usando.
* QOS COMPATIBILITY LIST: Estas são informações sobre a configuração de QoS do seu sistema.
* RMW MIDDLEWARE: Estas são informações sobre o MIDDLEWARE RMW, que define uma interface de primitivas de middleware usadas pelas APIs ROS de nível superior.
* ROS 2 INFORMATION: Estas são informações sobre o ROS2 que você está usando.
* TOPIC LIST: Estas são informações sobre os tópicos que estão sendo trabalhados com assinantes e editores.

Você também pode usar `ros2 doctor --help` para ver todas as opções disponíveis e realizar diagnósticos específicos, como:

```bash
ros2 doctor --report --report-network
ros2 doctor --report --report-rmw
```

# Node Composition
Ao desenvolver programas (nós) em ROS 2, você normalmente os compila em executáveis que podem ser executados diretamente (ros2 run) ou usando um arquivo de inicialização (ros2 launch). No entanto, há uma limitação nessa abordagem: cada executável é executado em um processo separado, o que significa que não é possível executar vários nós ROS em um único processo.

Para abordar isso, o ROS 2 apresenta **componentes**. Mas o que exatamente são componentes e como eles ajudam a melhorar a eficiência e o gerenciamento de recursos? Nesta unidade, você explorará o conceito de componentes do ROS 2, aprenderá como criá-los e entenderá como eles permitem que vários nós sejam gerenciados com eficiência dentro do mesmo processo.

## componentes
Componentes são o equivalente no ROS2 aos conhecidos **nodelets** do ROS1. Ao escrever seu programa como um componente, você pode compilá-lo em uma biblioteca compartilhada em vez de um executável. Isso permite carregar vários componentes em um único processo.

Existem duas abordagens principais sobre como usar componentes:

* Composição em tempo de execução: A composição em tempo de execução permite que a configuração e o comportamento do sistema robótico sejam modificados ou estendidos enquanto o sistema está em execução. Os componentes podem ser adicionados, removidos ou reconfigurados dinamicamente sem interromper todo o sistema.
* Composição em tempo de compilação: A composição em tempo de compilação envolve a especificação da configuração do sistema no momento da compilação do código. Isso significa que a estrutura e o comportamento do sistema robótico são determinados antes que o código seja compilado em um executável.

Ao escolher entre composição em tempo de compilação e composição em tempo de execução, é essencial ponderar os prós e os contras:
* A composição em tempo de compilação geralmente oferece desempenho superior e depuração simplificada devido ao seu objeto composto totalmente definido e otimizado. 
* A composição em tempo de execução oferece maior flexibilidade e adaptabilidade porque o objeto composto pode ser criado e modificado dinamicamente durante a execução do programa.

## Composição em tempo de execução
Para usar componentes é preciso adicionar **rclcpp_components** e **composition** como dependências ao pacote. Isso pode ser feito no momento da criação do pacote (`ros2 pkg create ...  --dependencies rclcpp rclcpp_components composition ...`), ou adicionando no arquivo **CMakeLists.txt**:

```txt
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
```

Primeiro, crie um arquivo **hpp** para definir sua classe, vanos usar como exeplo um pacote chamado **marcos_components**. Dentro da pasta **include/marcos_components** do pacote, crie um novo arquivo chamado **moverobot_component.hpp** e cole o código abaixo nele:

```c++
#ifndef COMPOSITION__MOVEROBOT_COMPONENT_HPP_
#define COMPOSITION__MOVEROBOT_COMPONENT_HPP_

#include "marcos_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace marcos_components{
  class MoveRobot : public rclcpp::Node  {
    public:
      COMPOSITION_PUBLIC
      explicit MoveRobot(const rclcpp::NodeOptions & options);

    protected:
      void on_timer();

    private:
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace composition
#endif  // COMPOSITION__MOVEROBOT_COMPONENT_HPP_
```

Neste arquivo, você define as diferentes variáveis e funções que usará na sua classe **MoveRobot**. Observe também que você está encapsulando tudo dentro de um namespace **marcos_components**.

Em seguida, você criará seu programa. Dentro da pasta **src** do pacote, crie um novo script chamado **moverobot_component.cpp** e cole o código abaixo nele:

```c++
#include "marcos_components/moverobot_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace marcos_components{
  MoveRobot::MoveRobot(const rclcpp::NodeOptions & options)
  : Node("moverobot", options){
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(1s, std::bind(&MoveRobot::on_timer, this));
  }

  void MoveRobot::on_timer(){
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.3;
    msg->angular.z = 0.3;
    std::flush(std::cout);
    pub_->publish(std::move(msg));
  }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(marcos_components::MoveRobot)
```

Vamos ver as partes mais importantes do código.

Primeiro, importe o arquivo .hpp que você criou:

```c++
#include "marcos_components/moverobot_component.hpp"
```

Observe que você também está encapsulando tudo dentro do namespace **marcos_components**:

```c++
namespace marcos_components{ }
```

Por fim, registre seu programa como um componente:

```c++
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(marcos_components::MoveRobot)
```

Observe que, como um componente é criado apenas em uma biblioteca compartilhada, ele não tem uma função principal.

Agora, crie outro arquivo, também dentro da pasta include/marcos_components, chamado **visibility_control.h**. Copie o código mostrado abaixo.

```c++
#ifndef COMPOSITION__VISIBILITY_CONTROL_H_
#define COMPOSITION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
  #endif

  // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
  //     https://gcc.gnu.org/wiki/Visibility

  #if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
      #define COMPOSITION_EXPORT __attribute__ ((dllexport))
      #define COMPOSITION_IMPORT __attribute__ ((dllimport))
    #else
      #define COMPOSITION_EXPORT __declspec(dllexport)
      #define COMPOSITION_IMPORT __declspec(dllimport)
    #endif
    #ifdef COMPOSITION_BUILDING_DLL
      #define COMPOSITION_PUBLIC COMPOSITION_EXPORT
    #else
      #define COMPOSITION_PUBLIC COMPOSITION_IMPORT
    #endif
    #define COMPOSITION_PUBLIC_TYPE COMPOSITION_PUBLIC
    #define COMPOSITION_LOCAL
  #else
    #define COMPOSITION_EXPORT __attribute__ ((visibility("default")))
    #define COMPOSITION_IMPORT
    #if __GNUC__ >= 4
      #define COMPOSITION_PUBLIC __attribute__ ((visibility("default")))
      #define COMPOSITION_LOCAL  __attribute__ ((visibility("hidden")))
    #else
      #define COMPOSITION_PUBLIC
      #define COMPOSITION_LOCAL
    #endif
    #define COMPOSITION_PUBLIC_TYPE
  #endif

  #ifdef __cplusplus
}
#endif

#endif  // COMPOSITION__VISIBILITY_CONTROL_H_
```

Basicamente, este arquivo otimizará o processo de carregamento de bibliotecas compartilhadas. Se você quiser mais detalhes sobre este arquivo, [consulte: https://gcc.gnu.org/wiki/Visibility](https://gcc.gnu.org/wiki/Visibility) 

Modifique o arquivo **CMakeLists.txt** para gerar a biblioteca compartilhada apropriada a partir do componente que você criou:

```txt
include_directories(include)

add_library(moverobot_component SHARED src/moverobot_component.cpp)
target_compile_definitions(moverobot_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(moverobot_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs")
rclcpp_components_register_nodes(moverobot_component "marcos_components::MoveRobot")
set(node_plugins "${node_plugins}marcos_components::MoveRobot;$<TARGET_FILE:moverobot_component>\n")

install(TARGETS
  moverobot_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

Vamos revisar as partes mais importantes do código.

Primeiro, gere a biblioteca compartilhada a partir do nosso script C++:

```c++
add_library(moverobot_component SHARED src/moverobot_component.cpp)
```

Registre seu componente:

```c++
rclcpp_components_register_nodes(moverobot_component "marcos_components::MoveRobot")
set(node_plugins "${node_plugins}marcos_components::MoveRobot;$<TARGET_FILE:moverobot_component>\n")
```

Por fim, instale-o em seu espaço de trabalho:

```c++
install(TARGETS
  moverobot_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

Apoś compilar, verificar se o componente foi criado com sucesso, você pode usar o comando `**ros2 component types**`. Este comando retornará uma lista com os componentes disponíveis:

Verifique se o componente **marcos_components::MoveRobot** foi criado.

Para **carregar** um componente, inicie o contêiner de componentes. Execute o seguinte comando: `ros2 run rclcpp_components component_container`

Agora, verifique se o contêiner está em **execução** com o seguinte comando: `ros2 component list`

**Carregue** seu componente. Execute o seguinte comando: `ros2 component load /ComponentManager marcos_components marcos_components::MoveRobot`

A estrutura do comando é a seguinte: `ros2 component load /ComponentManager <pkg_name> <component_name>`

O processo é o seguinte:

1. O gerenciador de componentes carrega o programa como uma biblioteca compartilhada.
2. Encontra a classe MoveRobot definida dentro do componente.
3. Instancia essa classe MoveRobot como um nó ROS.

Você também pode garantir que seu componente foi carregado corretamente usando o comando ros2 component list: `ros2 component list`

Como você pode ver, o componente /moverobot é identificado com o número 1. Todos os componentes carregados serão identificados com seu próprio número.

Também é possível **descarregar** componentes. Como você pode imaginar, isso é feito com o comando ros2 component unload: `ros2 component unload /ComponentManager <component_id>` por exemplo: `ros2 component unload /ComponentManager 1`

[Veja nesse pacote](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/exemplos/marcos_components) a componente criada.