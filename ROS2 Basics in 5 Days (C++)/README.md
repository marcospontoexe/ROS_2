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

Para obter informações sobre uma interface, use o seguinte comando: `ros2 interface show package_name/message_name/message_type` ou `ros2 interface proto package_name/message_name/message_type`, por exemplo `ros2 interface show std_msgs/msg/Int32`.

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
* Comando para saber qual é o **tipo de interface** que um serviço usa: `ros2 service type service_name`. Um exemplo de retorno `std_srvs/srv/Empty`.
É retornado algo como:
![interface_server](https://github.com/marcospontoexe/ROS_2/blob/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)/imagens/interface_server.png)
