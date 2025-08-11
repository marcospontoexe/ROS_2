# Behavior Trees
Nesta unidade, você entenderá o conceito de BT e a arquitetura de software simplificada que pode ser acomodada no framework ROS2. As Árvores de Comportamento são usadas para criar uma lógica para decidir o que **fazer** e **quando**.

Agora é hora de executar a Árvore de Comportamento! Aqui, o comportamento do robô (o robô gira, analisa o sinal do laser, detecta os obstáculos, move-se para o objetivo, etc.) é modelado em um BT (fluxo lógico de ação do robô):

![bt-t3-sim](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/bt-t3-sim.png)

* Você pode imaginar que o robô gostaria de ir para a "sala" contígua.
* O robô, usando o laser, escaneia os obstáculos (neste caso, procura a porta aberta).
* Você tem a "chave para abrir a porta". Você pode fazer isso removendo a parede na simulação.
* O robô detecta o caminho desobstruído e se move para a outra sala.

A BT para esse exemplo pode ser representada da seguinte forma.

Usamos o Groot para representar graficamente os detalhes da BT implantados na simulação. Leia a BT de cima para baixo e da esquerda para a direita.

* Na simulação a seguir, analisando a BT de cima para baixo, você pode ver a Root conectada à Sequência Reativa. Posteriormente, a primeira Sequência Reativa é conectada a:
    1. BlackBoard (detalhes posteriormente)
    2. Segunda Sequência Reativa
    3. Mover o robô
* Ignore o BlackBoard e considere o segundo bloco (nó) da Sequência Reativa. A Sequência Reativa funciona como uma porta lógica AND. Se todas as entradas forem VERDADEIRAS, a saída da porta AND também será VERDADEIRA.
* Aqui, o robô gira. Isso é VERDADEIRO e, em seguida, o robô usa o laser para escanear o caminho sem obstáculos. Isso será VERDADEIRO se você remover o obstáculo manualmente.
* Após a remoção, a segunda Sequência Reativa se torna VERDADEIRA para que o robô possa executar a última ação “anexada” à primeira Sequência Reativa.

![u0_sim](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u0_sim.png)

# Arquitetura de Software. Árvores de Comportamento (Behavior Trees) - ROS2
Ao projetar um agente autônomo (um robô), considere um baixo nível de abstração.

Pense em motores, acionamentos, sensores, controladores de robô e software. Por fim, seu robô executa com excelência a tarefa programada. Por exemplo, seu manipulador pode pegar algo da mesa e colocá-lo em outro lugar, ou um robô de limpeza pode limpar o chão.

Para simplificar, imagine seu robô executando tarefas de pegar e colocar:

* Primeiro, seu robô captura a posição do objeto a ser pego.
* Em seguida, o robô calcula o caminho curto e sem colisões até o destino (onde o objeto deve ser colocado). Enquanto o caminho sem colisões é calculado, o robô executa a tarefa programada.

Como você pode imaginar, a descrição acima é simples e não inclui muitas outras tarefas que precisam ser avaliadas. Aqui mencionamos a otimização da posição de preensão, estratégias de preensão em caso de falha e planejamento de movimento, onde o modelo dinâmico do robô deve ser considerado. Além disso, pense na supervisão de movimento, segurança e outras tarefas que não sejam relevantes para o seu caso simples.

Como o curso discutirá o framework Behaviour Tree, especialmente no domínio ROS2, você deve entender que o BT não é o único framework dedicado ao ROS. O framework BT pode ser incorporado a qualquer outro software no qual você desenvolva sua aplicação (aqui, em C++). A indústria de jogos é um bom exemplo de como o BT aprimora a experiência do usuário. As possibilidades são enormes.

No curso a seguir, você desenvolverá gradualmente sua competência em BT:

* Por exemplo, nesta unidade, você experimentará o framework BT puro sem ROS (compilará e executará o programa sem ROS).
* Enquanto você compreende os princípios básicos, mostraremos como incorporar a mesma estrutura do programa BT ao ambiente ROS2. 

![bt](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_0a.png)

Como você pode ver, o framework BT pode ser executado **sem ROS**. Neste caso, o BT fornece mecanismos abstratos (construídos em C++) que permitem arquitetar seu BT e imaginar as conexões lógicas dentro de sua aplicação. Por exemplo, suponha que você desenvolva um jogo. Nesse caso, o BT pode estar relacionado ao desempenho do jogo ou definir como os personagens do jogo se comportam em determinadas situações, etc. Nesse contexto, o framework BT fornece os mecanismos para suportar relações lógicas/abstratas entre as classes C++, cujos métodos são chamados de acordo com a estrutura da árvore lógica da sua aplicação.

Na próxima unidade, revisaremos a implementação no ROS2. É simples. Considere a classe C++ como um nó ROS2 que interage com o ambiente ROS2 e seu robô. O ROS2 que executa sua aplicação BT pode incluir classes auxiliares de função em C++ que suportam funções e não são chamadas diretamente da árvore lógica.

Enquanto o robô executa a tarefa, por exemplo, o planejamento da trajetória, considere diversas ações a serem incorporadas ao seu programa. Primeiro, você deve saber a posição e a orientação do robô. Para ter sucesso no cálculo da trajetória, você deve saber a posição do objeto a ser selecionado, o alvo. O cálculo de uma trajetória sem colisões exige a posição dos obstáculos.

Todas essas informações podem ser recebidas/recebidas de outros módulos do sistema robótico do seu programa ou de sensores do robô. Com essas informações, execute o algoritmo de planejamento de trajetória adequado. Pode ser o Dijkstra, A*, RRT ou outro. Para acelerar o processo de busca da trajetória sem colisões, incorpore diferentes heurísticas e execute seus algoritmos de planejamento de trajetória em diferentes unidades-alvo, como CPU, GPU ou FPGA.

Uma descrição mais ou menos detalhada revela que a tarefa de planejamento de trajetória compreende múltiplas atividades de baixo nível (módulos encapsulados de baixo nível).

Especificamente no que diz respeito à robótica, a abstração permitiu que você passasse do controle de tarefas de baixo nível e da detecção fundamental para comportamentos de alto nível.

Ao projetar um sistema robótico, ainda é importante considerar como os componentes fundamentais do robô (ações de baixo nível) executam as tarefas especificadas e buscar otimização (por exemplo, aumentar o desempenho dos algoritmos de caminho adicionando heurísticas).

No entanto, para construir a arquitetura de toda a aplicação do robô, é necessário avançar para um nível mais alto de abstração de programação, ou seja, avaliar a tarefa como no exemplo, o planejador de caminho como um componente simples (considere a figura abaixo).

A BT conta com arquitetos de robôs para orquestrar mapas de tomada de decisão para as aplicações do robô.

![u1_1](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_1.png)

Os blocos BT representados serão formulados posteriormente. No entanto, o símbolo `⟶` (bloco  de **sequência**) é considerado uma operação lógica **E**, já que o símbolo `?` (bloco de **retorno**) define o **OU** lógico. Levando em consideração operadores lógicos, é possível deduzir como o robô se comporta. O robô deve primeiro encontrar, pegar e colocar a bola. As operações de coleta requerem a execução de ações específicas.

Todas as ações sob o bloco **Sequência** devem ser bem-sucedidas para ter sucesso no "ramo de uma árvore". Para o bloco **Retorno**, no entanto, apenas uma ação deve ser bem-sucedida para encerrar (Sucesso) a execução das ações (naquele ramo da árvore). Voltemos à operação de coleta (pick ball). Você pode ver que a coleta (pick ball) **requer duas ações** para ter sucesso (Retornos). Se a bola estiver próxima (approach ball) e a ação de agarrar (grasp ball) a bola for bem-sucedida, a ramificação (ação de coleta (picking ball) é concluída) e a operação de colocação (place ball) podem ser continuadas. A árvore executada será bem-sucedida se o último robô colocar (place) a bola.

Considerando os conjuntos de componentes simples (tarefas), você pode facilmente compor e orquestrar o comportamento da Raiz enquanto interage com o ambiente e executa as tarefas planejadas.

O mapeamento das tarefas do robô na árvore hierárquica de tarefas (BT) oferece uma oportunidade (da perspectiva humana) para a compreensão e modelagem perfeitas de tarefas complexas do robô. Imagine que a construção da BT (mapeamento de tarefas em uma árvore hierárquica) seja considerada programação de abstração de alto nível.

Para arquitetar a BT da aplicação do robô, primeiro especifique o contexto lógico da aplicação e modele conexões consistentes entre ações e comportamentos específicos do robô (detalhes posteriormente). O **contexto lógico** é expresso no arquivo **XML**. A definição de nós, **classes e funções** é definida no framework **BehaviourTree.CPP**. Veja como definir a lógica da BT em XML.

# Conceito de Árvores de Comportamento
Introduzimos um conceito de alto nível de abstração no domínio do software robótico. Portanto, é razoável elaborar sobre a localização dessa abstração. Você pode considerar a seguinte pilha de abstração e retomar suas premissas anteriores.

* Seres humanos permanecem no topo dessa pilha. Os requisitos para a aplicação do robô fluem para baixo. Usando um conjunto herdado de habilidades e conceitos de BT, humanos podem arquitetar o raciocínio lógico da aplicação do robô necessária.
* Humanos usam Árvores de Comportamento para modelar a conexão lógica entre as tarefas do robô. Posteriormente, você aprenderá que o modelo é executado em um arquivo XML. Abaixo da abstração lógica, você se depara com a implementação de BTs.
* Na próxima unidade, você investigará profundamente essa camada e frameworks C++ específicos (BehaviourTree.CPP), permitindo a construção de BTs. A definição das tarefas do robô (função do programa em C++) completa a pilha a seguir.

![u1_5](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_5.png)

Após a configuração e as transições do programa do robô, você pode especificar que o BT organize as transições de agentes autônomos (robôs) entre tarefas (componentes simples).

O BT é formalmente definido como uma árvore raiz direcionada com nós centrais chamados **nós de fluxo de controle** e **nós folhas** chamados **nós de execução**. Você emprega a linguagem usual de pai e filho para cada nó conectado. O nó raiz é o único sem pais; os outros nós têm um pai. **Há pelo menos um filho em cada nó de fluxo de controle**. Os filhos de um nó são mostrados graficamente abaixo.

Agora, defina o primeiro bloco BT: **Sequência**.

![](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_0.png)

Observe o fluxo de tick e callback e estude os seguintes diagramas:

![u1_4](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_4.png)

O XML: descrevendo o comportamento do robô (nó Sequência) pode ser formulado. 

```xml
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <RobotTask1   name="task1"/>
            <RobotTask2   name="task2"/>
            <RobotTask3   name="task3"/>
        </Sequence>
    </BehaviorTree>
</root>
```

* Imagine que o robô executa três tarefas: encontrar a bola, pegar a bola e colocar a bola – chamadas task1, task2 e task3, respectivamente.
* Aqui, as tarefas devem concluir com sucesso a tarefa "principal".
* Neste exemplo, apenas a primeira tarefa é BEM-SUCEDIDA e imprime RobotTask1: task1.
* A segunda tarefa é FALHA (não imprime texto) e a Sequência é FALHA.

Para obter sucesso na Sequência, todos os nós devem retornar SUCESSO.

* Uma BT começa com o nó Raiz, que fornece sinais que permitem a execução de um nó chamado ticks com uma frequência específica, que é enviada aos seus filhos. Se, e somente se, um nó receber ticks, ele será executado. Se a execução estiver em andamento, o nó filho retornará instantaneamente Running para o pai, Success se o objetivo for atingido e Failure, caso contrário.
* Existem quatro nós de fluxo de controle na formulação clássica (Sequência, Fallback, Paralelo e Decorador) e dois tipos de nós de execução (Ação e Condição).
* O nó Sequência executa um algoritmo equivalente ao roteamento de ticks para seus filhos da esquerda até encontrar um filho que retorne Failure ou Running, e então retorna Failure ou Running para seu pai. Ele retorna Success somente se todos os seus filhos também retornarem Success. Deve-se observar que, quando um filho retorna Running ou Failure, o nó Sequência não encaminha os ticks para o próximo filho (se houver). Para simplificar, o nó Sequência pode ser considerado uma função lógica AND.

```python
#Do not run the cell

BT::NodeStatus RobotTask1::tick()
{
    std::cout << "RobotTask1: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTask2::tick()
{
    std::cout << "RobotTask2: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTask3::tick()
{
    std::cout << "RobotTask3: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
```

Abaixo, veja uma definição do próximo bloco BT: **Fallback**.

![u1_3](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_3.png)

O nó Fallback conduz um algoritmo que envolve o envio de ticks para seus filhos da esquerda até localizar um filho que retorne Sucesso ou Em Execução. Em seguida, ele envia Sucesso ou Em Execução para seu pai de acordo com essa descoberta. Se, e somente se, todos os seus filhos também retornarem Falha, ele retornará Falha. Observe que, quando um filho retorna Em Execução ou Sucesso, o nó Fallback não transmite os ticks para o filho subsequente (se houver), seja Em Execução ou Sucesso.

Seguindo essa abordagem simples, considere o nó Fallback uma função lógica OU.

Observe atentamente o fluxo de ticks e retornos de chamada. Estude os seguintes diagramas:

![u1_6](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_6.png)

O XML que descreve o comportamento do robô (nó de fallback) pode ser formulado da seguinte forma:

```xml
 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Fallback name="root_sequence">
            <RobotTask1   name="task1"/>
            <RobotTask2   name="task2"/>
            <RobotTask3   name="task3"/>
        </Fallback>
     </BehaviorTree>

 </root>
```

* Neste exemplo, imagine que o robô precisa executar três tarefas: task1, task2 e task3.
* Aqui, apenas uma tarefa precisa ser SUCESSO para retornar SUCESSO do nó de fallback (esta é uma função lógica OU).
* Neste exemplo, o primeiro nó retorna FALHA e imprime RobotTask1: task1.
* Se task2 for SUCESSO, o nó imprime RobotTask2: task2, e o fallback é SUCESSO. (O robô não executa task3).


source /home/simulations/ros2_sims_ws/install/setup.bashsource ~/ros2_ws/install/setup.bash