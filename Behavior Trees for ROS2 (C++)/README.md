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

## Arquitetura de Software. Árvores de Comportamento (Behavior Trees) - ROS2
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

## Conceito de Árvores de Comportamento
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
* Existem quatro nós de fluxo de controle na formulação clássica (**Sequência, Fallback, Paralelo e Decorador (Decorator)**) e dois tipos de nós de execução (Ação e Condição).
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
}
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

## BT em Ação
Aqui, considere uma BT simples e entenda o **fluxo de sinais**. Como você deve se lembrar, um sinal de "**tick**" é transmitido da **raiz da árvore e percorre a árvore até atingir um nó folha**.

O **retorno** de chamada de um **TreeNode** é executado quando recebe um sinal de tick. Esse retorno de chamada tem três resultados possíveis: a **atividade (Activity)** é **SUCESSO (SUCCESS)**, **FALHA (FAILURE)** ou **AINDA EM EXECUÇÃO (still RUNNING)**.

O exemplo a seguir exibe um "recurso" da BT ainda não discutido neste curso. O **framework** BehaviourTree.CPP será amplamente discutido na próxima unidade. Primeiramente, porém, é importante mencionar que a ação "comer banana" retorna um retorno de chamada: EM EXECUÇÃO, que, neste caso, reflete o processo de comer banana e requer tempo. Portanto, chamamos essa ação de **Ação Assíncrona**. Geralmente, essa funcionalidade da BT é alcançada implementando um **nó assíncrono em execução em uma thread separada**.

![u1_2](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_2.png)

O XML que descreve a lógica do BT descrito acima pode ser formulado.

```xml
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <ReactiveFallback name="root">
            <EatSandwich name="eat_sandwich"/>
            <EatApple name="eat_apple"/>
            <Sequence>
                <OpenBanana name="open_banana"/>
                <EatBanana       goal="1;2;3"/>
                <SaySomething   message="banan is gone!" />
            </Sequence>
        </ReactiveFallback>
     </BehaviorTree>
 </root>
```

Derivando essa filosofia da BT, você pode descrever o funcionamento da BT considerando o seguinte exemplo:

* A BT consiste em quatro ações, Fallback e Sequência.
* Comece com uma Raiz que envia o tick para esse Fallback. Siga a regra do lado esquerdo. O Fallback envia um tick para "sanduíche" - você tem um sanduíche para comer?. Não. Portanto, o callback retorna Falha.
* Seguindo a lógica do Fallback, o próximo tick é enviado para maçã. Você tem uma maçã para comer? Não. Como anteriormente, o callback retorna Falha. O Fallback não pode enviar o callback positivo para a Raiz, então ele continua.
* Agora, o tick é enviado para a Sequência. A Sequência envia um tick para abrir uma banana e recebe Sucesso (a banana está aberta), mas a Sequência é uma operação lógica AND, então você continua.
* Em seguida, o tick é enviado para comer uma banana. Como o processo leva tempo, o callback para a Raiz é **Executando**.

## Notação de nó BT
Você retornará ao exemplo no curso, mas para completar a discussão sobre o conceito de BT, defina **nós "auxiliares"**:

Um comando é executado por um **nó de ação (action node)** sempre que recebe um tick. Se a ação for executada com sucesso, ele retorna Sucesso; caso contrário, retorna Falha. Por fim, a **ação retorna Em Execução enquanto ainda está em andamento**.

Um **nó de Condição (condition node)** avalia uma proposição sempre que recebe um tick. Então, dependendo se a proposição é verdadeira, ele retorna Sucesso ou Falha. Lembre-se de que **um nó de Condição nunca retorna o status Em execução**.

![u1_10](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_10.png)

O **nó Decorador** é um nó de **fluxo de controle** com um único filho que marca o filho seletivamente de acordo com algumas regras estabelecidas e modifica o status de retorno do filho de acordo com uma regra definida pelo usuário. Por exemplo:

* um **decorador invertido** inverte o status Sucesso/Falha do filho. 
* Um **decorador com máximo de N tentativas** permite que o filho falhe apenas N vezes antes de retornar Falha sem marcar o filho.
* um **decorador com máximo de T segundos** permite que o filho seja executado por T segundos antes de retornar Falha sem marcar o filho.

A tabela a seguir condensa e define o raciocínio lógico dos nós BT.

![u1_11](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_11.png)

## Abordagem para BT
A seção anterior definiu os **nós de comportamento lógico** comuns usados durante a arquitetura de BT para aplicações de robôs (tarefas). Agora, tente entender o principal motivo pelo qual a BT está ganhando popularidade nos domínios da robótica, jogos e inteligência artificial.

Embora as BTs tenham ganhado popularidade nos últimos anos, as **Máquinas de Estados Finitos** (FSMs) ainda são um dos paradigmas mais conhecidos para definir o comportamento de robôs ou agentes de software.

Para ser consistente, lembre-se do conceito simples de uma máquina de estados finitos. Uma FSM é baseada em uma máquina abstrativa (virtual) com um ou mais estados. A máquina alterna entre estados para executar várias tarefas, pois apenas um estado pode estar ativo.

Ao implementar algoritmos, robótica ou jogos, as FSMs são frequentemente usadas para estruturar e descrever um fluxo de execução. Uma FSM, por exemplo, pode ser usada para criar o "cérebro" do robô. Geralmente, a FSM ajuda a arquitetar a aplicação do robô a partir de um nível de abstração mais alto. Cada estado corresponde às ações do robô (considere a figura abaixo), o que permite a execução de ações dedicadas para esse estado. Você também pode afirmar que o robô é observável, o que significa que, em cada registro de tempo, você conhece o estado do robô. O robô que segue o fluxo da FSM é previsível, em relação às ações que o robô executa e às transições que ele é capaz de realizar.

Um grafo pode representar FSMs, com os nós denotando os estados e as arestas denotando as transições. Cada aresta possui um rótulo que indica quando a transição deve ocorrer. Por exemplo, considerando a figura, você pode diferenciar vários estados do robô, como se aproximar da bola, agarrar a bola ou esperar por ajuda.

![u1_8](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_8.png)

### Desafios com FSMs, transição para BTs
Inúmeros problemas impactam as FSMs e surgem em situações do mundo real se houver transições suficientes entre estados e condições:

* Pode haver NxN transições de estado em uma FSM com N estados.
* Como o contexto de execução afeta o número de estados, N pode aumentar rapidamente.
* Todos os outros estados que transitaram para o estado novo ou antigo devem alterar suas condições quando um novo estado é adicionado ou removido.
* Os estados são fortemente conectados, o que impacta como eles podem ser reutilizados.
* As representações gráficas e verbais do comportamento completo tornam-se muito complexas para o projetista compreender quando o número de estados é suficientemente grande.

Não são os computadores, mas principalmente as pessoas, que são afetadas por esses problemas. Por exemplo, uma máquina de estados enorme pode ser facilmente manipulada por um interpretador de software, mas uma pessoa, sem dúvida, terá dificuldade em compreender e prever o comportamento geral do sistema.

Em outras palavras, o problema fundamental com as FSMs é que elas logo se tornam incontroláveis à medida que o número de estados aumenta devido à carga cognitiva enfrentada pelos desenvolvedores, principalmente os projetistas de comportamento.

A maioria desses problemas é resolvida pelas BTs, que aumentam a modularidade e podem ser reutilizadas. Elas são, por natureza, hierárquicas.

* Qualquer subárvore fornece uma ação possivelmente reutilizável de uma perspectiva semântica.
* O desenvolvedor pode utilizar e expandir a linguagem fornecida pela BT para aplicar padrões de projeto populares.
* O fato de a hierarquia ser ordenada de cima para baixo e a precedência dos nós da esquerda para a direita torna as representações textuais e gráficas mais fáceis de serem "lidas" por um humano.
* A BT geralmente usa Ações em vez de Estados (como em FSMs); essa estratégia está mais alinhada com o "modelo conceitual" usado para definir o comportamento e as interfaces de software fornecidas por arquiteturas orientadas a serviços.

As figuras abaixo mostram como o mesmo exemplo pode ser mais facilmente utilizado pelas BTs.

Os nós onde a Detecção de Obstáculos é implementada podem ser um **Decorator Node**, que executará seu nó filho somente se a condição interna for satisfeita, ou um **Condition Node**, que é uma folha da árvore que pode retornar SUCCESS ou FAILURE.

![u1_9](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_9.png)

## Principais vantagens do BT
Conforme discutido anteriormente, inúmeros benefícios resultam do fato de os BTs serem **modulares e reativos**. Listamos alguns benefícios dos BTs abaixo.

1. **Modular**: Um sistema modular refere-se à facilidade com que suas partes podem ser desmontadas em unidades menores e, em seguida, remontadas.

Um sistema modular pode ser criado, implementado, testado e reutilizado, um módulo de cada vez. As vantagens da modularidade, portanto, aumentam com a complexidade do sistema, permitindo uma estratégia de dividir para conquistar durante o projeto, a implementação e os testes.

Como cada subárvore de um BT pode ser percebida como um módulo, os BTs são modulares em todos os tamanhos, desde as subárvores mais altas até as folhas da árvore.

2. **Organização hierárquica**: Uma arquitetura de controle é hierárquica se tiver muitas camadas de tomada de decisão (seguindo a estrutura da árvore).

3. **Código reutilizável**: Ter código reutilizável é muito importante em qualquer projeto grande, complexo e de longo prazo. A capacidade de reutilizar projetos depende da construção de coisas maiores a partir de partes menores e da independência da entrada e saída dessas partes. Para permitir a reutilização de código, cada módulo deve interagir com a arquitetura de controle de forma clara e bem definida.

Os BTs permitem código reutilizável, pois, dada a implementação adequada, qualquer subárvore pode ser reutilizada em vários locais de um BT. Além disso, ao escrever o código de um nó folha, o desenvolvedor precisa retornar o status de retorno correto, que é universalmente predefinido como **Em Execução, Sucesso ou Falha**.

4. **Reatividade**: Por reatividade, entendemos a capacidade de responder a mudanças de forma oportuna e eficaz. Planos que foram gerados offline e posteriormente executados em um loop aberto são frequentemente propensos a falhas em ambientes não estruturados, onde os resultados das ações são incertos e atores externos modificam continuamente o estado do mundo. Os BTs são reativos porque um loop fechado é executado devido à criação contínua de ticks e à travessia da árvore. De acordo com a travessia dos ticks, que se baseia nos status de retorno dos nós-folha, as ações são executadas e canceladas. O ambiente está intimamente ligado aos nós-folha (por exemplo, os **nós de condição** avaliam as **qualidades gerais do sistema** (sensores completos). Em contraste, os **nós de ação** relatam **Falha/Sucesso** se a ação foi bem-sucedida ou não). BTs são, portanto, muito sensíveis a mudanças no ambiente.

5. **Legível por humanos**: Quando um job é criado por humanos, é essencial ter uma estrutura legível para reduzir o custo de desenvolvimento e solução de problemas. A estrutura precisa continuar sendo compreensível, mesmo para sistemas grandes. A legibilidade humana requer uma estrutura coerente e compacta.

Devido à sua modularidade e estrutura em árvore, os BTs são legíveis por humanos.

## EXEMPLO
Agora, execute a simulação no framework ROS2. O BT para a simulação pode ser representado da seguinte forma:

![u1_sim](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u1_sim.png)

* Agora, imagine que o robô parou e sua missão é virar à esquerda.
* O robô está equipado com um laser que varre o ambiente e detecta obstáculos.
* Primeiramente, o robô recebe a informação para virar 45 graus. O robô pode continuar sua viagem nessa direção se houver um caminho sem colisões. Caso contrário, o robô gira 90 graus e verifica novamente o obstáculo.
* Se houver uma colisão (detectada pelo laser), o robô gira para os 135 graus finais e se move nessa direção, já que não há colisão.

Esta simulação exibe o comportamento do nó **Fallback**. O robô continua verificando as opções de movimento se o retorno for **FAILURE**. Uma vez que for SUCCESS, todo o nó será SUCCESS.

Conforme descrito acima, o comportamento do robô é o padrão. Se você remover um obstáculo (por exemplo, na direção de 90 graus), o robô se moverá nessa direção.

## Arquitetura de Árvores de Comportamento
Na unidade anterior, introduzimos o conceito de BT e definimos primitivas lógicas para arquitetar o comportamento de agentes autônomos (um robô). Agora você entende a pilha de abstração e pode localizar BT nela.

Embora as BTs sejam razoavelmente simples de compreender e usar, é benéfico conhecer os princípios e métodos de design que podem ser usados em diversos contextos para atingir seu potencial máximo. Vários exemplos serão usados para ilustrar essas ideias na unidade seguinte.

Especificamos a interpretação lógica de como o robô se comporta durante a execução da tarefa, conforme refletido no arquivo XML.

Neste curso, você utilizará o framework BehaviourTree.CPP, que habilita e protege as tarefas do robô que serão executadas de acordo com as definições lógicas (escritas em XML). Isso significa que o robô percorre a BT adequadamente.

A pilha a seguir ilustra as relações e ações a serem executadas usando a BT para modelar a tarefa do robô. O diagrama fornece uma visão geral do fluxo de dados durante a execução da aplicação do robô incorporada à BT.

![u2_1](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u2_1.png)

Imagine que seu robô precisa executar uma tarefa (por exemplo, limpar o chão). A tarefa consiste em diversas ações, frequentemente relacionadas à interação com o ambiente do robô. A aplicação do robô frequentemente inclui mecanismos de supervisão de segurança e outras "verificações" (verificação da bateria). As ações do robô e os mecanismos auxiliares são modelados em BT, onde as relações lógicas são expostas no arquivo XML.

Usando o framework BehaviourTree.CPP;
1. você cria a declaração dos nós (seguindo a arquitetura XML; BT) e as definições. 
2. Em seguida, criando as definições dos nós, você expressa as ações executadas neste nó BT.

Um conjunto de ações definido por um nó BT é considerado um retorno de chamada (BT chama outras funções/ações). Neste curso, você define um conjunto de ações executadas no nó ROS. Além disso, o nó ROS se comunica com um robô ou simulação. Os nós ROS recebem o feedback e enviam o estado (feedback) para o nó BT. A análise de feedback afeta o nó BT e envia o retorno para a Raiz (SUCESSO, FALHA ou EXECUÇÃO).

Tendo em mente a pilha de abstração definida anteriormente, você pode avaliar novamente o BT, que define o fluxo lógico. O projeto detalhado dos nós ROS é desvinculado do conceito de BT.

Você não considera em detalhes como o nó executa a tarefa (por exemplo, calcula a cinemática inversa). No entanto, você considera apenas o comportamento do nó, ou seja, a funcionalidade do nó (como uma "caixa de blocos"). Ao arquitetar o BT, você sabe o que o nó ROS está fazendo, mas os detalhes da implementação não são obrigatórios nesse raciocínio abstrato.

Nas próximas seções, discutiremos as vantagens de utilizar a estrutura de arquivo XML explícita e a abordagem para modelar a estrutura lógica do BT. Além disso, você experimentará o mapeamento do XML no framework BehaviourTree.CPP e no ROS2.

Você terá uma visão mais aprofundada do BT e dos mecanismos que orquestram as definições lógicas das tarefas do robô. Referindo-se às suas premissas anteriores, ao arquitetar o BT, considere apenas o nível mais alto de abstração, evitando uma compreensão mais profunda dos mecanismos relacionados à implementação do nó ROS. O nó, como mencionado, é considerado uma "caixa preta" - executando uma função no sistema robótico que você constrói.

Considere o diagrama abaixo representando as conexões lógicas no BT. Comece pela função primária, que lê o arquivo XML.

A função primária e a estrutura do BT incluem a declaração dos nós. A estrutura BehaviourTree.CPP gerencia o fluxo lógico (marque o nó BT) de acordo com uma especificação XML (invoca um retorno de chamada).

Os nós BT executam seu retorno de chamada (conjunto de ações). O retorno de chamada retorna SUCESSO, FALHA ou EXECUÇÃO. A ação de retorno de chamada pode se comunicar com os nós ROS. Os nós ROS executam ações relacionadas ao robô.

![u2_2](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u2_2.png)

## Tipos de Nós e Comportamento. O Modelo de Fluxo Lógico BT em Formato XML
Enfrente os desafios do design conceitual da BT, que por natureza deve ser legível por humanos, introduzindo o conceito XML. O BehaviorTree.CPP oferece uma linguagem de script XML que permite que humanos especifiquem árvores específicas e nós únicos.

Para arquitetar a BT de cada tarefa do robô, você definiu brevemente as primitivas (nós) da BT na unidade anterior. Os nós Sequência, FallBack e Decoradores foram descritos. Isso ajudou você a entender a BT em geral. No entanto, você deve adicionar mecanismos e abstrações lógicas para ser mais consistente e aderir ao conceito do framework BehaviorTree.CPP.

Para fornecer uma compreensão completa da arquitetura da BT, veja abaixo uma variedade de técnicas e abstrações de framework na seção seguinte. O curso seguirá os princípios orientadores da abordagem de design XML. No entanto, você se concentrará principalmente em uma abordagem prática (usando exemplos em C++).

![u2_3](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u2_3.png)

Primeiro, considere o nó de sequência. O último diagrama descreve três tipos de nós de sequência. Na tabela a seguir, você pode entender o comportamento de cada um deles.

![u2_4](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u2_4.png)

Na tabela, **Reiniciar** e **Marcar novamente** podem ser compreendidos da seguinte forma:

* Reiniciar significa que todo o nó da sequência é reiniciado a partir do primeiro filho da lista; Se o nó da sequência incluir os filhos A, B, C e D, então A e B são SUCESSO, no entanto, C é FALHA. A próxima marcação força você a verificar novamente, começando de A.
* Marcar novamente significa que na próxima vez que a sequência for marcada, o mesmo filho será marcado. Os filhos anteriores lembram o status; Se o nó da sequência incluir os filhos A, B, C e D, então A e B são SUCESSO, no entanto, C é FALHA. A próxima marcação força você a verificar novamente C, e não A e B nos quais os estados são lembrados.

Reveja os exemplos simples, as definições lógicas em XML e os exemplos em C++. O framework BehaviorTree.CPP será estudado cuidadosamente na próxima unidade.

![u2_5](https://github.com/marcospontoexe/ROS_2/blob/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)/imagens/u2_5.png)

A sequência a seguir é simples.

* O robô se move em linha reta e para a frente, lendo o sensor laser. As leituras são precisas e fornecem uma excelente visão geral do entorno.
* O robô analisa os dados do sensor e percebe as condições para evitar obstáculos que possam aparecer nas proximidades do robô (enquanto se move em linha reta). Se o obstáculo estiver a menos de dois metros do robô (CONDIÇÃO 1), o robô precisa parar.
* O robô deve analisar novamente o ambiente e decidir como evitar o obstáculo (neste caso, suponha que o robô possa virar à esquerda ou à direita; o robô escolhe a direção aleatoriamente). O robô verifica se há energia suficiente armazenada na bateria para girar (CONDIÇÃO 2) antes de girar (à esquerda ou à direita). Em caso afirmativo, o robô gira 90 graus (AÇÃO 1) e se move em linha reta (AÇÃO 2).

Agora, você pode arquitetar a BT para essas ações do robô.

```xml
<root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Obstacle   name=" Obstacle detected"/>
            <EnoughBattery   name=" Battery OK"/>
            <Rotate       name=" Rotated"/>
            <MoveStraight name=" Moved"/>
        </Sequence>
    </BehaviorTree>
</root>
```

Verifique a tabela acima. Se a CONDIÇÃO 1 for FALHA (sem obstáculo), a sequência será encerrada e o nó Sequência será reiniciado.

Considere o caso em que o robô precisa de cinco segundos para girar antes que o retorno de chamada retorne **EM EXECUÇÃO**. Nesse caso, a ação, rotação, receberá o tick subsequente e determinará se o estado mudou de **EM EXECUÇÃO** para **SUCESSO**. O nó Sequência se lembra das verificações anteriores (CONDIÇÃO 1 e CONDIÇÃO 2) e não há necessidade de verificá-las novamente.

Além disso, você descreveu exemplos na unidade anterior e agora é uma ótima oportunidade para entender o design XML. Como discutido, o XML é usado para modelar o fluxo lógico em BT. Neste exemplo, você pode arquitetar o nó Sequência da seguinte forma.

Novamente, é crucial enfatizar que você projeta as conexões lógicas e o fluxo das ações aqui. Cada ação descrita nesta unidade possui um retorno de chamada (do ponto de vista do site raiz), o que implica que ela possui rotinas específicas para execução.

Para simplificar, se o robô verificar a bateria, ele receberá os resultados SUCESSO (energia suficiente) ou FALHA (bateria vazia) do retorno de chamada. Em um sistema real, verificar a bateria é significativamente mais difícil e envolve mais etapas do que apenas verificar.



source /home/simulations/ros2_sims_ws/install/setup.bashsource ~/ros2_ws/install/setup.bash