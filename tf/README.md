# tf (Transform Library) 
O TF é essencial para rastrear as posições e orientações de diferentes quadros de coordenadas ao longo do tempo, permitindo que os robôs entendam suas relações espaciais.

Suponha que você queira comandar um robô para se mover perto da grande rocha.

Para um humano, isso é fácil. Mas, em robótica, é necessário ser preciso e fornecer a posição exata usando valores numéricos.

Para fornecer a posição da rocha usando valores precisos, você precisa de um **ponto de referência fixo** de onde possa determinar suas coordenadas de posição. 

# Sistemas de Coordenadas, Referenciais e Quadros de Coordenadas
Um sistema de coordenadas é um conjunto de eixos ortogonais que se interseccionam em um ponto denominado **origem**, que descreve a posição dos pontos em relação a essa origem. Observe que o termo sistema de coordenadas pode ser usado indistintamente com sistema de referência ou sistema de coordenadas, mas use este último termo neste curso, pois ele se tornou mais amplamente utilizado pelo ROS. Na verdade, no ROS, você normalmente vê apenas o termo sistema TF, que é uma abreviação de **sistema de transformação**.

# Quadro de Coordenadas Global
Este é um quadro arbitrário que você coloca no ambiente em relação ao qual quaisquer pontos de interesse no ambiente podem ser localizados.

Nos exemplos mencionados, este quadro de coordenadas global é baseado na localização inicial do robô.

No entanto, ter mais de um quadro de coordenadas pode ser conveniente ou necessário na maioria das vezes. É aqui que o conceito de quadros de coordenadas locais entra em jogo.

# Quadro de Coordenadas Local e nomes de quadros de coordenadas
Para facilitar, crie uma nova coordenada local para referenciar facilmente um ponto ou objeto na cena.

Por exemplo, se você quiser saber onde a cabeça está em relação ao pescoço do robô, defina DOIS QUADROS DE COORDENADAS:

* O quadro de pescoço serve como o Quadro de Coordenadas LOCAL.
* O quadro de cabeça define onde a cabeça está, conhecido como quadro de coordenadas da cabeça.

Isso nos leva a outro conceito-chave para quadros de coordenadas: pode haver vários quadros de coordenadas, e cada quadro de coordenadas deve ter um nome exclusivo que o descreva e o distinga de outros quadros de coordenadas. Mais adiante nesta unidade, você verá que, no ROS, é comum usar o rótulo **frame_id** para especificar o nome de um quadro de coordenadas.

Agora, cada vez que você criar um novo quadro de coordenadas, especifique sua localização em relação a outro quadro de coordenadas. Sempre deve existir um relacionamento entre os dois quadros. É isso que a chamada **transformação** faz por você.

Nos exemplos mencionados, a seta rosa representa essa transformação.

![frame](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/headneckframes2.png)

Agora, como cada dois quadros de coordenadas estão relacionados por uma transformação única, você pode representar os quadros de coordenadas em seu sistema como uma hierarquia ou árvore.

# Convenções

## Quadro de coordenadas Left-handed vs right-handed
Uma das convenções mais importantes em ROS é que os quadros de coordenadas seguem a **regra da mão direita**. Isso significa que o eixo X aponta para a **frente**, o eixo Y para a **esquerda** e o eixo Z está sempre para **cima**. Observe que a regra da mão direita é padrão em física e engenharia, mas existem algumas exceções notáveis, como o sistema de coordenadas usado para definir a localização de um pixel em uma imagem ou tela.

Outro padrão amplamente adotado em ROS usa **vermelho**, **verde** e **azul** para colorir os eixos **X, Y e Z**, respectivamente.

Dos fatos mencionados acima, conclui-se que um robô ROS terá, por convenção, seu quadro de coordenadas do corpo principal escolhido de forma que o eixo X (em vermelho) aponte para a frente, o eixo Y (em verde) aponte para a esquerda e o eixo Z (em azul) aponte para cima.

![robot](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/tf_convention_mobile_base.png)

Todas as ferramentas ROS2 aderem automaticamente a essa convenção de sistema de coordenadas, portanto, normalmente, não há muito o que fazer além de estar ciente dela. No entanto, se você escrever um nó de direção, como um nó de acionamento diferencial, deverá garantir que uma taxa de **giro positiva** faça com que seu robô vire **à esquerda**. Caso contrário, você não estará em conformidade com o padrão. Seguir essa convenção é importante se você deseja integrar com outros componentes ROS e reutilizar softwares como o Navigation2.

A mesma convenção se aplica a outros tipos de robôs, como robôs semelhantes a carros e drones.

Abaixo, você encontra links para recursos para saber mais sobre as convenções do ROS que podem lhe interessar:

*[Standard Units of Measure and Coordinate Conventions (ROS.org)](https://www.ros.org/reps/rep-0103.html)

*[REP 105 -- Coordinate Frames for Mobile Platforms (ROS.org)](https://www.ros.org/reps/rep-0105.html)

*[REP 120 -- Coordinate Frames for Humanoid Robots (ROS.org)](https://www.ros.org/reps/rep-0120.html)

*[Coordinate Frames for Serial Industrial Manipulators (ROS.org)](https://gavanderhoorn.github.io/rep/rep-0199.html)

*[REP 147 -- A Standard interface for Aerial Vehicles (ROS.org)](https://ros.org/reps/rep-0147.html)

# Criando um Quadro de Coordenadas
Primeiro, **coordene os quadros** no ROS2 para criar um quadro de coordenadas. No ROS, existem várias maneiras de criar um quadro de coordenadas. Primeiro, consulte a linha de comando **static_transform_publisher** incluída no pacote tf2_ros.

A estrutura típica de comando para criar um novo quadro de coordenadas é semelhante a esta: `ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id`.

* **ros2 run tf2_ros static_transform_publisher**: Esta é a sintaxe típica usada para executar um nó ROS2. Como de costume, especifique o nome do pacote, neste caso, tf2_ros, e o nome do nó, neste caso, static_transform_publisher.

* **X Y Z yaw pitch roll**: Como cada quadro de coordenadas tem 6DOF (six degrees of freedom - seis graus de liberdade), forneça informações para os três graus de liberdade rotacionais (em graus).

* **frame_id child_frame_id**: Defina o frame_id (**pai**) e o child_frame_id da transformação.

Da seção anterior, lembre-se de algumas coisas:

* Ao criar um novo sistema de coordenadas, especifique sua localização em relação a outro sistema de coordenadas. **Sempre existe uma relação entre dois sistemas de coordenadas**.

* Você precisa dos **sistemas de coordenadas** e das **transformações entre eles**. Transformações são relações entre conjuntos de sistemas de coordenadas. Usando essas transformações, você pode expressar a posição de um objeto em um sistema de **referência/coordenadas** diferente.

* Você deve especificar um **nome exclusivo** para identificar cada sistema de coordenadas.

# Visualize transformações e quadros de coordenadas no RVIZ
RVIZ é uma interface gráfica que permite visualizar diversas informações presentes no sistema ROS2. Ela também fornece elementos para visualizar os quadros de coordenadas de posição e orientação. 

Para exibir dados de TF no RVIZ2, adicione um novo elemento de visualização chamado "TF". Para isso, clique no botão Adicionar no canto inferior esquerdo, role a lista um pouco para baixo e selecione **TF**. Em seguida, pressione "Ok".

Ao expandir a exibição do TF, você pode ver diversas opções configuráveis. Observe-as individualmente:

* Show Names:: Habilita/desabilita a visualização 3D do nome dos quadros de coordenadas.
* Show Axes: Habilita/desabilita a visualização 3D dos eixos dos quadros.
* Show Arrows: Habilita/desabilita a visualização 3D das setas que representam a transformação do filho para o pai.
* Marker Scale: Insira um valor diferente para redimensionar o tamanho dos eixos, setas e nomes do TF.
* Update Interval: O intervalo, em segundos, para atualizar as transformações dos quadros. Deixe em 0 para ver cada atualização.
* Frame Timeout: O tempo, em segundos, antes que um quadro que não foi atualizado seja considerado morto. Durante um terço desse tempo, o quadro parece correto. Durante o segundo terço, ele fica cinza e, em seguida, desaparece completamente. Isso não acontece se o Quadro estiver ESTÁTICO; ele permanecerá assim para sempre.
* Frames: Você pode ativar/desativar a visualização de quadros individuais marcando a caixa ao lado do nome do quadro. Clique em qualquer nome de quadro para obter informações mais detalhadas.
* Tree: Exibe toda a árvore TF, com todos os quadros de coordenadas disponíveis e sua relação pai-filho.

1. Crie seu primeiro quadro de coordenadas.
2. Crie um novo quadro de coordenadas publicando uma transformação de um quadro de coordenadas existente para este novo quadro de coordenadas.
3. Este novo quadro representa a localização da rocha; portanto, você o chama de rocha.
4. Minimize a janela da Ferramenta Gráfica e execute o seguinte comando: `ros2 run tf2_ros static_transform_publisher --x 0 --y 4.0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id deepmind_robot1_odom --child-frame-id rock`.
5. Publique uma transformação estática do quadro deepmind_robot1_odom existente para este novo quadro rock_frame.
6. Transformações estáticas são usadas para objetos que não se movem. Neste caso, a rocha é o objeto mais estático que existe.
7. As transformações são sempre publicadas dentro dos tópicos /tf e /tf_static do ROS2.
8. Retorne ao RVIZ2 clicando no ícone Ferramentas Gráficas na barra de ferramentas inferior.
9. Agora você deverá ver que um novo quadro apareceu.
10. Se você ativar os nomes, verá que esse novo quadro se chama rocha.
11. Há uma seta conectando o quadro rocha a deepmind_robot1_odom.
12. Essa é a transformação estática que você executou no terminal.
13. Agora, mova o robô para ver como todas essas transformações são atualizadas e verifique se a estrutura da rocha não se move.

# Os vários quadros de coordenadas de um robô
Ao trabalhar com robôs, é criado um **modelo de robô** para representar a estrutura do robô. Use arquivos **URDF** e **XACRO** no ROS para criar esses modelos de robô. Para trabalhar com transformações (tf), saiba que cada junta do robô possui um sistema de coordenadas associado. Dessa forma, você pode acompanhar facilmente a posição dos links do robô no espaço.

# Ferramentas e Visualização TF
Com base nos conceitos básicos de TF da unidade anterior, esta unidade apresentará as ferramentas e técnicas disponíveis no ROS 2 para introspecção e interação com a biblioteca TF2. Entender como visualizar e depurar quadros TF é crucial para trabalhar com sistemas robóticos complexos.

Para lhe ensinar as ferramentas mais importantes para introspecção e interação com a biblioteca TF2 no ROS2, você trabalhará com esta simulação:

![unit2_humblesim](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/unit2_humblesim.png)

Esta simulação possui dois robôs diferentes:

* Cam_bot
* Tartaruga

## CAM_BOT
Este robô tem 6 graus de liberdade, o que significa que pode se mover nos eixos X, Y e Z e girar nos três eixos.

Ele tem uma câmera que você usa para gravar o que quiser, e é controlado de uma maneira especial: ele segue qualquer estrutura do robô que você determinar (que nesse exemplo será a tartaruga).

## TARTARUGA
Este robô se comporta como o clássico **turtle_sim** do ROS, usado em muitos tutoriais de ROS. A única diferença é que se trata de um robô 3D. Portanto, ele se move como qualquer robô com acionamento diferencial se moveria (para frente/trás e gira, mas não para os lados).

### Onde está a tartaruga?
Prepare o cenário: O Cam_bot tem uma câmera montada na parte frontal do robô. O robô pode se mover para qualquer lugar no espaço e em qualquer orientação. Perfeito! Agora, você quer dizer ao Cam_bot para seguir a tartaruga. No entanto, logo, observe que existem algumas dificuldades com as quais você terá que lidar, por exemplo:

* Onde está a tartaruga?
* Como você pode colocar o Cam_bot em uma posição fixa em relação à tartaruga?
* Como você pode calcular os movimentos de translação e rotação que o Cam_bot deve fazer quando a tartaruga se move?

Como você pode ver, muitos problemas surgem dessa cena simples.

Informar ao Cam_bot ONDE (posição e orientação) sua câmera está montada e a tartaruga está localizada em relação ao Cam_bot ou a um ponto global comum.

Todos esses itens exigem a aplicação do mesmo conceito: **transformações entre quadros de coordenadas**.

Uma transformação especifica como **os dados expressos em um quadro de coordenadas podem ser transformados em um quadro de coordenadas diferente**. Por exemplo, se você detectar uma tartaruga com a câmera, ainda não saberá onde ela está (em relação ao mundo, que tem tf fixa e é usado como referencia para as outras TFs (estáticas)).

A câmera apenas informa que vê uma tartaruga, não onde ela está no espaço. Para informar isso, você precisa:

* A posição do Cam_bot no espaço
* A distância do centro do corpo do Cam_bot (comumente chamado de base_link) até o sensor da câmera
* E, finalmente, a distância do sensor da câmera até a tartaruga.

Você precisa dos **quadros de coordenadas** e das **transformações entre eles**.

Qual é a posição do Cam_bot no espaço? Para obter um valor útil para programação robótica, acompanhe suas posições no espaço. Para uma medição útil, você precisa de um ponto de referência fixo (mundo) para avaliar sua posição e orientação. É por isso que você precisa de um quadro de coordenadas antes de descrever qualquer objeto na cena.

Mas como trabalhar com **quadros de coordenadas** no ROS? Quais são as ferramentas que o ROS oferece para isso? O restante desta unidade tem como objetivo delinear ferramentas e bibliotecas úteis que podem ser usadas para visualizar e depurar problemas de TF.

## Ver quadros TF em formato PDF
O ROS 2 permite visualizar os quadros TF de um robô em formato PDF usando a ferramenta **view_frames**. Este nó do ROS 2 gera um diagrama da árvore TF atual e o salva como um arquivo PDF, que você pode visualizar facilmente.

O comando a seguir produz um arquivo PDF no diretório onde é executado, contendo a árvore TF atual que está sendo transmitida no sistema: `ros2 run tf2_tools view_frames`.

![viewframse1_humble](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/viewframse1_humble.png)

O pdf retorna o que chamamos de **árvore TF**. Ela representa todos os **quadros** dos robôs no sistema **e suas conexões**. O pdf também fornece algumas informações extras:
* Broadcaster: Este é o nome do transmissor de dados TF.
* A taxa média de publicação em Hz.
* O número da transformação mais recente e sua antiguidade. 0,0 significa que se trata de uma transformação constante ou Transformação Estática.
* A quantidade de dados armazenada no buffer TF em segundos de dados.

## Visualizar quadros TF usando rqt_tf_tree
**rqt_tf_tree** oferece a mesma funcionalidade que **view_frames**, com um benefício adicional:

Você pode atualizar a visualização e ver as alterações em **tempo real**, sem precisar gerar um novo PDF a cada vez.
Isso é especialmente útil ao testar novas transmissões de TF, pois permite verificar rapidamente se o TF que você está usando está publicando ativamente ou se é uma publicação desatualizada.

Para rodar o rqt_tf_tree use o comando: `ros2 run rqt_tf_tree rqt_tf_tree`.

Este TF não é estático, mas pode ser atualizado para mostrar o status atual da árvore de TFs. Pressione o botão de atualização sempre que desejar a atualização mais recente.

## Visualizar quadros TF no terminal usando tf_echo
O ROS usa tópicos para comunicar transformações. Como resultado, você pode ver todos esses dados brutos por meio de **tópicos**.

Há um tópico chamado **/tf** e outro chamado **/tf_static**, onde todos os TFs são publicados. O único problema é que TODOS os quadros são publicados lá.

Existe uma ferramenta de linha de comando útil que filtra a transformação de seu interesse e a exibe. Ou, mais importante, **ela calcula uma transformação indireta entre dois quadros conectados** (quadros intermediários fazendo a conexão entre o qudro de referência e o quadro alvo). Isso é útil e usado em muitas aplicações.

* O tópico **/tf** publica apenas os TFs diretos, não todas as transformações entre todos os quadros. 
* **tf_echo** retorna as transformações entre quaisquer quadros conectados para você.

### exemplo
Neste exemplo, veja como ecoar (echo) o tópico /tf e, em seguida, usar a ferramenta tf_echo para filtrar os dados do tópico /tf.

1. Execute o seguinte comando para ver uma publicação do tópico /tf diretamente: `ros2 topic echo /tf`.

Como você pode ver, muitos dados são publicados a cada segundo. Portanto, é difícil ou impossível obter os dados necessários, pois, aqui, você está publicando apenas as transformações TF de um quadro para o próximo quadro conectado (quadros conectados diretamente).

No entanto, se você estiver interessado em dois quadros que **não estão diretamente conectados**, precisará de outra ferramenta: **tf2_echo**.

2. Agora, filtre os dados do TF com a ferramenta **tf2_echo** para ver apenas a transformação entre o quadro **/rgb_camera_link_frame** e o quadro **/turtle_chassis**. Aqui estão o caminho e as transformações acumuladas que este sistema realiza e, no final, fornece um resultado:

![tfecho](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/tftransformecho_ros2_1.png)

O comando tf2_echo deve ser executado com a seguinte estrutura geral: `ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]`.

* [reference_frame] é onde você inicia a transformação, por exemplo, rgb_camera_link_frame.
* [target_frame] é onde você deseja finalizar a transformação, por exemplo, turtle_chassis.

Isso significa que você quer saber a translação e a rotação do reference_frame para o target_frame. (`ros2 run tf2_ros tf2_echo rgb_camera_link_frame turtle_chassis`).

Aguarde aproximadamente 10 segundos. As transformações começarão a ser exibidas. Normalmente, a primeira mensagem diz que não existe, mas o sistema TF precisa de um pouco de sincronização com os horários publicados pelo TF.

Aqui, com seu registro de data e hora, você pode ver a translação e a rotação de rgb_camera_link_frame para turtle_chassis:

![tf2_echo](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/tf2_echo.png)

## Visualizar quadros TF usando RVIZ2
Uma das melhores maneiras de confirmar se os TFs estão sendo publicados e visualizar as alterações é visualizar cada quadro no espaço 3D. O RVIZ2 pode ajudar com isso.

* É a melhor maneira de ver o que o tópico /tf está publicando, porque você o vê representado no espaço.
* Se um TF não for mais publicado, ele ficará cinza e desaparecerá, então o RVIZ2 fornece informações importantes sobre problemas.
* Ele também não renderizará dados de sensores ou qualquer coisa que precise de TF se o quadro fixo não estiver definido corretamente no RVIZ e, mesmo assim, não renderizará se, por exemplo, você tiver uma árvore quebrada com vários quadros raiz. Esse é um problema comum em sistemas multirrobôs.

### Exemplo
Neste exemplo, você representa os dados do tópico /tf no espaço 3D e observa como eles mudam ao mover a tartaruga. Você também instrui a câmera a seguir um quadro específico.

1. Abra o **RVIZ2** e adicione os seguintes elementos ao RVIZ e às configurações:
    * Defina o 'Fixed Frame' como /world.
2. Adicione dois modelos de robôs com configurações de tópicos diferentes:
    * Encontre os botões "Adicionar" na parte inferior do grupo "Exibições" e clique neles para adicionar cada modelo de robô.
    * Use a função "Renomear" para definir o nome exibido para cada modelo. Nesse exemplo é usado os nomes "TurtleRobotModel" e "CamBotRobotModel", mas você pode escolher qualquer nome de sua preferência. Este nome identifica apenas o modelo de robô no painel esquerdo do RVIZ.
3. Configure o primeiro modelo de robô para ler o tópico turtle_robot_description.
4. Configure o segundo modelo de robô para ler o tópico cam_bot_robot_description.
5. Verifique se as configurações de QoS estão corretas.

![rviz2_config_4](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/rviz2_config_4.png)

![rviz2_config_3](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/rviz2_config_3.png)

6. Adicione uma leitura de imagem do tópico /camera/image_raw e configure o QoS correto.

![rviz2_config_5](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/rviz2_config_5.png)

7. Claro, adicione o tf para visualizar os TFs em ação:
    * Altere a escala do marcador para 1,0 para ver melhor os quadros.

![rviz2_config_2](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/rviz2_config_2.png)

Agora você deve ver algo semelhante a isto:

![cambotrviz_humble1](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/cambotrviz_humble1.png)

Agora, observe que um quadro sempre segue a tartaruga. Esse quadro é chamado **turtle_attach_frame**.

8. Diga ao **Cam_bot** para imitar a posição e a orientação do **turtle_attach_frame**. Ao fazer isso, ele segue a tartaruga e a câmera captura a tartaruga o tempo todo.

Para isso, execute o seguinte comando: `ros2 run turtle_tf_3d_ros2 move_generic_model.py`. Este script ativa o movimento para um sistema de quadros do Cam_bot.

Ao iniciar, alguns AVISOS e ERROS apareceram. Isso é normal.

```shell
[WARN] [1702569495.705446757] [force_move_cam_bot]: No Coordinates available yet...
[INFO] [1702569495.706380946] [force_move_cam_bot]: Moved the Robot to frame =carrot
[ERROR] [1702569495.725491162] [force_move_cam_bot]: Could not transform world to carrot: "carrot" passed to lookupTransform argument source_frame does not exist.
```

Ele está aguardando que você informe qual sistema de coordenadas seguir.
É isso que você fará na próxima etapa.

9. Envie um comando para o tópico **/desired_frame** para dizer ao Cam_bot para se mover e imitar a posição e orientação daquele quadro: `ros2 topic pub /destination_frame std_msgs/msg/String "data: 'turtle_attach_frame'"`.

Agora você deve conseguir que o Cam_bot siga a tartaruga por onde ela for.

Como você pode ver, usando os TFs publicados, o **Cam_bot** pode seguir a orientação que você deseja atingir, neste caso, a **tartaruga**.

# Broadcast & Listen nos dados de TF
Entender como os TFs são publicados e recebidos é crucial para tarefas como localização, navegação e manipulação de robôs.

Nesta sessão, aprenda os conceitos básicos de transmissão e escuta de TF por meio da seguinte cena:

A principal diferença, além do mundo, é o fato de que:
* O Cam_bot não publica seu TF de cam_bot_base_link para o mundo. Isso significa que você não pode posicionar o Cam_bot no mundo.
* O quadro do Odom da tartaruga NÃO está conectado ao quadro do mundo, o que cria uma cena TF de duas árvores, uma para cada robô.
* O Cam_bot não possui esse sistema para seguir quadros; portanto, não é fácil seguir a tartaruga usando apenas os controles básicos.

Então você corrigirá esses problemas e, no processo, aprenderá sobre TF2 no ROS2.

## TF Broadcaster 
Objetivo: Resolver o problema em que o Cam_bot não tem uma transformação de **cam_bot_base_link para world**.

Os comandos `ros2 topic list` e `ros2 topic info -v` são úteis para ajudar você a configurar algumas definições do Rviz, como determinar uma configuração de **QoS** confiável para esse tópico.

Tarefas:

1. Inicie o RVIZ.
2. Adicione dois itens **RobotModel** à lista de exibição, um para o **Cam_bot**:
    * No campo "Tópico de Descrição", digite o tópico que publica os dados do modelo de robô. Neste caso, o tópico é **/cam_bot_robot_description**.

e outro para a **tartaruga**. Os tópicos precisam ser configurados corretamente.
3. Adicione um novo **display TF** para visualizar os quadros TF na visualização 3D com a grade.
4. Selecione "mundo" como **Quadro Fixo** nas Opções Globais.
5. Adicione também um **display de Imagem** para mostrar a visualização da perspectiva da câmera do robô. Lembre-se de que você precisa preencher o "Tópico de Imagem".
6. Agora você deve ter uma janela de imagem que será usada posteriormente.

Repita os passos 2 a 6 para adicionar um segundo modelo de robô. No campo "Nome de Exibição", digite um nome diferente, como "tartaruga". No campo "Tópico de Descrição", digite o tópico que publica os dados do modelo de robô para o robô tartaruga. Ambos os modelos de robô devem agora ser exibidos no RVIZ.

Como você pode ver no rviz, ambos os modelos de robô são brancos e se sobrepõem. Isso significa que não há transformação do quadro do mundo para o link raiz de nenhum modelo de robô:

Tente trocar a estrutura fixa (/world) por chassi. Se fizer isso, pelo menos verá que o Cam_bot não é mais branco.

![errornotf_humble1](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/errornotf_humble1.png)   


Além disso, você deve ver que, no RVIZ, o modelo do robô não se move, mas a imagem da câmera à esquerda mostra que ele está se movendo.

Este é o comportamento esperado, já que o Quadro Fixo está definido como chassi.

No entanto, e se você quiser mover o robô e também ver esse movimento refletido dentro da área de visualização 3D principal do RVIZ?

Você não pode fazer isso agora porque sua árvore TF tem uma transformação ausente.

Não há transformação da estrutura do chassi para a estrutura do mundo. Verifique isso observando a árvore TF: `ros2 run rqt_tf_tree rqt_tf_tree`.

![tftree_u2_unconnected](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/tftree_u2_unconnected.png)

Como você pode ver, você tem duas árvores TF com duas raízes diferentes. O nó mais alto em uma árvore é chamado de raiz. Observe também que não há um quadro de mundo.

Resolva isso agora. No entanto, primeiro feche o RVIZ e o rqt_tf_tree antes de continuar.

Crie um script Python chamado cam_bot_odom_to_tf_pub.py dentro do pacote my_tf_ros2_course_pkg.

Este script extrai a odometria do Cam_bot e publica uma transformação TF estática do camera_bot_base_link para o mundo do quadro.

Como este pacote é do tipo de compilação **ament_cmake**, crie uma pasta chamada **scripts** para colocar um script Python lá.

**cam_bot_odom_to_tf_pub.py:**

```python
#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class CamBotOdomToTF(Node):

    def __init__(self, robot_base_frame="camera_bot_base_link"):
        super().__init__('odom_to_tf_broadcaster_node')

        self._robot_base_frame = robot_base_frame
        
        # Create a new `TransformStamped` object.
        # A `TransformStamped` object is a ROS message that represents a transformation between two frames.
        self.transform_stamped = TransformStamped()
        # This line sets the `header.frame_id` attribute of the `TransformStamped` object.
        # The `header.frame_id` attribute specifies the frame in which the transformation is defined.
        # In this case, the transformation is defined in the `world` frame.
        self.transform_stamped.header.frame_id = "world"
        # This line sets the `child_frame_id` attribute of the `TransformStamped` object.
        # The `child_frame_id` attribute specifies the frame that is being transformed to.
        # In this case, the robot's base frame is being transformed to the `world` frame.
        self.transform_stamped.child_frame_id = self._robot_base_frame

        self.subscriber = self.create_subscription(
            Odometry,
            '/cam_bot_odom',
            self.odom_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        # This line creates a new `TransformBroadcaster` object.
        # A `TransformBroadcaster` object is a ROS node that publishes TF messages.
        self.br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("odom_to_tf_broadcaster_node ready!")

    def odom_callback(self, msg):
        self.cam_bot_odom = msg
        # print the log info in the terminal
        self.get_logger().debug('Odom VALUE: "%s"' % str(self.cam_bot_odom))
        self.broadcast_new_tf()

    def broadcast_new_tf(self):
        """
        This function broadcasts a new TF message to the TF network.
        """

        # Get the current odometry data.
        position = self.cam_bot_odom.pose.pose.position
        orientation = self.cam_bot_odom.pose.pose.orientation

        # Set the timestamp of the TF message.
        # The timestamp of the TF message is set to the current time.
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        # Set the translation of the TF message.
        # The translation of the TF message is set to the current position of the robot.
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z

        # Set the rotation of the TF message.
        # The rotation of the TF message is set to the current orientation of the robot.
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        # Send (broadcast) the TF message.
        self.br.sendTransform(self.transform_stamped)


def main(args=None):

    rclpy.init()
    odom_to_tf_obj = CamBotOdomToTF()
    rclpy.spin(odom_to_tf_obj)

if __name__ == '__main__':
    main()
```

Vemos analisar alguns trechos:

```python
self.br = tf2_ros.TransformBroadcaster(self)
```

Isso cria o objeto TF Broadcasting que você usa para transmitir os TFs:


```python
self.transform_stamped = TransformStamped()
self.transform_stamped.header.frame_id = "world"
self.transform_stamped.child_frame_id = self._robot_base_frame
```

* Esta parte cria a mensagem usada para o TF.
* Defina os valores que não serão alterados.
* frame_id = O quadro respeita a transformação, o Quadro Pai.
* child_frame_id é o quadro no qual você está publicando a transformação do TF.

Observe que, ao transmitir, não importa se esses quadros existem ou não.

* Se ambos existirem, você publicará um novo TF que provavelmente interferirá no que ja está sendo publicando.
* Se um dos quadros existir e o outro não, o que não existe será criado e publicado para transformar o que existe.
* Se nenhum existir, ambos os quadros serão criados.

Na realidade, os quadros não são criados. Os dados do TF são apenas um tópico que possui transformações entre quadros, nada mais. Portanto, não há conexão real entre os quadros do TF e o robô real/simulado por padrão.

```python
def broadcast_new_tf(self):
    """
    This function broadcasts a new TF message to the TF network.
    """

    # Get the current odometry data.
    position = self.cam_bot_odom.pose.pose.position
    orientation = self.cam_bot_odom.pose.pose.orientation

    # Set the timestamp of the TF message.
    # The timestamp of the TF message should be set to the current time.
    self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

    # Set the translation of the TF message.
    # The translation of the TF message should be set to the current position of the robot.
    self.transform_stamped.transform.translation.x = position.x
    self.transform_stamped.transform.translation.y = position.y
    self.transform_stamped.transform.translation.z = position.z

    # Set the rotation of the TF message.
    # The rotation of the TF message should be set to the current orientation of the robot.
    self.transform_stamped.transform.rotation.x = orientation.x
    self.transform_stamped.transform.rotation.y = orientation.y
    self.transform_stamped.transform.rotation.z = orientation.z
    self.transform_stamped.transform.rotation.w = orientation.w

    # Send (broadcast) the TF message.
    self.br.sendTransform(self.transform_stamped)
```

* Neste caso, você está extraindo a posição e a orientação do robô por meio de sua odometria. Robôs normalmente publicam esses tópicos de odometria. Esta é uma ótima maneira de obter os primeiros dados do TF.

* Em seguida, use o método sendTransform() para publicar a transformação preparada.

```python
self.subscriber = self.create_subscription(
    Odometry,
    '/cam_bot_odom',
    self.listener_callback,
    QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))
```

```python
def odom_callback(self, msg):
    self.cam_bot_odom = msg
    # print the log info in the terminal
    self.get_logger().debug('Odom VALUE: "%s"' % str(self.cam_bot_odom))
    self.broadcast_new_tf()
```

* Aqui está o código relacionado à extração dos dados de odometria.
* Cada vez que você recebe dados de odometria, você transmite uma nova transformação TF.

Após compilar o pacote e executar: `ros2 run rqt_tf_tree rqt_tf_tree` novamente.

você deverá ver um gráfico parecido com a imagem abaixo:

![worldtree_humble](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/worldtree_humble.png)

## tf2_monitor
Esta ferramenta é usada para verificar o **atraso entre transformações**. Isso significa quanto tempo decorre entre a publicação de um quadro e outro quadro conectado.

* Se estiverem conectados diretamente, é o tempo entre esses quadros.
* No entanto, suponha que os quadros NÃO estejam conectados diretamente. Nesse caso, o tempo será acumulado desde a publicação do quadro original até a publicação do quadro de destino.

E por que você precisa disso? Um sistema comum e crítico são os carimbos de tempo (time stamps). Os dados do sensor, chamados de quadros, devem ser consistentes com a publicação de tempo do TF. Caso contrário, os dados do sensor serão descartados.

Analise seu sistema para entender melhor isso. 
Agora veja os tempos do quadro **camera_bot_base_link** até **rgb_camera_link_frame**: `ros2 run tf2_ros tf2_monitor camera_bot_base_link rgb_camera_link_frame`.

```shell
RESULTS: for camera_bot_base_link to rgb_camera_link_frame
Chain is: rgb_camera_link_frame -> chassis -> camera_bot_base_link
Net delay     avg = 3.23952e+08: max = 1.70257e+09

Frames:
Frame: camera_bot_base_link, published by , Average Delay: 0.000354682, Max Delay: 0.00420332
Frame: chassis, published by , Average Delay: 1.70257e+09, Max Delay: 1.70257e+09
Frame: rgb_camera_link_frame, published by , Average Delay: 1.70257e+09, Max Delay: 1.70257e+09

All Broadcasters:
Node:  155.869 Hz, Average Delay: 1.28033e+09 Max Delay: 1.70257e+09
```

Agora veja os tempos do frame **world** para o **camera_bot_base_link**: `ros2 run tf2_ros tf2_monitor world camera_bot_base_link`.

```shell
RESULTS: for world to camera_bot_base_link
Chain is: world -> camera_bot_base_link
Net delay     avg = 0.00443906: max = 0.0457809

Frames:
Frame: camera_bot_base_link, published by , Average Delay: 0.000344517, Max Delay: 0.00268316

All Broadcasters:
Node:  157.413 Hz, Average Delay: 1.28033e+09 Max Delay: 1.70257e+09
```

Você pode ver que:

* Atraso médio para rgb_camera_link_frame -> chassis -> camera_bot_base_link == 3,23952e+08 segundos
* Atraso médio para world -> camera_bot_base_link == 0,00443906 segundos

Por que o atraso na transformação de rgb_camera_link_frame para chassis para camera_bot_base_link é tão alto?

* Isso ocorre porque essas transformações são estáticas ou se comportam como transformações estáticas. Isso significa que o atraso do TF é, na realidade, zero. O fato de o atraso ser tão alto reflete isso, mesmo quando é contraintuitivo.
* Então, qual é o problema?

O problema é que o atraso médio da transformação world -> camera_bot_base_link é de 0,00443906 segundos (4,4 ms). E 4 ms é muito para o módulo de imagem no RVIZ considerar essas transformações aceitáveis.

Então, o que você pode fazer? Bem, houve um pequeno erro no código original que gerou toda essa confusão:

* Extraia o tempo dos dados do sensor e utilize-o no TF. Neste caso, o tempo de odometria.
* Os dados do TF serão consistentes e não haverá atraso entre os dois quadros. Agora, faça estas correções para criar um novo script:

**cam_bot_odom_to_tf_pub_late_tf_fixed.py:**

```python
#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class CamBotOdomToTF(Node):

    def __init__(self, robot_base_frame="camera_bot_base_link"):
        super().__init__('odom_to_tf_broadcaster_node')

        self._robot_base_frame = robot_base_frame
        
        # Create a new `TransformStamped` object.
        # A `TransformStamped` object is a ROS message that represents a transformation between two frames.
        self.transform_stamped = TransformStamped()
        # This line sets the `header.frame_id` attribute of the `TransformStamped` object.
        # The `header.frame_id` attribute specifies the frame in which the transformation is defined.
        # In this case, the transformation is defined in the `world` frame.
        self.transform_stamped.header.frame_id = "world"
        # This line sets the `child_frame_id` attribute of the `TransformStamped` object.
        # The `child_frame_id` attribute specifies the frame that is being transformed to.
        # In this case, the robot's base frame is being transformed to the `world` frame.
        self.transform_stamped.child_frame_id = self._robot_base_frame

        self.subscriber = self.create_subscription(
            Odometry,
            '/cam_bot_odom',
            self.odom_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        # This line creates a new `TransformBroadcaster` object.
        # A `TransformBroadcaster` object is a ROS node that publishes TF messages.
        self.br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("odom_to_tf_broadcaster_node ready!")

    def odom_callback(self, msg):
        self.cam_bot_odom = msg
        # print the log info in the terminal
        self.get_logger().debug('Odom VALUE: "%s"' % str(self.cam_bot_odom))
        self.broadcast_new_tf()

    def broadcast_new_tf(self):
        """
        This function broadcasts a new TF message to the TF network.
        """

        # Get the current odometry data.
        time_header = self.cam_bot_odom.header
        position = self.cam_bot_odom.pose.pose.position
        orientation = self.cam_bot_odom.pose.pose.orientation

        # Set the timestamp of the TF message.
        # The timestamp of the TF message is set to the odom message time.
        self.transform_stamped.header.stamp = time_header.stamp

        # Set the translation of the TF message.
        # The translation of the TF message should be set to the current position of the robot.
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z

        # Set the rotation of the TF message.
        # The rotation of the TF message should be set to the current orientation of the robot.
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        # Send (broadcast) the TF message.
        self.br.sendTransform(self.transform_stamped)


def main(args=None):

    rclpy.init()
    odom_to_tf_obj = CamBotOdomToTF()
    rclpy.spin(odom_to_tf_obj)

if __name__ == '__main__':
    main()
```

Observe que a única coisa que mudou é que agora você usa o **horário das mensagens de dados de odometria, em vez do horário atual do relógio**:

```python
...
        # Get the current odometry data.
        time_header = self.cam_bot_odom.header
        position = self.cam_bot_odom.pose.pose.position
        orientation = self.cam_bot_odom.pose.pose.orientation

        # Set the timestamp of the TF message.
        # The timestamp of the TF message is set to the odom message time.
        self.transform_stamped.header.stamp = time_header.stamp
...
```

Compile e verifique o tempo médio da transformação: `ros2 run tf2_ros tf2_monitor world camera_bot_base_link`.

```shell
RESULTS: for world to camera_bot_base_link
Chain is: world -> camera_bot_base_link
Net delay     avg = 1.6717e+09: max = 1.70257e+09

Frames:
Frame: camera_bot_base_link, published by , Average Delay: 1.70257e+09, Max Delay: 1.70257e+09

All Broadcasters:
Node:  157.79 Hz, Average Delay: 1.49486e+09 Max Delay: 1.70257e+09
```

Agora você tem o seguinte:

* Atraso médio de transformação para o mundo da cadeia -> camera_bot_base_link == 1,6717e+09 segundos.
* Isso indica que se comporta como uma transformação estática, ou seja, o atraso é nulo.

Ótimo! No entanto, agora você precisa resolver os seguintes problemas:

* Os quadros de transformação da tartaruga não estão representados corretamente no RVIZ2.
* Isso ocorre porque nenhum dos quadros do robô está CONECTADO.

Corrija isso usando uma **transformação estática**.

## Static Broadcaster
Você pode publicar transformações estáticas de três maneiras diferentes:

* Por meio da linha de comando
* Por meio de um programa Python/C++
* Por meio de arquivos de inicialização

Transformações estáticas são usadas para TFs que **NÃO mudam ao longo do tempo**. O motivo é que as transformações estáticas são publicadas no tópico tf_static e somente quando mudam, reduzindo significativamente o impacto no desempenho do sistema e no uso de recursos.

### Static Broadcaster usando a linha de comando
Vamos explorar como usar a linha de comando para transmitir transformações estáticas no ROS 2

1. Opção 1: XYZ Roll-Pitch-Yaw (radianos): Se você quiser definir a orientação usando ângulos de Euler, o comando ros2 run deve obedecer à seguinte estrutura: `ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id`. 

2. Opção 2: Quaternion XYZW: O esquema geral para definir a orientação em Quaternions é o seguinte: `ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id`

3. No entanto, você também pode usar esses comandos dentro de um **arquivo de inicialização**. É isso que você usará para publicar uma Transformação Estática do mundo (frame raiz do Cam_bot) -> odom (frame raiz do TurtleBot).

### Static Broadcaster por meio de um arquivo de inicialização (launch)
Vamos agora explorar como usar arquivos de inicialização para transmitir transformações estáticas no ROS 2.

* Estamos publicando o TF do **camera_bot_base_link** no odom, como fizemos na seção anterior.
* Estamos publicando o TF estático do **Odom** para o **world**.
* Isso conectará o quadro do mundo ao quadro do Odom, e então o quadro do Odom será conectado ao link base do camera_bot_base, conectando assim todas as partes do robô.

**publish_static_transform_odom_to_world.launch.py:**

```python
#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    return LaunchDescription(
        [
            static_tf_pub
        ]
    )
```

O arquivo de inicialização acima define um nó chamado **static_tf_pub**. Este nó inicia o executável **static_transform_publisher** do pacote **tf2_ros**. Este nó é configurado para publicar uma transformação estática entre os quadros **world** e **odom** usando os argumentos. Os argumentos para o nó são a ***translação (x, y e z)*** e a ***rotação (roll, pitch, yaw)***  da transformação.

Verifique a árvore TF: `ros2 run rqt_tf_tree rqt_tf_tree`:

![working_completetreetf_ros2_u2](https://github.com/marcospontoexe/ROS_2/blob/main/tf/imagens/working_completetreetf_ros2_u2.png)

Agora, todos os quadros TF estão conectados em uma única ÁRVORE TF. Isso permite que o RVIZ renderize tudo no espaço. E agora, você deve conseguir mover os dois robôs e verificar se a posição deles no espaço no RVIZ2 está correta e igual à do mundo real/simulação.

### Static Broadcaster via Python script
Por fim, vamos explorar como usar um script Python para transmitir transformações estáticas no ROS 2.

Escreva um script para criar um novo quadro anexado à tartaruga que fique mais próximo dela, para que você tenha um bom primeiro plano.

O script Python abaixo pode ser usado para publicar qualquer coisa estaticamente onde você quiser, essencialmente da mesma forma que os comandos que você usou anteriormente:

**static_broadcaster_front_turtle_frame.py:**

```python
#! /usr/bin/env python3
import sys
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_broadcaster_front_turtle_frame_node')

        # This line creates a `StaticTransformBroadcaster` object.
        # The `StaticTransformBroadcaster` object is used to publish static transforms between frames.
        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms()

        self.get_logger().info("static_broadcaster_front_turtle_frame ready!")

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = sys.argv[1]
        static_transformStamped.child_frame_id = sys.argv[2]
        static_transformStamped.transform.translation.x = float(sys.argv[3])
        static_transformStamped.transform.translation.y = float(sys.argv[4])
        static_transformStamped.transform.translation.z = float(sys.argv[5])
        quat = tf_transformations.quaternion_from_euler(float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

def main():
    rclpy.init()
    assert len(sys.argv) > 8, "Please add all the arguments: ros2 run tf_ros2_solutions parent_frame child_frame x y z roll pitch yaw"
    node_obj = StaticFramePublisher()
    try:
        rclpy.spin(node_obj)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

A única diferença para o exemplo anteriro (tf dinâ,ica) é que você cria um objeto StaticTransformBroadcaster:

```python
StaticTransformBroadcaster(self)
```

Em vez de um objeto **TransformBroadcaster** regular como feito antes:


```python
TransformBroadcaster(self)
```

Isso torna o quadro permanente após a publicação. Portanto, você não precisa publicá-lo regularmente para não considerá-lo obsoleto. Essa é a vantagem de usar um objeto transmissor de transformação estático.

É por isso que não há loop de transmissão, apenas o spin.

Considere os seguintes parâmetros ao iniciar o novo script static_broadcaster_front_turtle_frame.py:

* quadro pai = turtle_chassis
* quadro filho = my_front_turtle_frame (você pode colocar o nome que quiser)
* X = 0,4 (são 0,4 metros no eixo X do quadro pai)
* Y = 0,0
* Z = 0,4 (são 0,4 metros no eixo Z do quadro pai)
* Roll = 0,0
* Pitch = 0,7 (você quer que ele aponte para 45 graus/0,7 radianos de inclinação para baixo)
* Yaw = 3,1416 (você quer que ele aponte para 180 graus/3,1416 radianos de inclinação para baixo)

Como é uma transformação estática, a transformação do quadro permanece lá para sempre — **não há necessidade de publicá-la novamente**.

##  TF Listener (Ouvinte)
Os Ouvintes TF recebem dados TF e os utilizam para o que você precisar. Quando um novo nó inicia um ouvinte TF, ele escuta os tópicos tf e tf_static. Em seguida, ele cria um buffer de relacionamentos de quadros, que usa para resolver consultas TF. Portanto, cada nó só tem conhecimento das transformações publicadas desde o início daquele nó.

Para mostrar como isso funciona, crie um script que faça o Cam_bot seguir um quadro, neste caso, o turtle_attach_frame, como você fez na seção anterior.

**move_generic_model.py:**

```python
#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations

from std_msgs.msg import String

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
# from gazebo_msgs.msg import ModelState

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
class CamBotMove(Node):
    def __init__(self, timer_period=0.05, model_name="cam_bot", init_dest_frame="carrot", trans_speed=1.0, rot_speed=0.1):
        super().__init__('force_move_cam_bot')
        
        self._model_name = model_name

        self.set_destination_frame(new_dest_frame=init_dest_frame)
        self.set_translation_speed(speed=trans_speed)
        self.set_rotation_speed(speed=rot_speed)

        # For the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer_period = timer_period
        self.timer_rate = 1.0 / self.timer_period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.subscriber= self.create_subscription(
            String,
            '/destination_frame',
            self.move_callback,
            QoSProfile(depth=1))

        self.set_entity_client = self.create_client(SetEntityState, "/cam_bot/set_entity_state")
        while not self.set_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service READY...')
        
        # create a request
        self.req = SetEntityState.Request()

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.move_step_speed()
        self.get_logger().info("Moved the Robot to frame ="+str(self.objective_frame))

    def set_translation_speed(self, speed):
        self.trans_speed = speed
    
    def set_rotation_speed(self, speed):
        self.rot_speed = speed

    def set_destination_frame(self, new_dest_frame):
        self.objective_frame = new_dest_frame

    def move_callback(self, msg):
        self.set_destination_frame(new_dest_frame=msg.data)
        
    def move_step_speed(self):
        coordinates_to_move_to = self.calculate_coord()
        if coordinates_to_move_to is not None:
            self.move_model(coordinates_to_move_to)
        else:
            self.get_logger().warning("No Coordinates available yet...")

    def get_model_pose_from_tf(self, origin_frame="world", dest_frame="camera_bot_base_link"):
        """
        Extract the pose from the TF
        """
        # Look up for the transformation between dest_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach dest_frame
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                origin_frame,
                dest_frame,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {origin_frame} to {dest_frame}: {ex}')
            return None


        translation_pose = trans.transform.translation
        rotation_pose = trans.transform.rotation

        self.get_logger().info("type translation_pose="+str(type(translation_pose)))
        self.get_logger().info("type rotation_pose="+str(type(rotation_pose)))


        pose = Pose()
        pose.position.x = translation_pose.x
        pose.position.y = translation_pose.y
        pose.position.z = translation_pose.z
        pose.orientation.x = rotation_pose.x
        pose.orientation.y = rotation_pose.y
        pose.orientation.z = rotation_pose.z
        pose.orientation.w = rotation_pose.w

        return pose

    def calculate_coord(self):
        """
        Gets the current position of the model and adds the increment based on the Publish rate
        """
        pose_dest = self.get_model_pose_from_tf(origin_frame="world", dest_frame=self.objective_frame)
        self.get_logger().error("POSE DEST="+str(pose_dest))
        if pose_dest is not None:

            explicit_quat = [pose_dest.orientation.x, pose_dest.orientation.y,
                             pose_dest.orientation.z, pose_dest.orientation.w]
            pose_now_euler = tf_transformations.euler_from_quaternion(explicit_quat)

            roll = pose_now_euler[0]
            pitch = pose_now_euler[1]
            yaw = pose_now_euler[2]

            coordinates_to_move_to = Coordinates(x=pose_dest.position.x,
                                                y=pose_dest.position.y,
                                                z=pose_dest.position.z,
                                                roll=roll,
                                                pitch=pitch,
                                                yaw=yaw)
        else:
            coordinates_to_move_to = None

        return coordinates_to_move_to

    def move_model(self, coordinates_to_move_to):
        pose = Pose()

        pose.position.x = coordinates_to_move_to.x
        pose.position.y = coordinates_to_move_to.y
        pose.position.z = coordinates_to_move_to.z

        quaternion = tf_transformations.quaternion_from_euler(coordinates_to_move_to.roll,
                                                              coordinates_to_move_to.pitch,
                                                              coordinates_to_move_to.yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # You set twist to Null to remove any prior movements
        twist = Twist()
        linear = Vector3()
        angular = Vector3()

        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        twist.linear = linear
        twist.angular = angular

        state = EntityState()
        state.name = self._model_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = "world"

        self.req.state = state

        self.get_logger().error("STATE to SEND="+str(self.req.state))

        # send the request
        try:     
            self.future = self.set_entity_client.call_async(self.req)
        except Exception as e:
            self.get_logger().error('Error on calling service: %s', str(e))
        
            

def main(args=None):
    rclpy.init()
    move_obj = CamBotMove()
    print("Start Moving")
    rclpy.spin(move_obj)

if __name__ == '__main__':
    main()
```

Este script é bastante complexo, mas vamos analisar os elementos que definem um TF Listener:

```python
# For the TF listener
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
```

As linhas de código acima iniciam um objeto ouvinte do TF. Observe que você também precisa de um objeto Buffer.

```python
def get_model_pose_from_tf(self, origin_frame="world", dest_frame="camera_bot_base_link"):
    """
    Extract the pose from the TF
    """
    # Look up for the transformation between dest_frame and turtle2 frames
    # and send velocity commands for turtle2 to reach dest_frame
    try:
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
            origin_frame,
            dest_frame,
            now)
    except TransformException as ex:
        self.get_logger().error(
            f'Could not transform {origin_frame} to {dest_frame}: {ex}')
        return None

    translation_pose = trans.transform.translation
    rotation_pose = trans.transform.rotation

    self.get_logger().info("type translation_pose="+str(type(translation_pose)))
    self.get_logger().info("type rotation_pose="+str(type(rotation_pose)))

    pose = Pose()
    pose.position.x = translation_pose.x
    pose.position.y = translation_pose.y
    pose.position.z = translation_pose.z
    pose.orientation.x = rotation_pose.x
    pose.orientation.y = rotation_pose.y
    pose.orientation.z = rotation_pose.z
    pose.orientation.w = rotation_pose.w

    return pose
```

Esta linha obtém a hora atual:

```python
now = rclpy.time.Time()
```

Esta linha aqui verifica o buffer e procura as transformações de origin_frame para dest_frame no momento:

```python
trans = self.tf_buffer.lookup_transform(origin_frame,dest_frame,now)
```

Se nada for encontrado, ele captura a exceção.

Finalmente, use as informações da transformação para criar um objeto **Pose()** para mover o robô Cam_bot para esse local preciso. Isso coloca o robô no mesmo quadro que você solicitou à transformação por meio de um tópico chamado **/destination_frame**.

Antes de tentar isso, crie um script de inicialização que execute tudo o que você fez nas seções anteriores, necessário para que todas as transformações funcionem, além deste novo script. Isso facilitará sua vida:

**start_tf_fixes.launch.xml:**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>

    <include file="$(find-pkg-share my_tf_ros2_course_pkg)/launch/publish_static_transform_odom_to_world.launch.py"/>

    <node pkg="my_tf_ros2_course_pkg" exec="cam_bot_odom_to_tf_pub_late_tf_fixed.py" name="cam_bot_odom_to_tf_pub_late_tf_fixed_node">
    </node>

    <node pkg="my_tf_ros2_course_pkg" exec="move_generic_model.py" name="move_generic_model_node">
    </node>

</launch>
```

Publique o TF estático que você deseja: `ros2 run my_tf_ros2_course_pkg static_broadcaster_front_turtle_frame.py turtle_chassis my_front_turtle_frame 0.4 0 0.4 0 0.7 3.1416`

Mate esse nó com CTRL+C.

Em seguida, diga ao Cam_bot para seguir esse quadro: `ros2 topic pub /destination_frame std_msgs/msg/String "data: 'my_front_turtle_frame'"`

Agora, se você mover a tartaruga, o Cam_bot deverá segui-la.

# Robot State Publisher

