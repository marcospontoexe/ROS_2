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

# Ver quadros TF em formato PDF
O ROS 2 permite visualizar os quadros TF de um robô em formato PDF usando a ferramenta **view_frames**. Este nó do ROS 2 gera um diagrama da árvore TF atual e o salva como um arquivo PDF, que você pode visualizar facilmente.

O comando a seguir produz um arquivo PDF no diretório onde é executado, contendo a árvore TF atual que está sendo transmitida no sistema: `ros2 run tf2_tools view_frames`.

O pdf retorna o que chamamos de **árvore TF**. Ela representa todos os **quadros** dos robôs no sistema **e suas conexões**. O pdf também fornece algumas informações extras:
* Broadcaster: Este é o nome do transmissor de dados TF.
* A taxa média de publicação em Hz.
* O número da transformação mais recente e sua antiguidade. 0,0 significa que se trata de uma transformação constante ou Transformação Estática.
* A quantidade de dados armazenada no buffer TF em segundos de dados.

# Visualizar quadros TF usando rqt_tf_tree
**rqt_tf_tree** oferece a mesma funcionalidade que **view_frames**, com um benefício adicional:

Você pode atualizar a visualização e ver as alterações em **tempo real**, sem precisar gerar um novo PDF a cada vez.
Isso é especialmente útil ao testar novas transmissões de TF, pois permite verificar rapidamente se o TF que você está usando está publicando ativamente ou se é uma publicação desatualizada.

Para rodar o rqt_tf_tree use o comando: `ros2 run rqt_tf_tree rqt_tf_tree`.

Este TF não é estático, mas pode ser atualizado para mostrar o status atual da árvore de TFs. Pressione o botão de atualização sempre que desejar a atualização mais recente.