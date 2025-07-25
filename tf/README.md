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






