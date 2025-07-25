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

Você revisará esse conceito importante em breve. Agora, aprenda sobre convenções antes de continuar.