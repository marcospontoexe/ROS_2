# Behavior Trees
Nesta unidade, você entenderá o conceito de BT e a arquitetura de software simplificada que pode ser acomodada no framework ROS2. 

# Arquitetura de Software. Árvores de Comportamento (Behavior Trees) - ROS2
Ao projetar um agente autônomo (um robô), considere um baixo nível de abstração.

Pense em motores, acionamentos, sensores, controladores de robô e software. Por fim, seu robô executa com excelência a tarefa programada. Por exemplo, seu manipulador pode pegar algo da mesa e colocá-lo em outro lugar, ou um robô de limpeza pode limpar o chão.

Para simplificar, imagine seu robô executando tarefas de pegar e colocar:

* Primeiro, seu robô captura a posição do objeto a ser pego.
* Em seguida, o robô calcula o caminho curto e sem colisões até o destino (onde o objeto deve ser colocado). Enquanto o caminho sem colisões é calculado, o robô executa a tarefa programada.

Como você pode imaginar, a descrição acima é simples e não inclui muitas outras tarefas que precisam ser avaliadas. Aqui mencionamos a otimização da posição de preensão, estratégias de preensão em caso de falha e planejamento de movimento, onde o modelo dinâmico do robô deve ser considerado. Além disso, pense na supervisão de movimento, segurança e outras tarefas que não sejam relevantes para o seu caso simples.

Como o curso discutirá o framework Behaviour Tree, especialmente no domínio ROS2, você deve entender que o BT não é o único framework dedicado ao ROS. O framework BT pode ser incorporado a qualquer outro software no qual você desenvolva sua aplicação (aqui, em C++). A indústria de jogos é um bom exemplo de como o BT aprimora a experiência do usuário. As possibilidades são enormes.