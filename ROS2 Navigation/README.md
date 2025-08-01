# Introdução

esse curso é voltado para a versão **Humble**.

# Como os robôs navegam

Os robôs navegam de uma forma que espelha a forma como os humanos se movem de um lugar para outro. Embora o processo pareça simples para nós, por ser instintivo, destrinchar o processo revela várias etapas essenciais.

Para se mover de um ponto a outro, um robô deve:

* Mapear o ambiente (Mapping) – O robô precisa criar ou acessar um mapa de seus arredores.
* Determinar sua localização (Localization) – Ele deve identificar onde está dentro do ambiente mapeado.
* Planejar uma rota (Path Planning) – O robô calcula a melhor maneira de se mover entre dois pontos.
* Executar o movimento evitando obstáculos (Controle e Desvio de Obstáculos) – Ele envia comandos para suas rodas (ou outros atuadores) para seguir o caminho planejado, detectando e evitando obstáculos em tempo real.

*Desenvolver todos esses recursos do zero é complexo e demorado. É aí que entra o ROS (Sistema Operacional do Robô) – ele fornece pacotes de navegação pré-construídos que simplificam o processo.

