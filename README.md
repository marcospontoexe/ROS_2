# ROS_2

O **ROS 2 (Robot Operating System 2)** é a segunda geração do **ROS**, um conjunto de bibliotecas e ferramentas de código aberto para o desenvolvimento de sistemas robóticos.

Ele foi redesenhado para superar as limitações do **ROS 1**, focando em **robustez, escalabilidade, segurança** e **tempo real**, sendo mais adequado para **aplicações industriais**, **robôs autônomos**, e sistemas complexos.


## 🆚 ROS 1 vs ROS 2 — Principais Melhorias

| Recurso                        | ROS 1                  | ROS 2                               |
| ------------------------------ | ---------------------- | ----------------------------------- |
| Comunicação                    | Baseada em `roscore`   | **DDS (Data Distribution Service)** |
| Tempo real                     | Limitado               | ✅ Suporte a tempo real              |
| Multiplataforma                | Linux (principalmente) | ✅ Linux, Windows, macOS             |
| Segurança                      | Ausente                | ✅ Segurança integrada (com DDS)     |
| Multi-robô                     | Complexo               | ✅ Comunicação descentralizada       |
| Gerenciamento de ciclo de vida | Limitado               | ✅ Com suporte nativo                |


## Estrutura do ROS 2

* **Nodes:** módulos que executam tarefas específicas.
* **Topics:** canais para troca de mensagens assíncronas.
* **Services:** chamadas de função síncronas entre nodes.
* **Actions:** chamadas assíncronas com feedback contínuo (ex: mover o robô).
* **Parameters:** valores configuráveis em tempo de execução.
* **Launch system:** baseado em Python (mais poderoso que o XML do ROS 1).
* **Packages:** unidades de software com funcionalidades específicas.


## Comunicação com DDS

ROS 2 usa o padrão **DDS (Data Distribution Service)** para comunicação entre nodes. Isso significa:

* Não precisa mais do `roscore`
* Comunicação mais robusta e distribuída
* Suporte nativo a **QoS (Qualidade de Serviço)**


## Ferramentas do ROS 2

* **colcon:** ferramenta de build moderna que substitui o `catkin`
* **rviz2:** visualização gráfica do robô e sensores
* **ros2bag:** gravação e reprodução de dados
* **rqt:** interface gráfica para debug e visualização
* **launch:** arquivos Python para iniciar múltiplos nodes


## Linguagens Suportadas

* **C++**
* **Python**
* (E suporte parcial a Rust e outras via bindings)


 ## C++
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/C%2B%2B) dicas sobre C++.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(Python)) veja o básico de Ros2 usando Python.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Basics%20in%205%20Days%20(C%2B%2B)) veja o básico de Ros2 usando C++.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/Intermediate%20ROS2%20(C%2B%2B)) veja Ros2 intermediário usando C++.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/ROS2%20Navigation%20(python)) Navigation para Ros2 usando Python.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/Advanced%20ROS2%20Navigation%20(python)) Navigation avançado para Ros2 usando Python.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/URDF%20for%20Robot%20Modeling%20in%20ROS2) como usar URDF no Ros2.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/tf) como usar TF no Ros2.
* [Veja nesse repositório](https://github.com/marcospontoexe/ROS_2/tree/main/Behavior%20Trees%20for%20ROS2%20(C%2B%2B)) Behavior Trees para Ros2 usando C++.



```mermaid
graph TD
    subgraph "Ciclo de Operação do Robô"
        direction LR

        %% --- Definição dos Estados ---
        S1[("Teleoperação  
(LED Branco)")]
        S2[("Mapeamento (SLAM)  
(LED Azul)")]
        S3[("Criação de Rotas  
(LED Verde)")]

        %% --- Transições Principais ---
        S1 -- "Evento: select + start" --> S2
        S2 -- "Evento: select + start" --> S3
        S3 -- "Evento: select + start" --> S1

        %% --- Transição de Atalho (Salvar Mapa) ---
        S2 -- "Evento: select + R1  
(Salvar Mapa)" --> S3

        %% --- Ações Internas (Loops no mesmo estado) ---
        S3 -- "select + A: Capturar Ponto  
select + Y: Apagar Ponto  
select + R1: Salvar Rota  
select + L1/L2: Apagar Rotas" --> S3
    end

    %% --- Estado Inicial e Ações Globais ---
    [*] --> S1: Iniciar Nó
    S1 -- "Movimento do Joystick" --> S1
    S2 -- "Movimento do Joystick" --> S2

    subgraph "Ações Globais (Disponíveis em qualquer estado)"
        direction LR
        A1(Qualquer Estado)
        A1 -- "select + analógico esquerdo  
(Mudar Velocidade)" --> A1
        A1 -- "select + analógico direito  
(Desligar Robô)" --> Fim([Desligado])
    end


```