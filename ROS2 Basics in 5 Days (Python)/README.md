# Pacotes
O ROS 2 utiliza pacotes para organizar seus programas. Você pode pensar em um pacote como uma coleção de todos os arquivos relacionados a um programa ROS 2 específico; todos os seus arquivos CPP, arquivos Python, arquivos de configuração, arquivos de compilação, arquivos de inicialização e arquivos de parâmetros. Além disso, organizar seus programas ROS 2 em pacotes facilita muito o compartilhamento com outros desenvolvedores/usuários.

No ROS2, você pode criar dois tipos de pacotes:

* Pacotes Python
* Pacotes CMake (C++)

Neste curso, vamos nos concentrar no primeiro tipo. Os pacotes Python conterão executáveis Python.

Cada pacote Python terá a seguinte estrutura de arquivos e pastas:

* **package.xml** - Arquivo contendo meta-informações sobre o pacote (mantenedor do pacote, dependências, etc.).
* **setup.py** - Arquivo contendo instruções sobre como compilar o pacote.
* **setup.cfg** - Contém instruções sobre como instalar o pacote.
* src/\<nome_do_pacote> - Este diretório recebe o nome do seu pacote. Você colocará todos os seus scripts Python dentro desta pasta. Ela já contém um arquivo __init__.py vazio por padrão.

![2_3_ros2_package_no_title]()