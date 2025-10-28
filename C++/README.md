# Índice

1.  [C++](#c)
    *   [Pacotes compilados](#pacotes-compilados)
    *   [INCLUDE](#include)
    *   [namespace](#namespace)
        *   [namespace std](#namespace-std)
    *   [return 0](#return-0)
    *   [break](#break)
    *   [continue](#continue)
    *   [Listas](#listas)
    *   [Dicionários](#dicionários)
    *   [Arrays](#arrays)
    *   [Saída de dados na tela](#saida-de-dados-na-tela)
        *   [printf](#printf)
        *   [cout](#cout)
    *   [Ponteiro](#ponteiro)
        *   [Usando ponteiro em funções](#usando-ponteiro-em-funções)

# C++
Programas em C++ podem ser um pouco complicados de executar. Primeiramente, para um programa escrito em C++, o nome do arquivo precisa ser algo como **nome.cpp**, onde a extensão .cpp especifica a linguagem de programação que estamos usando.

Em segundo lugar, precisamos de um compilador para compilar o código (que seria **g++**), e também de um nome para o código compilado, por exemplo **nome_compilado**, sem extensão.

Por exemplo: `g++ -std=c++11 name.cpp -o name_compiled`.

Com este comando, estamos dizendo ao compilador **g++** para usar sua versão **c++11**, pegar o código em **name.cpp** e compilá-lo em um arquivo executável chamado **name_compiled**.

Para ver o código funcionando, precisamos apenas chamar o arquivo executável digitando: `./name_compiled`.

## Pacotes compilados
Ao compilar o espaço de trabaçhp (work space) ROS, com os comandos:
* `cd ~/catkin_ws`
* `catkin_make`
* `source devel/setup.bash`

os programas internos (pasta src) são compilados automaticamente.
Para executar um código_qualquer pertecente a um pacote_qualquer, execute o comando: `rosrun pacote_qualquer código_qualquer`.

## INCLUDE
A diretiva **#include** é uma diretiva do pré-processador que informa ao compilador para incluir o conteúdo do arquivo de cabeçalho especificado no arquivo de código-fonte.

Observe que, ao incluir arquivos de cabeçalho, usamos a sintaxe **" " (aspas)** ou **< > (colchetes angulares)**. 
* Usar **< >** informa ao pré-processador para pesquisar o local usual para cabeçalhos de sistema a fim de encontrar o cabeçalho necessário.
*  usar **" "** informa ao pré-processador para pesquisar primeiro o diretório onde o arquivo atual está localizado antes de pesquisar o caminho usual para cabeçalhos de sistema.

## namespace
C++ possui um recurso específico chamado namespaces. Eles nos permitem agrupar entidades nomeadas em um único escopo. Dessa forma, elas só terão significado dentro desse namespace. Além disso, podemos repetir nomes de variáveis, mas sempre dentro de namespaces diferentes.

### namespace std
Esta linha significa que podemos usar o código no **namespace std** sem digitar **std::** antes dele. Por exemplo, podemos escrever **cout** em vez de **std::cout**, alem de outras variáveis, como *string*, *cin*, *endl*...

## return 0
Em um programa C++, return 0; é usado para **indicar a conclusão bem-sucedida do programa**.

Lembre-se de que a função principal de um programa é definida como **int main()**, o que significa que se espera que ela retorne um valor inteiro. O valor inteiro retornado pela função principal é chamado de status de saída ou código de retorno do programa. **return 0**; indica **sucesso** e **return 1; ou outros números para falhas**.

## break
Ele encerra imediatamente um loop por completo. A execução do programa prossegue para a primeira instrução após o corpo do loop.

## continue
Ele encerra imediatamente a iteração atual do loop. A execução salta para o topo do loop e a condição é reavaliada para determinar se o loop será executado novamente ou encerrado.

## Listas
Em C++, listas são sequências ordenadas de variáveis ​​do mesmo tipo. Para inicializá-las, precisamos especificar de que tipo são as variáveis ​​dentro delas. Por exemplo, para uma lista de string: `list<string> vocals_list( {"a","e","i","o","u"} );`

Aqui está um exemplo de impressão de uma lista:

```c++
for (int val : numbers_list)             // Loop
    cout << val << "  ";                 // Print function


for (string val : vocals_list)           // Loop
    cout << val << "  ";                 // Print function
```

* Adicione um novo item no início da lista: `numbers_list.push_front(0);`          
* Adicione um novo item no final da lista: `numbers_list.push_back(3000);`
* Concatenar uma lista no final de outra, ampliando a primeira e não excluindo a segunda, com a função interna insert():

```c++
list<int> new_list({5,50,500});

numbers_list.insert(numbers_list.end(),new_list.begin(),new_list.end());
```

## Dicionários
Um dicionário em C++ é chamado de mapa e é um contêiner de valores indexados por uma chave. Isso significa que ele armazena dois tipos de informação: **chaves** e **valores**.

Para inicializá-lo, precisamos chamar **map** e especificar os **tipos de dados** das chaves e **valores**.

Aqui estamos criando um dicionário chamado **girls_dictionary**, onde as chaves são **strings** e os valores são **inteiros**. `map<string,int> girls_dictionary;`.

Para inserir dados neste dicionário, podemos chamar cada chave e atribuir a ela um valor, um por um:
```c++
girls_dictionary["Dolores"] = 30;
girls_dictionary["Maeve"] = 27;
girls_dictionary["Theresa"] = 6;
girls_dictionary["Clementine"] = 11;

for (auto item : girls_dictionary)    // para imprimir 
    cout << item.first << " appears in " << item.second << " episodes\n";
```

## Arrays
Um array é uma série de elementos do mesmo tipo, colocados em locais de memória contíguos. Para inicializá-los, precisamos especificar de **que tipo são as variáveis ​​dentro** dele e **qual será o tamanho da sequência**. Por exemplo, para um array de valores inteiros com 6 números:`int myarray[6];`.

Assim como com outras variáveis, podemos inicializá-las e atribuir valores na mesma linha: `int myarray[6] = {4,8,15,16,23,42};` ou `int myarray[] = {4,8,15,16,23,42};`.

Se quisermos imprimir o terceiro valor do array: `cout << myarray[2] << endl;`.

Também podemos imprimir todos os elementos de um array com um loop for:

```c++
for (auto items : myarray) {
    cout << items << endl;
}
```

## Saida de dados na tela

### printf
Precisa do tipo da variável a ser impressa. Por exemplo, se quisermos imprimir um **inteiro**, especificamos com o símbolo **%i**, se for um **float** com **%f** e se for uma **string** com **%s**. No caso específico de imprimir uma string, ela precisa ser convertida em um caractere pela função **c_str()**.

Finalmente, também precisa ser acompanhada pela inclusão da classe **iostream**:
```c++
#include <iostream>
using namespace std;

int main(){
    
    int a = 42;
    printf("Value a is %i \n",a);            // Print an integer
    float b = 3.1415;
    printf("Value a is %f \n",b);            // Print a float
    string word = "Hey you!";
    printf("- %s \n",word.c_str());           // Print a string
    
    return 0;}
```

### cout
É um objeto que se refere ao Character OUTput (COUT), que precisa ser acompanhado pela inclusão da classe **iostream**, a importação do namespace **std** e o símbolo **<<**.
```c++
#include <iostream>
using namespace std;

int main(){
    
    cout << "Are you enjoying this course? \n";     // Print a sentence
    string answer = "Of course!";
    cout << "Answer is " << answer;                  // Print a variable
    
    return 0;}
```

## Ponteiro
Para descobrir o endereço de memória da variável **myvariable** podemos usar o operador **&** antes da variável: `cout << "myvariable's address is" << &myvariable << endl;`

O tipo de variável que pode armazenar o endereço de uma variável é chamado de ponteiro. **Ponteiros** podem ser declarados com o operador **`*`**, especificando o tipo de variável para a qual apontarão: `int* mypointer;`.

Se quisermos que este ponteiro armazene o endereço da nossa variável anterior myvariable, precisamos apenas dar a ela o valor do seu endereço: `mypointer = &myvariable;`.

Agora que a relação entre myvariable e mypointer foi estabelecida, podemos acessar o valor original de myvariable usando apenas seu ponteiro: `cout << *mypointer << endl;`.

### Usando ponteiro em funções
Quando passamos o valor de um array como parametros de entrada para uma função, um array local é criado dentro da função e recebe os valores do array fornecido para a função. Imagine que esse array tem muitos índices, para evitar alto consumo de memória, podemos passar o ponteiro do array para a função:

```c++
#include <iostream>
using namespace std;

void increment(int &number) {
    cout << "The address of number is: " << &number << endl;
    number++;
    cout << "Now the value of number is: " << number << endl;
}

int main() {
    
    // Declare the number
    int number = 42;
    // Print the number's value
    cout << " The value of number is: " << number << " and the address is: " << &number << endl;
    // Call the function increment
    increment(number);
    
    
    return 0;
}
```

Podemos também passar pontieros constantes (`const int* pointer;`). Dessa forma, podemos garantir que essas funções não modifiquem nossos parâmetros.

Em C++, **funções NÃO PODEM retornar arrays**. Simplesmente não é possível. O que elas PODEM fazer é retornar um ponteiro para um array:
```c++
float *RosbotClass::get_laser_full() {
  float *laser_range_pointer = laser_range.data();
  return laser_range_pointer;
}
```

Esta função, chamada get_laser_full(), retorna um ponteiro (veja o operador asterisco antes do nome da classe RosbotClass). O que ela faz é pegar os valores de um vetor ROS chamado laser_range e atribuir a eles um ponteiro, retornando esse ponteiro.

outro exemplo: 

```c++
double *getVel() {
  // a function that gets the current linear and angular velocities of the robot.
  double *velocities = new double[2];
  velocities[0] = robot_interface->linear_velocity;
  velocities[1] = robot_interface->angular_velocity;
  return velocities;  // retorna o ponteiro
}
```

A função **getVel()** cria um ponteiro para um array de duas posições, cada posição recebe o valor de uma variável.
