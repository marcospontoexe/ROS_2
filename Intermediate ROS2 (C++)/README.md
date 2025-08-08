# LogInfo
O que é **launch.actions.LogInfo()** ?

No arquivo **launch** podemos  usar um recurso de ação de inicialização chamado **LogInfo**:

```python
# This is to publish messages inside Launch files.
message_info = launch.actions.LogInfo(msg=str(rviz_config_dir))

...

return LaunchDescription(
        [
            ...,
            message_info
        ]
    )
```

O LogInfo permite que você publique uma mensagem dentro do sistema de registro ROS2 a partir de um arquivo de inicialização. 

Neste caso, a mensagem a ser impressa é o caminho completo para o arquivo de configuração do Rviz. Isso pode ser útil para verificar se estamos carregando o arquivo de configuração correto.

Para imprimir uma mensagem durante a inicialização:

* crie uma variável LogInfo, com o argumento (msg='a mensagem em string que você deseja imprimir').
* e então forneça essa variável para LaunchDescription.