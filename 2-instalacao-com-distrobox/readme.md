# Como instalar o ros2 no distrobox

nesse tutorial primeiro vamos seguir o passo a passo para criar uma distrobox com base da documentação oficial.

https://distrobox.it/

como no VRI vivemos a vida perigosamente, iremos seguir a sugestão da documentação de instalar o distrobox usando curl.

- curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh

precisaremos também instalar o podman, por favor siga a documentação

https://podman.io/docs/installation#installing-on-linux

então crie uma distrobox

distrobox create --name ros_box --init --image ubuntu:22.04 --additional-packages "systemd" && distrobox enter ros_box

pode demorar um pouco...

# instalando ros no distrobox 

para instalar o ros no distrobox, é exatamente o mesmo processo para instalar no ubuntu 22.04 LTS, porém há algumas considerações. O distrobox utiliza os arquivos de configuração do seu sistema, inclusive o bashrc, que utilizamos para setar o ambiente automaticamente sempre que abrirmos um novo terminal. Então se você adicionar as linhas no .bashrc, sempre que abrir um novo terminal haverá mensagens de erro:
"bash: /opt/ros/humble/setup.bash: Arquivo ou diretório inexistente
not found: "/opt/ros/humble/local_setup.bash"
bash: /opt/ros/humble/setup.bash: Arquivo ou diretório inexistente"

para contornar esse problema podemos colocar em uma estrutura condicional, que verifique se pelo menos um desses arquivos existe antes de executar os comandos.


