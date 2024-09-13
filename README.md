# kit_camera_wrapper

Esse é o pacote ROS do Kit de Robótica dedicado à transformar as informações de câmera em mensagens ROS.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_camera_wrapper.git
```

### Rosdep
Esse pacote foi desenvolvido para a versão ROS 2 Jazzy. Uma vez que o ROS estiver devidamente instalado e inicializado, as dependencias especificas desse pacote podem ser instaladas através do rosdep.

Rosdep é um meta-package manager, utilizado para facilitar a instalação de dependencias. Apesar do nome com base histórica, essa ferramenta se tornou independente do ROS e deve ser instalada separadamente:

```bash
sudo apt install python3-rosdep
```

Uma vez instalado, ele deve ser inicializado e atualizado:

```bash
sudo rosdep init
rosdep update
```

Por fim, para esse repositorio, vamos utilizar o rosdep para instalar as dependencias listadas no arquivo `package.xml`. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/kit_camera_wrapper
rosdep install --from-paths . -y --ignore-src
```

### Colcon
Inicialize o seu ROS workspace e compile ele utilizando a ferramenta `colcon`. Mais uma vez, é necessario substituir o endereço e nome do seu próprio ROS workspace abaixo:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_camera_wrapper
```
Fique de olho em possiveis erros nesse processo e realize debug se necessario.

### PiCamera
Esse pacote pega a camera padrão do dispositivo, instanciada em `/dev/ttyUSB0`. Para que a PiCamera seja reconhecida e possa ser acessada nesse endereço, é necessario seguir as seguintes instruções:

- Atualiza o sistema
```bash
sudo apt update
sudo apt upgrade
```
- Edite o arquivo `/boot/firmware/config.txt`, adicionando a seguinte entrada no final do arquivo:
```bash
start_x=1
```
- Reinicie o sistema
```bash
sudo reboot
```

## Configurações
As configurações desse pacote são feitas através de ROS `params`. Os arquivos que descrevem as configurações, que podem ser editados manualmente, se encontram na pasta `config`.
Lembre-se sempre de recompilar o pacote depois de fazer mudanças na configuração, com:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_camera_wrapper
```

## Launch
Para iniciar o programa `camera_node`, responsavel por transformar a imagem capturada em `/dev/ttyUSB0` em uma mensagem ROS, basta utilizar o seguinte comando:
```bash
ros2 launch kit_camera_wrapper camera_launch.py
```