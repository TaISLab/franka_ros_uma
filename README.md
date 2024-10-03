# ROS integration for Franka Robotics research robots

Este metapaquete contiene las modificaciones realizadas al metapaquete franka_ros dentro del proyecto RAFI.

Para consultar la documentación del paquete original:
https://frankaemika.github.io/docs/index.html

## Instalación de la versión modificada:

Las siguientes instrucciones de instalación son para Ubuntu 20.04 LTS y ROS Noetic.


### Requisitos previos:

Instalación de dependencias necesarias
```bash
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```

Desistala versiones previas de libfranka:
```bash
sudo apt remove "*libfranka*"
```

### Instalación de libfranka:
```bash
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
cd ~/libfranka
```

Crea un directorio de construcción y compila:
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
### Instalación de franka_ros_rafi. Versión de franka_ros modificada.

Creación del workspace (si todavía no está creado):

```bash
mkdir -p ~/workspace/rafi_ws_test/src
cd ~/workspace/rafi_ws_test
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
```

Clonar repositorio de franka_ros_rafi modificado:
```bash
git clone --recursive https://github.com/rodri-castro/franka_ros_rafi.git src/franka_ros_rafi
```

Instalar dependencias y cosntruir los paquetes:
```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

Configurar el entorno:
```bash
source devel/setup.sh
```
## Configuración del real-time kernel.

Sigue el procedimiento descrito en https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel.





