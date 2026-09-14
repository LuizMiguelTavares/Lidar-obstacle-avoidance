# Pioneer Obstacle Avoidance com LiDAR

Pacote ROS 1 para rodar um experimento de desvio de obstaculos com um robo Pioneer diferencial e LiDAR 2D. O pacote publica uma rota circular, estima o contorno do robo, calcula um potencial de desvio usando `/RosAria/scan` e envia comandos de velocidade para `/RosAria/cmd_vel`.

## Ambiente Esperado

- ROS 1 Noetic no Ubuntu 20.04
- Python 3 com `numpy`, `scipy`, `matplotlib`, `numba` e `PyQt5`
- `RosAria` publicando `/RosAria/scan` e `/RosAria/pose`
- Opcionalmente, `natnet_ros` publicando `/natnet_ros/P1/pose`

Este pacote agora define a propria mensagem `ObjectPoints`, entao o pacote antigo `obstacle_avoidance_drone_follower` nao e mais necessario para essa mensagem.

## Build

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:LuizMiguelTavares/Lidar-obstacle-avoidance.git lidar_obs_avoidance

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
chmod +x src/lidar_obs_avoidance/scripts/*.py
catkin_make
source devel/setup.bash
```

## Rodar

Use o launch indoor quando a pose vier do `natnet_ros`:

```bash
roslaunch lidar_obstacle_avoidance pioneer_indoor.launch
```

Use o launch outdoor quando a pose vier da odometria do RosAria:

```bash
roslaunch lidar_obstacle_avoidance pioneer_outdor.launch
```

## Topicos Principais

- Assina `/RosAria/scan` (`sensor_msgs/LaserScan`)
- Assina `/RosAria/pose` (`nav_msgs/Odometry`) no modo outdoor
- Assina `/natnet_ros/P1/pose` (`geometry_msgs/PoseStamped`) no modo indoor
- Publica `/RosAria/cmd_vel` (`geometry_msgs/Twist`)
- Publica `/P1/route`, `/P1/points` e `/P1/potential`
- Publica `/emergency_flag` a partir do botao de emergencia em PyQt

## Observacoes

- O botao de emergencia abre uma janela grafica, entao precisa de sessao grafica ou X11 forwarding.
- `plot_2D.py` tambem abre uma janela do matplotlib.
- Confira se os topicos do robo e do LiDAR estao ativos antes de iniciar o experimento completo:

```bash
rostopic echo -n1 /RosAria/scan
rostopic echo -n1 /RosAria/pose
```
