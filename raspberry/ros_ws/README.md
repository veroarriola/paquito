## Para ejecutar

Cargar el _underlay_ y el _overlay_ y asignar el ```ROS_DOMAIN_ID``` según su número de robot:

```
source ~/ros2_jazzy/install/setup.bash
source ~/ros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=1
```

Para activar el nodo que escucha los comandos de velocidad:

```
ros2 run paquito_raspberry_cpp car_subscriber
```