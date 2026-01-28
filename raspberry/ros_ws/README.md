## Configuración

Para comunicar a la Raspberry con Arduino por medio del protocolo I2C, el usuario de la Raspberry debe estar en el grupo con los permisos correspondientes:

```

```

## Para ejecutar

Al iniciar la sesión, cargar el _underlay_ y el _overlay_ y asignar el ```ROS_DOMAIN_ID``` según su número de robot:

```
source ~/ros2_jazzy/install/setup.bash
source ~/ros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=1
```

Para ejecutar todos los nodos en la Raspberry:

```
ros2 launch paquito_raspberry_cpp car_launch.py
```

### Ejecución unidad por unidad

Para activar el nodo que escucha los comandos de velocidad:

```
ros2 run paquito_raspberry_cpp car_subscriber
```

Para activar el nodo que envía comandos I2C:
```
ros2 run paquito_raspberry_cpp command_executor
```