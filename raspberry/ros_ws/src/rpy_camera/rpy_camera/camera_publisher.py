# import rclpy
# from rclpy.node import Node

# import io
# from picamera2 import Picamera2
# from libcamera import controls
# from picamera2.encoders import JpegEncoder
# from picamera2.outputs import FileOutput
# from threading import Condition

# from sensor_msgs.msg import Image


# class StreamingOutput(io.BufferedIOBase):
#     def __init__(self):
#         self.frame = None
#         self.condition = Condition()

#     def write(self, buf):
#         with self.condition:
#             self.frame = buf
#             self.condition.notify_all()


# class CameraPublisher(Node):
#     def __init__(self, picam):
#         super().__init__('camera_publisher')
        

    


# def main(args=None):
#     rclpy.init(args=args)

#     picam2 = Picamera2()
#     conf = picam2.create_video_configuration(main={"size": (1280, 720)})
#     picam2.configure(conf)
#     output = StreamingOutput()
#     picam2.start_recording(JpegEncoder(), FileOutput(output))
#     picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.
# AfSpeedEnum.Fast})
    
#     node = CameraPublisher(picam2)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Deteniendo nodo cámara...")
#     finally:
#         node.destroy_node()
#         picam2.stop_recording()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # Publicador de la imagen
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
        # Inicializamos CvBridge para convertir de OpenCV a ROS2
        self.br = CvBridge()

        # Configuración de PiCamera2
        self.picam2 = Picamera2()
        
        # Ajusta la resolución según tus necesidades (ej. 640x480 para fluidez)
        config = self.picam2.create_video_configuration(main={'format': 'XRGB8888', 'size': (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # Timer para capturar y publicar (30 FPS aprox)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('Nodo publicador de cámara iniciado')

    def timer_callback(self):
        # Capturamos un frame
        frame = self.picam2.capture_array()

        if frame is not None:
            # Convertimos el array a un mensaje de imagen de ROS2
            # Usamos 'bgr8' porque es el estándar en procesamiento de imagen
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            
            # Publicamos
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    