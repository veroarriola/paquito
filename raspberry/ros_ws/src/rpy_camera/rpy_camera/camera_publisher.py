import rclpy
from rclpy.node import Node

import io
from picamera2 import Picamera2
from libcamera import controls
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from threading import Condition

from sensor_msgs.msg import Image


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class CameraPublisher(Node):
    # Selected sensor format: 1536x864-SBGGR10_1X10 - Selected unicam format: 1536x864-pBAA
    HEIGHT = 1536
    WIDTH = 864
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # Editor que publica la imagen
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)

        # Configuración de PiCamera2
        self.picam2 = picam2 = Picamera2()

        # Ajusta la resolución
        conf = picam2.create_video_configuration(
            main={
                'format': 'RGB888',
                'size': (CameraPublisher.HEIGHT, CameraPublisher.WIDTH)
            }
        )
        picam2.configure(conf)
        picam2.start()

        #output = StreamingOutput()
        #picam2.start_recording(JpegEncoder(), FileOutput(output))
        #picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.AfSpeedEnum.Fast})

        # Cronómetro para capturar y publicar (30 FPS aprox)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('Nodo editor de la cámara iniciado')

    def timer_callback(self):
        # Capturamos un cuadro
        frame = self.picam2.capture_array()
        #print(type(frame))

        if frame is not None:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            #msg.header.frame_id = "camera_link"
            msg.height = CameraPublisher.HEIGHT
            msg.width = CameraPublisher.WIDTH
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = CameraPublisher.WIDTH * 3    # Full row length in bytes = ancho * canales

            # Convertimos el array de NumPy a bytes puros para el mensaje
            msg.data = frame.tobytes()

            self.publisher_.publish(msg)

    def destroy_node(self):
        self.picam2.stop()
        # picam2.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)    
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo nodo cámara...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# from cv_bridge import CvBridge
# import numpy as np

# class CameraPublisher(Node):
#     def __init__(self):
#         super().__init__('camera_publisher_node')
        
#         # Publicador de la imagen
#         self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
#         # Inicializamos CvBridge para convertir de OpenCV a ROS2
#         self.br = CvBridge()


#     def timer_callback(self):
#         # Capturamos un frame
#         frame = self.picam2.capture_array()

#         if frame is not None:
#             # Convertimos el array a un mensaje de imagen de ROS2
#             # Usamos 'bgr8' porque es el estándar en procesamiento de imagen
#             msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "camera_link"
            
#             # Publicamos
#             self.publisher_.publish(msg)

