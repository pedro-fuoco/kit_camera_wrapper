#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(Node):

    def __init__(self):
        # Inicializa o node ROS com o nome 'camera_wrapper'
        super().__init__('camera_wrapper')
        
        # Cria os publicadores para enviar imagens e informações da câmera
        # Os nomes dos tópicos seguem os padrões encontrados em http://wiki.ros.org/ROS/Patterns/Conventions
        # A fila dos topicos é limitada em 10 elementos... Isso será relevante caso os subscritores sejam mais lentos que os publicadores
        self.image_publisher = self.create_publisher(Image, 'kit/camera/image_raw', 10)
        self.info_publisher = self.create_publisher(CameraInfo, 'kit/camera/camera_info', 10)
        
        # Declara os ROS params com valores padrão
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('FPS', 15)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        # Obtém os valores dos ROS params, caso o node tenha sido inicializado com valores customizados
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        fps = self.get_parameter('FPS').get_parameter_value().integer_value
        frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value

        # Loga os parâmetros iniciais da câmera para o usuario
        self.get_logger().info(f'Camera initialized with Index: {camera_index}, FPS: {fps}, Width: {frame_width}, Height: {frame_height}')

        # Cria um timer para garantir que a imagem será amostrada no periodo de 1/fps segundos
        self.timer = self.create_timer(1/fps, self.timer_callback)
        
        # Inicializa a captura de vídeo
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            # Caso não seja possível abrir a camera, esse node será inútil...
            # Ele deve ser desligado para poupar recursos.
            self.get_logger().error("Failed to open camera!")
            rclpy.shutdown()
        else:
            self.get_logger().info("Camera initialized successfully")
        
        # Configura FPS, largura e altura do frame para a câmera
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        # Instancia o adaptador que que converte imagens OpenCV para mensagens ROS
        # Estamos utilizando o CvBridge, um pacote ROS Open Source
        self.bridge = CvBridge()

        # Prepara a mensagem de informações da câmera
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_link'
        self.camera_info.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_info.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Caso algum tipo de distorção seja aplicado na imagem da câmera, ela deve ser descrita aqui
        self.camera_info.distortion_model = 'none'

    # Função de callback do timer
    def timer_callback(self):
        # Lê um frame do objeto de captura
        ret, frame = self.cap.read()
        if not ret:
            # Loga um aviso se a captura da imagem falhar
            # Perder um frame não é catastrófico para o node, então ele não será desinicializado
            self.get_logger().warn("Failed to capture image")
            return

        # Converte o frame capturado em uma mensagem de imagem ROS
        image_msg = self.bridge.cv2_to_imgmsg(frame)
        # Popula o header da mensagem ROS da imagem
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_link'
        # Publica a mensagem da imagem
        self.image_publisher.publish(image_msg)

        # Popula o header da mensagem ROS de especificações da camera
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        # Publica a mensagem das especificações da camera
        self.info_publisher.publish(self.camera_info)

    def __del__(self):
        # Libera os recursos da camera caso esse objeto seja destruido
        # Seguindo principios de "Resource acquisition is initialization" (RAII)
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_wrapper = CameraNode()

    try:
        # Mantém o nó ativo para continuar capturando e publicando imagens
        rclpy.spin(camera_wrapper)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        camera_wrapper.get_logger().info('Shutting down camera driver...')
    finally:
        # Destrói o node e finaliza o ROS 2
        camera_wrapper.destroy_node()
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()