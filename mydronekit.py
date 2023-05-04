from collections.abc import MutableMapping
from socket import MSG_CMSG_CLOEXEC
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rclpy.node import Node


class CameraNode(Node):
    def __init__(self, callback):
        super().__init__('camera_node')
        self.get_logger().info('Iniciando o nó da câmera...')
        self.image_publisher = self.create_publisher(Image, '/camera_image_raw', 10)
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
          # Conecta ao meu drone
        self.vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, heartbeat_timeout=10)
        try:
            self.vehicle.wait_ready('autopilot_version')
        except Exception as e:
            print('Erro ao conectar-se ao veículo:', e)


        # Decola o drone
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        self.vehicle.simple_takeoff(0.10)
        print("Decolagem iniciada..")


        if self.vehicle.location.global_relative_frame.alt >= 0.10:
            print("Altitude desejada alcançada")
        time.sleep(1)

#função de callback para o tópico de imagem
    def image_callback(self, msg: Image):
        print("Identificando camera..")
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
 
    # os valores para as cores
        verde = (0, 255, 0)
        azul = (0, 0, 255)
        rosa = (238, 130, 238)
        vermelho = (255, 0, 0)

    # Aplica os filtros para encontrar as cores
        mask_verde = cv2.inRange(image, verde, verde)
        print("Filtro verde aplicado")
        mask_azul = cv2.inRange(image, azul, azul)
        print("Filtro azul aplicado")
        mask_rosa = cv2.inRange(image, rosa, rosa)
        print("Filtro rosa aplicado")
        mask_vermelho = cv2.inRange(image, vermelho, vermelho)
        print("Filtro vermelho aplicado")

    # Encontra os contornos das cores
        contours_verde, _ = cv2.findContours(mask_verde, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Contornos verdes encontrados")
        contours_azul, _ = cv2.findContours(mask_azul, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Contornos azuis encontrados")
        contours_rosa, _ = cv2.findContours(mask_rosa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Contornos rosas encontrados")
        contours_vermelho, _ = cv2.findContours(mask_vermelho, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Contornos vermelhos encontrados")

    # Verifica se as cores foram encontradas
        if len(contours_verde) > 0:
            print("verde detectado!")
        # para frente
            self.vehicle.simple_goto(self.vehicle.location.global_relative_frame + LocationGlobalRelative(0, 5, 0))
        elif len(contours_azul) > 0:
            print("azul detectado!")
        # para a direita
            self.vehicle.simple_goto(self.vehicle.location.global_relative_frame + LocationGlobalRelative(5, 0, 0))
        elif len(contours_rosa) > 0:
            print("rosa detectado!")
        # pra trás
            self.vehicle.simple_goto(self.vehicle.location.global_relative_frame + LocationGlobalRelative(0, -5, 0))
        elif len(contours_vermelho) > 0:
            print("vermelho detectado!")
        # para a esquerda
            self.vehicle.simple_goto(self.vehicle.location.global_relative_frame + LocationGlobalRelative(-5, 0, 0))
        else:
            print("Nenhuma cor detectada")
        self.vehicle.mode = VehicleMode("LAND")


# Encerre o nó quando terminar de execu
def main(args=None):
   rclpy.init(args=args)
# vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, heartbeat_timeout=10)
   node =CameraNode(callback=None)
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

