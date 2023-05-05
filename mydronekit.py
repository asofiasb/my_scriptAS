from collections.abc import MutableMapping
from socket import MSG_CMSG_CLOEXEC
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rclpy.node import Node
import math
from pymavlink import mavutil


class CameraNode(Node):
    def __init__(self, image_callback):
        super().__init__('camera_node')
        self.get_logger().info('Iniciando o nó da câmera...')
        self.image_publisher = self.create_publisher(Image, '/camera_image_raw', 10)
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Conecta ao meu drone
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True, heartbeat_timeout=60)
        try:
            self.vehicle.wait_ready('autopilot_version')
        except Exception as e:
            print('Erro ao conectar-se ao veículo:', e)

        # Decola o drone
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        self.vehicle.simple_takeoff(0.10)
        print("Decolagem iniciada..")

        while self.vehicle.location.global_relative_frame.alt < 0.15:
            print("Aguardando decolagem... Altitude atual: ", self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)

        print("Altitude desejada alcançada.")

    def goto_position_target_local_ned(self, north, east, down):
        current_location = self.vehicle.location.global_relative_frame
        target_location = self.get_location_metres(current_location, north, east, down)
        target_distance = self.get_distance_metres(current_location, target_location)
        bearing = self.get_bearing(current_location, target_location)

        # Estabelecer o objeto Target-Position-Local-NED
        target_local_ned = self.LocationLocal(north=north, east=east, down=down)

        # Estabelecer o objeto Target-Position-Global-INT
        target_global_int = self.get_location_metres(
            current_location,
            target_local_ned.north,
            target_local_ned.east,
            -target_local_ned.down
        )

        # Setar os parâmetros para o objeto Target-Position-Target-Global-INT
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only positions enabled)
            0, 0, 0,  # x, y, z positions (not used)
            north, east, -down,  # north, east, down velocity (m/s)
            0, 0, 0,  # x, y, z acceleration
            0, 0  # yaw, yaw_rate
        )

        # Enviar o comando para o veículo
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        # Monitorar a mudança de posição
        while True:
            remaining_distance = self.get_distance_metres(self.vehicle.location.global_relative_frame, target_global_int)

            if remaining_distance <= 0.05 * target_distance:
                break

            time.sleep(0.5)



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

    # Verifica se as cores foram encontradas

    #pra frente
    if len(mask_verde) > 0:
        print("verde detectado!")
        self.goto_position_target_local_ned(5, 0, 0)

    #pra direita
    elif len(mask_azul) > 0:
        print("azul detectado!")
        self.goto_position_target_local_ned(0, 5, 0)

    # pra trás
    elif len(mask_rosa) > 0:
        print("rosa detectado!")
        self.goto_position_target_local_ned(-5, 0, 0)

    # para a esquerda
    elif len(mask_vermelho) > 0:
        print("vermelho detectado!")
        self.goto_position_target_local_ned(0, 0, 5)

    else:
          ###?????? verde dnv??????
          print("Nenhuma cor detectada")
          self.vehicle.mode = VehicleMode("LAND")



# Encerre o nó quando terminar de executar
def main(args=None):
   rclpy.init(args=args)
   node =CameraNode(image_callback)
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
