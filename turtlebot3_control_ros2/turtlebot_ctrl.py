#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random
import time

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")

        # Inicializa as variáveis para guardar os dados do laser e da odometria
        self.laser = LaserScan()
        self.odom = Odometry()

        # Mapa discretizado do ambiente, onde 0 é obstáculo, 1 é área não explorada e 2 é área explorada
        self.map = np.array([
            # Mapa fixo de 20x20, onde o robô deve explorar as áreas marcadas com 1
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])

        # Publicador para o tópico /cmd_vel para enviar comandos de velocidade ao robô
        self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # Inscrições nos tópicos para receber dados de odometria e laser scan
        self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)

        # Timer para chamar a função cmd_vel_pub periodicamente (a cada 0,5 segundos)
        self.timer = self.create_timer(0.5, self.cmd_vel_pub)

        # Armazena o tempo de início da exploração
        self.start_time = time.time()

    # Função para encontrar o ponto mais próximo na lista de pontos fornecidos
    def nearby_points(self, current_pos, points):
        # Calcula a distância euclidiana entre a posição atual e todos os pontos
        distance = np.sqrt((points[:, 0] - current_pos[0])**2 + (points[:, 1] - current_pos[1])**2)
        # Encontra o índice do ponto mais próximo
        indexp = np.argmin(distance)
        return points[indexp]

    # Função chamada periodicamente para controlar o movimento do robô
    def cmd_vel_pub(self):
        # Resolução do mapa (cada célula representa 0,25m)
        map_resolution = 4

        # Calcula o índice do robô no mapa discreto baseado em sua posição no ambiente
        index_x = -int(self.odom.pose.pose.position.x * map_resolution)
        index_y = -int(self.odom.pose.pose.position.y * map_resolution)

        # Ajusta os índices para se referirem ao centro do mapa
        index_x += int(self.map.shape[0] / 2)
        index_y += int(self.map.shape[0] / 2)

        # Garante que o robô permaneça dentro dos limites do mapa
        if index_x < 1: index_x = 1
        if index_x > self.map.shape[0] - 1: index_x = self.map.shape[0] - 1
        if index_y < 1: index_y = 1
        if index_y > self.map.shape[0] - 1: index_y = self.map.shape[0] - 1

        # Marca a célula como explorada (2) se o robô passar por uma célula não explorada (1)
        if self.map[index_x][index_y] == 1:
            self.map[index_x][index_y] = 2

            # Calcula a porcentagem do mapa explorada até o momento
            percentage = 100 * float(np.count_nonzero(self.map == 2)) / (np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))
            elapsed_time = time.time() - self.start_time  # Calcula o tempo decorrido

            # Exibe no console informações sobre o progresso da exploração e o mapa atual
            self.get_logger().info("Another part reached ... percentage total reached: {:.2f}% - Time elapsed: {:.2f} seconds".format(percentage, elapsed_time))
            self.get_logger().info("Discrete Map")
            self.get_logger().info("\n" + str(self.map))

        # Busca as células não exploradas (com valor 1) no mapa
        points = np.argwhere(self.map == 1)

        # Se existirem pontos não explorados
        if points.size > 0:
            # Posição atual do robô
            pos_atual = np.array([index_x, index_y])

            # Encontra o ponto mais próximo do robô
            nearby_points = self.nearby_points(pos_atual, points)

            # Calcula a distância e o ângulo entre o robô e o ponto mais próximo
            distance_x = nearby_points[0] - pos_atual[0]
            distance_y = nearby_points[1] - pos_atual[1]
            angle_close = np.arctan2(distance_y, distance_x)
            distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

            # Inicializa a mensagem de velocidade
            msg = Twist()

            # Obtém as leituras do laser nas direções esquerda e direita
            left_laser = self.laser.ranges[-30:]
            right_laser= self.laser.ranges[:30]

            if len(left_laser) <= 0 or len(right_laser) <= 0:
                return

            # Evita obstáculos se houver algo próximo à direita ou à esquerda
            if min(right_laser) < random.uniform(0.3, 0.5):  
                msg.linear.x = 0.0  # Para de andar para frente
                msg.angular.z = -0.4  # Gira
            elif min(left_laser) < random.uniform(0.3, 0.5):
                msg.linear.x = 0.0
                msg.angular.z = 0.4  # Gira
            else:
                # Se a distância ao ponto mais próximo for grande, anda em direção a ele
                if distance > 0.1:
                    msg.linear.x = min(0.3 * distance, 0.3)  # Limita a velocidade linear a 0.3
                    msg.angular.z = 0.3 * angle_close  # Ajusta a rotação para alinhar com o alvo
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0  # Para o robô quando estiver próximo ao ponto

            self.publish_cmd_vel.publish(msg)

    # Função de callback que recebe dados do laser scan
    def callback_laser(self, msg):
        self.laser = msg

    # Função de callback que recebe dados de odometria
    def callback_odom(self, msg):
        self.odom = msg

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)  # Mantém o nó em execução
    rclpy.shutdown()  # Finaliza o nó quando interrompido

if __name__ == "__main__":
    main()
