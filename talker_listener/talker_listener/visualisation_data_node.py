import rclpy
import struct
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2


class VisualisationNode(Node):
    def __init__(self):
        super().__init__('visualisation_node')
        self.msg_lidar = 0.0 
        self.msg_m1    = 0.0
        self.msg_m2    = 0.0
        self.msg_x     = 0.0 
        self.msg_y     = 0.0
        self.msg_z     = 0.0

        self.subscription_pc2       = self.create_subscription(PointCloud2, '/points2', self.listener_callback_points2, 10)
        self.subscription_lidar = self.create_subscription(Float32, '/topic_lidar', self.listener_callback_lidar, 10) # Attribuer le bon topic à celui voulu
        self.subscription_m1    = self.create_subscription(Float32, '/topic_moteur1', self.listener_callback_m1, 10) # Attribuer le bon topic à celui voulu
        self.subscription_m2    = self.create_subscription(Float32, '/topic_moteur2', self.listener_callback_m2, 10) # Attribuer le bon topic à celui voulu
        
        timer_period = 0.01  # le mettre à une seconde (avant : 0.01s)
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback_lidar(self, msg):
        #self.get_logger().info(f"Lidar received {msg.data}")
        self.msg_lidar = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def listener_callback_m1(self, msg):
        #self.get_logger().info(f"M1 received {msg.data}")
        self.msg_m1 = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def listener_callback_m2(self, msg):
        #self.get_logger().info(f"M2 received {msg.data}")
        self.msg_m2 = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def listener_callback_points2(self, msg):
        # Vérifier si le message contient bien des points
        if len(msg.data) == 0:
            return
        # Convertir les données binaires en une liste de points
        points_list = []
        point_step = msg.point_step
        for i in range(0, len(msg.data), point_step):
            # Lire les coordonnées x, y, z
            self.msg_x, self.msg_y, self.msg_z = struct.unpack_from('fff', msg.data, offset=i)
            points_list.append((self.msg_x, self.msg_y, self.msg_z))

        # Votre code de traitement des points ici
        #print(points_list)

    def timer_callback(self):
        if self.msg_lidar is not None and self.msg_m1 is not None and self.msg_m2 is not None:
            print (self.get_clock().now().to_msg(),
                   "Lidar :",    "{:.2f}".format(self.msg_lidar), " | ",
                   "Moteur 1 :", "{:.2f}".format(self.msg_m1), " | ",
                   "Moteur 2:",  "{:.2f}".format(self.msg_m2), " | ",
                   "X :", "{:.2f}".format(self.msg_x), " | ",
                   "Y :", "{:.2f}".format(self.msg_y), " | ",
                   "Z :", "{:.2f}".format(self.msg_z))

        

def main(args=None):
    rclpy.init(args=args)
    node = VisualisationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()