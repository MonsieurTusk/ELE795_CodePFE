from __future__ import print_function  # Utile pour PointCloud

import rclpy
import pcl_msgs

from rclpy.node import Node

from std_msgs.msg import Float32

# Librairie pour le fichier point cloud
import numpy as np
import sensor_msgs.msg as sensor_msgs
#import sensor_msgs.point_cloud2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


width = 100
height = 100

class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener_node")
        self.msg_lidar = None  # Initialiser msg_lidar à None
        self.msg_moteur1 = None
        self.msg_moteur2 = None

        #Ecoute du topic /topic_lidar pour obtenir les donnees du lidar
        self.subscription_lidar = self.create_subscription(Float32, '/topic_lidar', self.listener_callback_lidar, 10)

        #Ecoute du topic /topic_moteur1 pour obtenir les donnees du moteur1
        self.subscription_m1 = self.create_subscription(Float32, '/topic_moteur1', self.listener_callback_m1, 10)

        #Ecoute du topic /topic_moteur2 pour obtenir les donnees du moteur2
        self.subscription_m2 = self.create_subscription(Float32, '/topic_moteur2', self.listener_callback_m2, 10)
        
        self.publisher_ = self.create_publisher(PointCloud2, '/points2', 12)  # Attribuer le bon topic à celui voulu
        timer_period = 0.01  # le mettre à une seconde (avant : 0.01s)
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def listener_callback_lidar(self, msg):
        self.get_logger().info(f"Lidar received {msg.data}")
        self.msg_lidar = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def listener_callback_m1(self, msg):
        self.get_logger().info(f"M1 received {msg.data}")
        self.msg_moteur1 = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def listener_callback_m2(self, msg):
        self.get_logger().info(f"M2 received {msg.data}")
        self.msg_moteur2 = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def timer_callback(self):
        if self.msg_lidar is not None and self.msg_moteur1 is not None and self.msg_moteur2 is not None:
            msg_pc2 = PointCloud2()

            #Envoie des valeurs du 
            msg_pc2 = self.publishPC2(self.msg_lidar, self.msg_moteur1, self.msg_moteur2)
            self.publisher_.publish(msg_pc2)
            self.get_logger().info('Valeur POINTCLOUD envoyee')


    def publishPC2(self, lidar_data, moteur1_data, moteur2_data):
        #Initialisation des donnees
        angle_origine_m1 = 90.0
        angle_origine_m2 = 90.0

        
        #Calcul des angles pour le PointCloud
        x = lidar_data * np.sin(moteur2_data - angle_origine_m2)
        y = lidar_data * np.sin(moteur1_data - angle_origine_m1)
        z = lidar_data * np.abs(np.cos(moteur1_data - angle_origine_m1) * np.cos(moteur2_data - angle_origine_m2))
        
        points = np.array([(x,y,z)], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])  # Point 3 avec coordonnées XYZ et couleur RGB
        
        # Créer le message PointCloud2
        header = Header()
        header.frame_id = "base_link"  # Indiquez ici le nom du frame_id approprié
        #header.stamp = self.get_clock().now().to_msg()
        pcl_msg = PointCloud2()
        pcl_msg.header = header
        pcl_msg.height = 1
        pcl_msg.width = 1
        pcl_msg.is_dense = False
        pcl_msg.is_bigendian = False
        pcl_msg.point_step = 12 #3 * 4 bytes
        pcl_msg.row_step = 12
        
        fields = [
               PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
               PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
               PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
           ]

        # Créer un nuage de points avec les coordonnées x, y, z
        #points = [(float(x_), float(y_), float(z_)) for x_, y_, z_ in zip(x.flatten(), y.flatten(), z.flatten())]

        pcl_msg.fields = fields

        # Convertir les coordonnées XYZ et couleurs RGB en bytes et les affecter au nuage de points
        pcl_msg.data = points.tobytes()

        return pcl_msg


def main(args=None):
    rclpy.init(args=args)

    # create node
    listenerNode = ListenerNode()

    # use node
    rclpy.spin(listenerNode)

    # destroy node
    listenerNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()  # Sert a faire rouler les noeuds et les commandes de reception
