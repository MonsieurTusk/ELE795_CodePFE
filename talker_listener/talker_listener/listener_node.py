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

        self.subscription = self.create_subscription(
            Float32,
            '/topic_lidar',  # Attribuer le bon topic à celui voulu
            self.listener_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(PointCloud2, '/points2', 10)  # Attribuer le bon topic à celui voulu
        timer_period = 0.01  # le mettre à une seconde (avant : 0.01s)
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def listener_callback(self, msg):
        self.get_logger().info(f"Received {msg.data}")
        self.msg_lidar = msg.data  # Stocker la valeur msg.data dans msg_lidar

    def timer_callback(self):
        if self.msg_lidar is not None:
            msg_pc2 = PointCloud2()
            msg_pc2 = self.publishPC2(self.msg_lidar)
            self.publisher_.publish(msg_pc2)
            self.get_logger().info('Valeur POINTCLOUD envoyee')


    def publishPC2(self, count):
        #x, y = np.meshgrid(np.linspace(-2, 2, width), np.linspace(-2, 2, height))
        x = 10.0
        y = 20.0
        z = 30.0  # 0.5 * np.sin(2 * x - count / 10.0) * np.sin(2 * y)

        # Convertir x, y et z en tableaux NumPy avant d'appeler flatten()
        #x = np.array(x)
        #y = np.array(y)
        #z = np.array(z)
        
        #points = np.array([x.flatten(), y.flatten(), z.flatten()]).T
        points = np.array([(x,y,z)], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])  # Point 3 avec coordonnées XYZ et couleur RGB
        
        # Créer le message PointCloud2
        header = Header()
        header.frame_id = "base_link"  # Indiquez ici le nom du frame_id approprié
        #header.stamp = self.get_clock().now().to_msg()
        pcl_msg = PointCloud2()
        pcl_msg.header = header
        pcl_msg.height = 1
        pcl_msg.width = len(points)
        pcl_msg.is_dense = False
        pcl_msg.is_bigendian = False
        
        fields = [
               PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
               PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
               PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
           ]

        #header = Header()
        #header.frame_id = "map"



        # Créer un nuage de points avec les coordonnées x, y, z
        #points = [(float(x_), float(y_), float(z_)) for x_, y_, z_ in zip(x.flatten(), y.flatten(), z.flatten())]
        
        #pc2 = PointCloud2()
        #pc2 = PointCloud2.create_cloud_xyz32(header, [x], [y], [z])
        #pc2 = PointCloud2.create_cloud_xyz32(header, points)
        
        #pc2 = sensor_msgs.point_cloud2.create_cloud_xyz32(header,fields, points)
        #pc2.header = header
        #pc2.height = 1
        #pc2.width = len(points)
        #pc2.is_bigendian = False
        #pc2.is_dense = False
        #pc2.fields = fields
        #pc2.point_step = len(points[0]) * 3 #Correction de 4 a 3
        #pc2.row_step = pc2.point_step * pc2.width
        #pc2.data = points.tobytes()

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
