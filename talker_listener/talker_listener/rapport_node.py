import rclpy
import struct
import numpy as np
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2



class rapportNode(Node):
    def __init__(self):
        super().__init__('rapport_node')
        self.subscription = self.create_subscription( PointCloud2, '/points2', self.points_callback, 10)
        
        self.sum_z = 0.0
        self.count = 0

        self.msg_x = 0.0 
        self.msg_y = 0.0
        self.msg_z = 0.0
        self.msg_z_min = None
        self.msg_z_max = None
        self.msg_z_moy = 0.0
    
        self.msg_x0 = None
        self.msg_y0 = None
        self.distance_2points       = -1.0
        self.distance_2points_min   = None
        self.distance_2points_max   = None
        self.distance_2points_moy   = 0.0
        self.sum_distance_2points   = 0.0
        self.count_distance_2points = 0

    def points_callback(self, msg):
        # Vérifier si le message contient bien des points
        if len(msg.data) == 0:
            return

        # Lire les coordonnées x, y, z
        self.msg_x, self.msg_y, self.msg_z = struct.unpack_from('fff', msg.data)

        #Calcul de la moyenne de z
        self.sum_z += self.msg_z 
        self.count += 1

        if self.count > 0:
            self.msg_z_moy = self.sum_z / self.count
            #self.get_logger().info('Average Z value: %f' % self.msg_z_moy)

        #Calcul maximum z
        if self.msg_z_max is None:
            self.msg_z_max = self.msg_z
        elif self.msg_z_max < self.msg_z:
            self.msg_z_max = self.msg_z
        #Calcul minimum z
        if self.msg_z_min is None:
            self.msg_z_min = self.msg_z
        elif self.msg_z_min > self.msg_z:
            self.msg_z_min = self.msg_z

        #Calcul distance min max et moyenne entre 2 points
        if self.msg_x0 is not None and self.msg_y0 is not None:
            if self.msg_x0 == self.msg_x:
                self.distance_2points = np.abs(self.msg_y0 - self.msg_y)
            elif self.msg_y0 == self.msg_y:
                self.distance_2points = np.abs(self.msg_x0 - self.msg_x)
            else:  
                self.distance_2points = -1.0
        
            if self.distance_2points != -1.0:
                #Moyenne distance entre 2 points
                self.sum_distance_2points += self.distance_2points
                self.count_distance_2points += 1
                if self.count_distance_2points > 0:
                    self.distance_2points_moy = self.sum_distance_2points / self.count_distance_2points
                #Max distance entre 2 points
                if self.distance_2points_max is None:
                    self.distance_2points_max = self.distance_2points
                elif self.distance_2points_max < self.distance_2points:
                    self.distance_2points_max = self.distance_2points
                #Min distance entre 2 points
                if self.distance_2points_min is None:
                    self.distance_2points_min = self.distance_2points
                elif self.distance_2points_min > self.distance_2points:
                    self.distance_2points_min = self.distance_2points
        
        self.msg_x0 = self.msg_x
        self.msg_y0 = self.msg_y
        print("--------------------------------------------------------------------")
        print("Z min :", "{:.2f}".format(self.msg_z_min), " | ",
              "Z max :", "{:.2f}".format(self.msg_z_max), " | ",
              "Z moy :", "{:.2f}".format(self.msg_z_moy), " | ",
              "Z count :", "{:.2f}".format(self.count))
        if self.distance_2points_min is not None:
            print("D min :", "{:.2f}".format(self.distance_2points_min), " | ",
                "D max :", "{:.2f}".format(self.distance_2points_max), " | ",
                "D moy :", "{:.2f}".format(self.distance_2points_moy), " | ",
                "D count :", "{:.2f}".format(self.count_distance_2points))

        #Générer un rapport PDF de préférence avec les données des print et un en-tête
        #Trouver un moyen pour mettre une capteur de la fenetre de rviz dans le PDF

def main(args=None):
    rclpy.init(args=args)
    node = rapportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()