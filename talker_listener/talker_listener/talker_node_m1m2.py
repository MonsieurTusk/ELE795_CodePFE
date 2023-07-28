import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

angle_moteur1_min = 70.0
angle_moteur1_max = 130.0

angle_moteur2_min = 70.0
angle_moteur2_max = 110.0

direction_positive = 1
direction_negative = 0

pas_moteur = 0.2
time_publish_sec = 0.01

class TalkerNode_moteur1(Node):
    def __init__(self):
        super().__init__("talkerNode_m1")
        self.publisher_1 = self.create_publisher(Float32, '/topic_moteur1', 10) #Attribuer le bon topic a celui voulu
        self.publisher_2 = self.create_publisher(Float32, '/topic_moteur2', 10) #Attribuer le bon topic a celui voulu
        timer_period = time_publish_sec #le mettre a une seconde (avant : 0.01s)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle_1 = angle_moteur1_min
        self.angle_2 = angle_moteur2_min

        self.direction_1 = 1
        self.direction_2 = 1

    def timer_callback(self):
        msg_m1 = Float32()
        msg_m1.data = self.angle_1
        self.publisher_1.publish(msg_m1)
        self.get_logger().info('Valeur moteur 1 envoyee: "%f"' % self.angle_1)

        msg_m2 = Float32()
        msg_m2.data = self.angle_2
        self.publisher_2.publish(msg_m2)
        self.get_logger().info('Valeur moteur 2 envoyee: "%f"' % self.angle_2)

        if self.angle_1 <= angle_moteur1_min: #Debute l'activation du moteur dans une direction a partir d'un angle
            self.direction_1 = direction_positive
            
            if self.direction_2 == direction_positive:
                self.angle_2 += pas_moteur
            
            if self.direction_2 == direction_negative:
                self.angle_2 -= pas_moteur
        
        if self.angle_1 >= angle_moteur1_max: #Changement de direction du moteur une fois l'angle maximale atteint
            self.direction_1 = direction_negative

            if self.direction_2 == direction_positive:
                self.angle_2 += pas_moteur
            
            if self.direction_2 == direction_negative:
                self.angle_2 -= pas_moteur

        if self.direction_1 == direction_positive: #le moteur va dans une direction
            self.angle_1 += pas_moteur

        if self.direction_1 == direction_negative: #Changement de direction du moteurs
            self.angle_1 -= pas_moteur

        if self.angle_2 <= angle_moteur2_min:
            self.direction_2 = direction_positive
        
        if self.angle_2 >= angle_moteur2_max:
            self.direction_2 = direction_negative

        


def main(args=None):
    rclpy.init(args=args)

    #create node
    talkerNode_m1 = TalkerNode_moteur1()

    #use node 
    rclpy.spin(talkerNode_m1)

    #destroy node
    talkerNode_m1.destroy_node()


    rclpy.shutdown()

if __name__ == "__name__":
    main()