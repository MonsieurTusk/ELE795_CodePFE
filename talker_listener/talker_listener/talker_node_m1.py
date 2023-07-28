import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class TalkerNode_moteur1(Node):
    def __init__(self):
        super().__init__("talkerNode_m1")
        self.publisher_ = self.create_publisher(Float32, '/topic_moteur1', 10) #Attribuer le bon topic a celui voulu
        timer_period = 0.01 #le mettre a une seconde (avant : 0.01s)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 30.0
        self.direction = 1

    def timer_callback(self):
        msg = Float32()
        msg.data = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info('Valeur moteur 1 envoyee: "%f"' % self.angle)

        if self.angle <= 70: #Debute l'activation du moteur dans une direction a partir d'un angle
            self.direction = 1
        
        if self.angle >= 130: #Changement de direction du moteur une fois l'angle maximale atteint
            self.direction = 0

        if self.direction == 1: #le moteur va dans une direction
            self.angle += 0.1

        if self.direction == 0: #Changement de direction du moteurs
            self.angle -= 0.1

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
