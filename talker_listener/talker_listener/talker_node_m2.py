import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class TalkerNode_moteur2(Node):
    def __init__(self):
        super().__init__("talkerNode_m2")
        self.publisher_ = self.create_publisher(Float32, '/topic_moteur2', 10) #Attribuer le bon topic a celui voulu
        timer_period = 0.01 #le mettre a une seconde
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 30.0
        self.direction = 1

    def timer_callback(self):
        msg = Float32()
        msg.data = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info('Valeur moteur 2 envoyee: "%f"' % self.angle)

        if self.angle <= 60: #Debute l'activation du moteur dans une direction a partir d'un angle
            self.direction = 1
        
        if self.angle >= 120: #Changement de direction du moteur une fois l'angle maximale atteint
            self.direction = 0

        if self.direction == 1: #le moteur va dans une direction
            self.angle += 0.1

        if self.direction == 0: #Changement de direction du moteurs
            self.angle -= 0.1

def main(args=None):
    rclpy.init(args=args)

    #create node
    talkerNode_m2 = TalkerNode_moteur2()


    #use node 
    rclpy.spin(talkerNode_m2)

    #destroy node
    talkerNode_m2.destroy_node()

    rclpy.shutdown()


if __name__ == "__name__":
    main()
