import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10
        )
        
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        self.velocity = Twist()
        
        # Definição da distância para desvio de obstaculo
        self.min_safe_distance = 0.5  # 0.5 metros
        self.obstacle_detected = False

    def lidar_callback(self, msg):
        """Callback para processar os dados do Lidar"""
        # essa parte define o angulo frontal de detecção de obstaculo
        front_angles = msg.ranges[0:12] + msg.ranges[-12:]  # 12 graus de cada lado
        
        front_distance = min(front_angles)

        if front_distance < self.min_safe_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def move_turtlebot(self):
        """Função para mover o robô"""
        if self.obstacle_detected:
            # Se obstáculo detectado, gira para a direita
            self.velocity.linear.x = 0.0  # Parar de avançar
            self.velocity.angular.z = 0.7  # Girar
        else:
            # Se não houver obstáculo, continua em frente
            self.velocity.linear.x = 0.5  # Velocidade de movimento
            self.velocity.angular.z = 0.0  # <-- coloque um valor caso queira uma rotação nessas condições

        self.publisher_.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBotController()
    
    try:
        rclpy.spin(turtlebot_controller)
    except KeyboardInterrupt:
        pass

    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
