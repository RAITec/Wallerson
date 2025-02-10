import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.velocity = Twist()
        
        # Parâmetros de desvio de obstáculos
        self.min_safe_distance = 0.5 
        self.obstacle_detected = False
        self.obstacle_cleared = False
        self.obstacle_side = None  # 'left', 'right', 'front'

        # Posição atual e lista de destinos
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_positions = [
            {'x': -2.0, 'y': 0.0},
            {'x': 0.0, 'y': 2.0},
            {'x': 2.0, 'y': 0.0},
            {'x': 0.0, 'y': -2.0}
        ]
        self.position_tolerance = 0.1

        self.state = "NAVIGATE"  # Estados: NAVIGATE, AVOID_OBSTACLE, RECOVER_NAVIGATION
        self.recovery_timer = 0 

        self.current_target_index = 0

    def lidar_callback(self, msg):
        """Callback para processar os dados do Lidar"""
        # Campo de visão
        left_sector = msg.ranges[6:30]  # esquerda (~6° a 30°)
        right_sector = msg.ranges[-30:-6]  # direita (~-30° a -6°)
        front_sector = msg.ranges[-5:] + msg.ranges[0:5]  # frente (~-5° a 5°)

        min_left = min(left_sector) if left_sector else float('inf')
        min_front = min(front_sector) if front_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')

        if min_front < self.min_safe_distance:
            self.obstacle_detected = True
            self.obstacle_side = 'front'
        elif min_left < self.min_safe_distance:
            self.obstacle_detected = True
            self.obstacle_side = 'left'
        elif min_right < self.min_safe_distance:
            self.obstacle_detected = True
            self.obstacle_side = 'right'
        else:
            self.obstacle_detected = False
            self.obstacle_side = None

        self.obstacle_cleared = (
            min_front > (self.min_safe_distance + 0.2) and
            min_left > (self.min_safe_distance + 0.2) and
            min_right > (self.min_safe_distance + 0.2)
        )

    def odom_callback(self, msg):
        """Callback para atualizar a posição atual do robô"""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, theta = quat2euler(
            [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        )
        self.current_position['theta'] = theta

    def control_loop(self):
        """Máquina de estados para controlar o robô"""
        if self.state == "NAVIGATE":
            self.navigate_to_goal()
            if self.obstacle_detected:
                self.state = "AVOID_OBSTACLE"

        elif self.state == "AVOID_OBSTACLE":
            self.avoid_obstacle()
            if self.obstacle_cleared:
                self.recovery_timer = 20  # Define um tempo para retomar a navegação
                self.state = "RECOVER_NAVIGATION"

        elif self.state == "RECOVER_NAVIGATION":
            self.recover_navigation()
            self.recovery_timer -= 1
            if self.recovery_timer <= 0:
                self.state = "NAVIGATE"

    def navigate_to_goal(self):
        """Mover o robô para o próximo objetivo"""
        target = self.target_positions[self.current_target_index]

        distance_to_goal = math.sqrt(
            (target['x'] - self.current_position['x'])**2 +
            (target['y'] - self.current_position['y'])**2
        )

        if distance_to_goal > self.position_tolerance:
            angle_to_goal = math.atan2(
                target['y'] - self.current_position['y'],
                target['x'] - self.current_position['x']
            )
            
            angle_diff = angle_to_goal - self.current_position['theta']
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) 
            
            if abs(angle_diff) > 0.1:  # Se não estiver alinhado
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.7 * angle_diff
            else:
                self.velocity.linear.x = 0.5
                self.velocity.angular.z = 0.0
        else:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.get_logger().info(f"Objetivo {self.current_target_index + 1} alcançado!")

            self.current_target_index += 1
            if self.current_target_index >= len(self.target_positions):
                self.get_logger().info("Todos os objetivos foram alcançados!")
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0 
                self.publisher_.publish(self.velocity)
                return

        self.publisher_.publish(self.velocity)

    def avoid_obstacle(self):
        """Desviar de obstáculos"""
        self.velocity.linear.x = 0.0 

        if self.obstacle_side == 'left':
            self.velocity.angular.z = -0.7  # Gira para a direita
        elif self.obstacle_side == 'right':
            self.velocity.angular.z = 0.7  # Gira para a esquerda
        else:  # Obstáculo à frente
            self.velocity.angular.z = -0.7  # Padrão: girar para a direita

        self.publisher_.publish(self.velocity)

    def recover_navigation(self):
        """Permite que o robô estabilize após evitar obstáculos"""
        self.velocity.linear.x = 0.2  # Avançar lentamente
        self.velocity.angular.z = 0.0  # Manter direção
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
