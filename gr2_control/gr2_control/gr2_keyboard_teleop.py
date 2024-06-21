import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class GR2TeleopNode(Node):

    def __init__(self):
        super().__init__('gr2_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop node has been started. Use W, A, S, D to control, and Q to quit.')

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption('GR2 Teleop Control')
        
        twist = Twist()

        running = True
        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        twist.linear.x = 0.5
                        twist.angular.z = 0.0
                    elif event.key == pygame.K_s:
                        twist.linear.x = -0.5
                        twist.angular.z = 0.0
                    elif event.key == pygame.K_a:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5
                    elif event.key == pygame.K_d:
                        twist.linear.x = 0.0
                        twist.angular.z = -0.5
                    elif event.key == pygame.K_q:
                        running = False
                elif event.type == pygame.KEYUP:
                    if event.key in (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d):
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0

            self.publisher_.publish(twist)
            self.get_logger().info(f'Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

        self.get_logger().info('Shutting down teleop node.')
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = GR2TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
