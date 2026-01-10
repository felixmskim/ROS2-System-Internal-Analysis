import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Start to Draw a Square!')
        self.draw_square()

    def move_straight(self, speed, duration):
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop_turtle()

    def turn(self, angular_speed, duration):
        msg = Twist()
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop_turtle()

    def stop_turtle(self):
        msg = Twist()
        self.publisher_.publish(msg)
        time.sleep(1) # 동작 간의 간격

    def draw_square(self):
        for i in range(4):
            self.get_logger().info(f'{i+1}번째 변 그리는 중...')
            self.move_straight(2.0, 2.0)  # 2.0 속도로 2초간 전진
            self.turn(1.5708, 1.0)        # 약 90도(PI/2) 회전
        self.get_logger().info('사각형 완성!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDrawer()
    # 사각형 그리기가 끝나면 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
