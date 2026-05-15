import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 키 매핑: (직진 방향, 회전 방향)
move_bindings = {
    'w': (1.5, 0.0),
    's': (-1.5, 0.0),
    'a': (0.0, 1.5),
    'd': (0.0, -1.5),
    ' ': (0.0, 0.0),
    'x': (0.0, 0.0)
}

msg = """
로봇 키보드 조작을 시작합니다!
---------------------------
w : 전진
s : 후진
a : 좌회전
d : 우회전
space 또는 x : 정지

Ctrl-C 로 종료합니다.
---------------------------
"""

def get_key(settings):
    # 터미널을 raw 모드로 전환하여 입력을 즉시 읽음
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.2       # 전후진 기본 속도 (m/s)
        self.turn_speed = 0.5  # 회전 기본 속도 (rad/s)

    def publish_command(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear * self.speed
        cmd.angular.z = angular * self.turn_speed
        self.publisher_.publish(cmd)


def main(args=None):
    # 실행 당시 터미널 설정 저장
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = RobotController()
    
    print(msg)
    
    linear_val = 0.0
    angular_val = 0.0
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            # 지정된 키 입력 시 처리
            if key in move_bindings.keys():
                linear_val = move_bindings[key][0]
                angular_val = move_bindings[key][1]
                node.publish_command(linear_val, angular_val)
            # 종료 (Ctrl + C) 처리
            elif key == '\x03': 
                break
                
            rclpy.spin_once(node, timeout_sec=0.0)
            
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        # 종료 전 정지 명령 전송
        node.publish_command(0.0, 0.0)
        # 터미널 설정 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
