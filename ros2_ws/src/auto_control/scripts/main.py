import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from piracer.vehicles import PiRacerPro

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.piracer = PiRacerPro()
        self.get_logger().info("✅ MainNode has been started.")

        # 수동모드와 자동모드에 대한 상태 초기화
        self.is_auto = 0  # 0은 수동모드, 1은 자동모드

        # 수동 조작을 위한 변수 (실수형으로 초기화)
        self.manual_steer = 0.0
        self.manual_throttle = 0.0
        
        # 오토모드에서 구독할 변수 (실수형으로 초기화)
        self.steer = 0.0
        self.throttle = 0.0

        # ⭐ 해결책 1: 버튼의 '이전 상태'를 저장할 변수 추가
        self.last_buttons = None

        # 수동 조작을 위한 joy 토픽 구독
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 오토모드에서 구독할 steer, throttle 퍼블리셔 구독
        self.steer_sub = self.create_subscription(Float32, '/auto_steer', self.steer_callback, 10)
        self.throttle_sub = self.create_subscription(Float32, '/auto_throttle', self.throttle_callback, 10)

        # 타이머 설정 (0.1초 -> 0.05초로 변경하여 더 부드러운 제어)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def joy_callback(self, joy_msg):
        # 최초 실행 시 last_buttons를 현재 버튼 상태로 초기화
        if self.last_buttons is None:
            self.last_buttons = joy_msg.buttons
            return

        # [B] 버튼이 '눌리는 순간'(0->1)에 자동모드로 전환
        if joy_msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            self.is_auto = 1
            self.get_logger().info("Mode switched to 🧠 AUTO")
            # 모드 전환 시 안전을 위해 차량 정지
            self.piracer.set_steering_percent(0.0)
            self.piracer.set_throttle_percent(0.0)

        # [A] 버튼이 '눌리는 순간'(0->1)에 수동모드로 전환
        elif joy_msg.buttons[0] == 1 and self.last_buttons[0] == 0:
            self.is_auto = 0
            self.get_logger().info("Mode switched to 🕹️ MANUAL")
            # 모드 전환 시 안전을 위해 차량 정지
            self.piracer.set_steering_percent(0.0)
            self.piracer.set_throttle_percent(0.0)
        
        # 수동모드일 때만 조향/스로틀 값 업데이트
        if self.is_auto == 0:
            self.manual_steer = joy_msg.axes[0]  # 조향 (왼쪽 스틱 좌우)
            self.manual_throttle = -joy_msg.axes[4] * 0.2 # 스로틀 (오른쪽 스틱 상하)
        
        # 다음 비교를 위해 현재 버튼 상태를 '이전 상태'로 저장
        self.last_buttons = joy_msg.buttons

    def steer_callback(self, msg):
        if self.is_auto == 1:
            self.steer = msg.data

    def throttle_callback(self, msg):
        if self.is_auto == 1:
            self.throttle = msg.data

    def timer_callback(self):
        if self.is_auto == 1:
            self.get_logger().info(f"AUTO Mode -> Throttle: {self.throttle:.2f}, Steer: {self.steer:.2f}", throttle_duration_sec=1.0)
            self.piracer.set_steering_percent(self.steer)
            self.piracer.set_throttle_percent(self.throttle)
        else:
            self.get_logger().info(f"MANUAL Mode -> Throttle: {self.manual_throttle:.2f}, Steer: {self.manual_steer:.2f}", throttle_duration_sec=1.0)
            self.piracer.set_steering_percent(self.manual_steer)
            self.piracer.set_throttle_percent(self.manual_throttle)

    def cleanup(self):
        """노드 종료 시 차량을 안전하게 정지시키는 함수"""
        self.get_logger().info("Stopping the vehicle...")
        self.piracer.set_throttle_percent(0.0)
        self.piracer.set_steering_percent(0.0)

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C 로 종료 시 cleanup 함수를 호출하여 안전하게 정지
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()