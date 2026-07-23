import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'

# 시리얼 통신 속도(baudrate)
BAUDRATE = 115200
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, port=PORT, baudrate=BAUDRATE):
    super().__init__('serial_sender_node')

    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('port', port)
    self.declare_parameter('baudrate', baudrate)

    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.port = self.get_parameter('port').get_parameter_value().string_value
    self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

    # 포트가 없거나 권한이 없어도 노드가 죽지 않도록 예외 처리 후 재연결을 시도한다.
    self.ser = None
    self._open_serial()

    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             depth=1)

    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def _open_serial(self):
    """시리얼 포트를 (재)연결한다. 실패해도 예외를 던지지 않고 None으로 둔다."""
    try:
      self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
      time.sleep(1)
      self.get_logger().info(f"Serial port opened: {self.port} @ {self.baudrate}")
    except serial.SerialException as e:
      self.ser = None
      self.get_logger().error(f"Failed to open serial port {self.port}: {e}")

  def send(self, steering, left_speed, right_speed):
    """조향/속도 명령을 시리얼로 전송한다. 실패 시 재연결을 시도한다."""
    serial_msg = PCFL.convert_serial_message(steering, left_speed, right_speed)
    if self.ser is None:
      self._open_serial()
    if self.ser is None:
      return
    try:
      self.ser.write(serial_msg.encode())
    except serial.SerialException as e:
      self.get_logger().warn(f"Serial write failed, will retry to reconnect: {e}")
      try:
        self.ser.close()
      except Exception:
        pass
      self.ser = None

  def data_callback(self, msg):
    self.send(msg.steering, msg.left_speed, msg.right_speed)

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)

  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      # 종료 시 정지 명령을 전송하여 차량이 멈추도록 한다.
      node.send(0, 0, 0)

  finally:
    if node.ser is not None:
      node.ser.close()
      print('closed')

  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
