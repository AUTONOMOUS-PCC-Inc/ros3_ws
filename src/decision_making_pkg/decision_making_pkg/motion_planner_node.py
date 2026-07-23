import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_function_lib as DMFL  # 실제 파일명은 decision_making_function_lib.py

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

#---------------조향/속도 제어 튜닝 파라미터---------------
# 아래 값들만 바꿔가며 트랙에서 실주행 튜닝하면 됩니다.
MAX_STEERING = 7          # 하드웨어가 낼 수 있는 최대 조향 (±7)
STEERING_GAIN = 0.2       # 비례(P) 게인: 조향 = GAIN × 차선 기울기(deg). 커브가 부족하면 키우고, 진동하면 줄임
STEERING_SIGN = 1         # 실제 차량이 반대로 꺾이면 -1 로 바꿔서 방향만 뒤집기 (안전 검증용)

BASE_SPEED = 100          # 직진 시 속도 (255가 최대)
MIN_SPEED = 60            # 최대 조향(급커브) 시 감속 후 속도 - 이 값과 BASE_SPEED 사이에서 조향에 비례해 감속
#--------------------------------------------------------

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):

        if self.lidar_data is not None and self.lidar_data.data is True:
            # 라이다가 장애물을 감지한 경우
            self.steering_command = 0 
            self.left_speed_command = 0 
            self.right_speed_command = 0 

        elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red' \
                and self.detection_data is not None:
            # 빨간색 신호등을 감지한 경우 (detection_data가 아직 없으면 정지 판단을 건너뜀)
            for detection in self.detection_data.detections:
                if detection.class_name=='traffic_light':
                    x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
                    x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
                    y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
                    y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

                    if y_max < 150:
                        # 신호등 위치에 따른 정지명령 결정
                        self.steering_command = 0 
                        self.left_speed_command = 0 
                        self.right_speed_command = 0
        else:
            if self.path_data is None or len(self.path_data) < 10:
                # 경로가 없거나 조향각 계산에 필요한 점(최소 10개)이 부족하면 직진 유지
                self.steering_command = 0
            else:
                # [원인1 해결] look-ahead: 범퍼 바로 앞(-10~-1)이 아니라
                # '먼 차선점(path_data[0])' 과 '차량 범퍼(path_data[-1])' 사이 기울기를 본다.
                # path_data는 y 오름차순 → [0]=가장 먼 곳(앞), [-1]=범퍼(320,179).
                far_point  = self.path_data[0]
                near_point = self.path_data[-1]
                target_slope = DMFL.calculate_slope_between_points(far_point, near_point)

                if target_slope == 'inf':
                    # 두 점의 y가 같은 예외 상황(거의 없음) → 직진 유지
                    self.steering_command = 0
                else:
                    # [원인2 해결] 부호만 보던 고정 ±3 → 기울기 크기에 비례한 P 제어.
                    # 커브가 급할수록 크게 꺾고, 살짝 휘면 살짝만 꺾는다.
                    steering = STEERING_SIGN * STEERING_GAIN * target_slope
                    steering = int(round(steering))
                    # 하드웨어 한계(±7)로 클램프
                    self.steering_command = max(-MAX_STEERING, min(MAX_STEERING, steering))

            # [원인3 해결] 커브에서 감속: |조향|이 클수록 BASE_SPEED → MIN_SPEED 로 선형 감속.
            # 직진(조향 0)이면 BASE_SPEED, 최대 조향이면 MIN_SPEED.
            speed_ratio = abs(self.steering_command) / MAX_STEERING          # 0.0 ~ 1.0
            speed = BASE_SPEED - (BASE_SPEED - MIN_SPEED) * speed_ratio
            speed = int(round(speed))
            self.left_speed_command = speed
            self.right_speed_command = speed



        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
