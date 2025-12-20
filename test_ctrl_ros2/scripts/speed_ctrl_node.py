#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class SpeedCtrlNode(Node):
    def __init__(self):
        super().__init__('speed_ctrl_node')
        
        # 参数配置
        self.target_joint = 'force1'
        self.limit_angle = 0.2 * math.pi  # ±0.5π
        self.speed_magnitude = 0.5        # 0.5 rad/s
        self.control_rate = 10.0          # 10 Hz
        
        # 状态变量
        self.current_pos = 0.0
        self.current_direction = 1.0      # 1.0 for positive, -1.0 for negative
        self.joint_found = False
        
        # 创建发布者：发布速度指令
        # 注意：JointGroupVelocityController 接收 Float64MultiArray
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/force1_velocity_controller/commands', 
            10
        )
        
        # 创建订阅者：订阅关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 创建定时器进行控制循环
        self.timer = self.create_timer(1.0/self.control_rate, self.timer_callback)
        
        self.get_logger().info('Speed Control Node has been started.')
        self.get_logger().info(f'Target Joint: {self.target_joint}')
        self.get_logger().info(f'Range: ±{self.limit_angle:.2f} rad')
        self.get_logger().info(f'Speed: {self.speed_magnitude} rad/s')

    def joint_state_callback(self, msg):
        try:
            # 查找目标关节在消息中的索引
            if self.target_joint in msg.name:
                index = msg.name.index(self.target_joint)
                self.current_pos = msg.position[index]
                self.joint_found = True
            else:
                # 偶尔可能收不到包含该关节的消息（虽然不太可能）
                pass
        except ValueError:
            pass

    def timer_callback(self):
        if not self.joint_found:
            self.get_logger().warn(f'Waiting for joint state of {self.target_joint}...', throttle_duration_sec=2.0)
            return

        # 简单的状态机逻辑
        # 如果当前位置超过正限位，且正在向正方向运动 -> 切换为负方向
        if self.current_pos >= self.limit_angle and self.current_direction > 0:
            self.current_direction = -1.0
            self.get_logger().info(f'Reached upper limit ({self.current_pos:.2f}), switching to negative direction.')
            
        # 如果当前位置超过负限位，且正在向负方向运动 -> 切换为正方向
        elif self.current_pos <= -self.limit_angle and self.current_direction < 0:
            self.current_direction = 1.0
            self.get_logger().info(f'Reached lower limit ({self.current_pos:.2f}), switching to positive direction.')

        # 构建并发布指令
        msg = Float64MultiArray()
        msg.data = [self.speed_magnitude * self.current_direction]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止时发送 0 速度
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0]
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
