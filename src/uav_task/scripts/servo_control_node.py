#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # 使用标准浮点数消息类型来传递角度

import Jetson.GPIO as GPIO
import time

# --- 配置 (和你的原程序一样) ---
SERVO_PIN = 32      # 物理引脚号
PWM_FREQ = 50       # 50Hz

# --- 角度舵机参数 ---
MIN_ANGLE = 0
MAX_ANGLE = 180
MIN_DUTY_CYCLE = 2.5
MAX_DUTY_CYCLE = 12.5

def angle_to_duty_cycle(angle=90):
    """将角度 (0 到 180) 转换为 PWM 占空比"""
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    duty_cycle_range = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE
    duty_cycle = MIN_DUTY_CYCLE + (angle / MAX_ANGLE) * duty_cycle_range
    return duty_cycle

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        self.get_logger().info('--- Python 舵机控制节点已启动 ---')

        # 创建一个订阅者，监听名为 "/servo/set_angle" 的话题
        self.subscription = self.create_subscription(
            Float32,
            '/servo/set_angle',
            self.angle_callback,
            10)
        
        # 初始化 GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        
        # 创建并启动 PWM 实例
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(0) # 启动PWM，初始占空比为0
        self.get_logger().info(f"GPIO {SERVO_PIN} 已配置为PWM输出。")

    def angle_callback(self, msg):
        """接收到角度命令后的回调函数"""
        target_angle = msg.data
        self.get_logger().info(f'接收到新角度命令: {target_angle:.1f} 度')
        
        # 计算并设置占空比
        dc = angle_to_duty_cycle(target_angle)
        self.pwm.ChangeDutyCycle(dc)
        self.get_logger().info(f"舵机角度已设置为: {target_angle:.1f} 度 (占空比: {dc:.2f}%)")

    def cleanup(self):
        """节点关闭时清理GPIO资源"""
        self.get_logger().info("正在停止PWM并清理GPIO...")
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("GPIO清理完毕。")


def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoControllerNode()
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        servo_node.get_logger().info('检测到Ctrl+C，正在关闭节点...')
    finally:
        # 确保在退出时调用清理函数
        servo_node.cleanup()
        servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()