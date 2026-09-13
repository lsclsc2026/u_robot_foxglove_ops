#!/usr/bin/env python3
import math
import time
from mos_hal import ArmControlMode, ChasisControlMode, MosHal, Jointstate, JointCmd
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

WHEElCMDVEL_TOPIC  ='/cmd_vel' #'/wheel_cmd_vel'
# WHEEL_SEPARATION = 0.5   
# WHEEL_RADIUS     = 0.1   
WHEELBASE  = 0.54474

class CmdVelController(Node):
    """订阅 cmd_vel 并控制底盘的 ROS2 节点"""

    def __init__(self, mos_hal: MosHal):
        super().__init__('cmd_vel_controller')
        self.mos_hal = mos_hal

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('cmd_vel 控制器已启动，等待速度指令...')

    def cmd_vel_callback(self, msg: Twist):
        # current_time = self.get_clock().now()
        # sec = current_time.nanoseconds / 1e9
        # nsec = current_time.nanoseconds
        
        # self.get_logger().info(f"当前时间戳  s: {sec:.6f}")
        # self.get_logger().info(f"当前时间戳 ns: {nsec}")

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # print("linear_x is: ", linear_x, "angular_z is: ", angular_z)

        left_wheel_linear = linear_x - 0.5 * WHEELBASE * angular_z   #左轮线速度+
        right_wheel_linear = linear_x + 0.5 * WHEELBASE * angular_z   #右轮线速度

        left_wheel_angular = 11.11 * left_wheel_linear   #左轮角速度  11.11 为 1/R
        right_wheel_angular = -11.11 * right_wheel_linear   #右轮角速度

        # print("left_wheel_angular is: ", vel[0], "right_wheel_angular is: ", vel[1])

        cmd = JointCmd()
        cmd.name     = ["right_wheel_joint", "left_wheel_joint"]
        cmd.velocity = [right_wheel_angular, left_wheel_angular]   # 右轮在前，左轮在后
        cmd.position = [0.0, 0.0]
        cmd.torque   = [0.0, 0.0]

        # 下发指令
        self.mos_hal.setRobotCmd(cmd)

        self.get_logger().info(
            f'cmd_vel: v={linear_x:.2f} m/s, ω={angular_z:.2f} rad/s → '
            f'左轮={left_wheel_angular:.2f} rad/s, 右轮={right_wheel_angular:.2f} rad/s'
        )


def main():
    # 初始化 ROS2
    rclpy.init()

    # 加载 MosHal 配置并启动
    config = "/opt/lumos/config/mos_hal/mos_p15.yaml"
    mos_hal = MosHal(config)

    # 设置底盘控制模式（如 PI 控制）
    mos_hal.set_chasis_control_mode(ChasisControlMode.PI)

    if not mos_hal.start():
        print("✗ MosHal start failed")
        rclpy.shutdown()
        return

    # 打印初始关节状态（方便调试）
    state = mos_hal.get_robot_state()
    print("=== Motor State ===")
    for name, pos, vel, eff in zip(state.name, state.position, state.velocity, state.torque):
        print(f"{name:20s}  pos={pos:.4f} vel={vel:.4f} torque={eff:.4f}")
    print("*******************************")

    # 创建 cmd_vel 控制器节点
    controller = CmdVelController(mos_hal)

    try:
        rclpy.spin(controller)          # 阻塞等待回调
    except KeyboardInterrupt:
        print("\nCtrl+C 捕获，正在安全停止...")
    finally:
        # 发送零速度指令让机器人停止
        stop_cmd = JointCmd()
        stop_cmd.name     = ["right_wheel_joint", "left_wheel_joint"]
        stop_cmd.velocity = [0.0, 0.0]
        stop_cmd.position = [0.0, 0.0]
        stop_cmd.torque   = [0.0, 0.0]
        mos_hal.setRobotCmd(stop_cmd)
        time.sleep(0.1)                 # 等待指令确实发出

        # 清理资源
        mos_hal.stop()
        controller.destroy_node()
        rclpy.shutdown()
        print("已安全退出")


if __name__ == "__main__":
    main()
