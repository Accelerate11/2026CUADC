#!/usr/bin/env python3

import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.node import Node


COMMAND_SERVICE = "/mavros/cmd/command"
MAV_CMD_DO_SET_SERVO = 183
STOWED_PWM = 1100.0
RELEASE_PWM = 1900.0
RELEASE_DURATION_S = 0.7
ACTION_INTERVAL_S = 2.0
TEST_SEQUENCE = (7, 8, 7, 8)


class ServoCycleTest(Node):
    def __init__(self) -> None:
        super().__init__("cuadc_servo_cycle_test")
        self.fcu_state = State()
        self.state_received = False
        self.create_subscription(State, "/mavros/state", self._state_callback, 10)
        self.command_client = self.create_client(CommandLong, COMMAND_SERVICE)

    def _state_callback(self, message: State) -> None:
        self.fcu_state = message
        self.state_received = True

    def wait_for_fcu(self, timeout_s: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state_received and self.fcu_state.connected:
                if self.fcu_state.armed:
                    raise RuntimeError("飞控处于解锁状态，拒绝执行地面舵机测试")
                self.get_logger().info("飞控已连接且未解锁")
                return
        raise RuntimeError("等待飞控连接超时")

    def wait_for_command_service(self, timeout_s: float = 20.0) -> None:
        if not self.command_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"等待服务 {COMMAND_SERVICE} 超时")

    def set_servo(self, channel: int, pwm: float) -> None:
        if self.fcu_state.armed:
            raise RuntimeError("测试期间飞控被解锁，立即停止")

        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(channel)
        request.param2 = pwm

        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            raise RuntimeError(f"CH{channel} PWM={pwm:.0f} 命令应答超时")
        response = future.result()
        if response is None or not response.success:
            result = "无应答" if response is None else str(response.result)
            raise RuntimeError(
                f"CH{channel} PWM={pwm:.0f} 命令被拒绝，result={result}"
            )
        self.get_logger().info(f"CH{channel} PWM={pwm:.0f} 已确认")

    def wait(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
            if self.fcu_state.armed:
                raise RuntimeError("测试期间飞控被解锁，立即停止")

    def stow_all(self) -> None:
        for channel in (7, 8):
            try:
                self.set_servo(channel, STOWED_PWM)
            except Exception as error:
                self.get_logger().error(f"收回 CH{channel} 失败：{error}")

    def run_test(self) -> None:
        self.wait_for_command_service()
        self.wait_for_fcu()

        self.get_logger().info("初始化：CH7/CH8 收回到 PWM 1100")
        self.set_servo(7, STOWED_PWM)
        self.set_servo(8, STOWED_PWM)
        self.wait(1.0)

        for index, channel in enumerate(TEST_SEQUENCE, start=1):
            action_started = time.monotonic()
            self.get_logger().info(
                f"动作 {index}/4：CH{channel} 投放，PWM 1900，保持 0.7 秒"
            )
            self.set_servo(channel, RELEASE_PWM)
            self.wait(RELEASE_DURATION_S)
            self.set_servo(channel, STOWED_PWM)
            remaining = ACTION_INTERVAL_S - (time.monotonic() - action_started)
            if index < len(TEST_SEQUENCE) and remaining > 0.0:
                self.wait(remaining)

        self.get_logger().info("舵机测试完成：CH7 和 CH8 各投放两次")


def main() -> int:
    rclpy.init()
    node = ServoCycleTest()
    status = 0
    try:
        node.run_test()
    except KeyboardInterrupt:
        node.get_logger().warning("收到中断，正在收回舵机")
        status = 130
    except Exception as error:
        node.get_logger().error(f"舵机测试失败：{error}")
        status = 1
    finally:
        if node.state_received and node.fcu_state.connected and not node.fcu_state.armed:
            node.stow_all()
        node.destroy_node()
        rclpy.shutdown()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
