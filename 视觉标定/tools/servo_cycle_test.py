#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import rclpy
import yaml
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.node import Node


MAV_CMD_DO_SET_SERVO = 183
ACTION_INTERVAL_S = 2.0


class ServoCycleTest(Node):
    def __init__(self, channels, stowed_pwm, release_pwm, durations, namespace) -> None:
        super().__init__("vision_servo_cycle_test")
        self.channels = channels
        self.stowed_pwm = stowed_pwm
        self.release_pwm = release_pwm
        self.durations = durations
        self.test_sequence = (0, 1, 0, 1)
        namespace = namespace.rstrip("/")
        self.command_service = namespace + "/cmd/command"
        self.fcu_state = State()
        self.state_received = False
        self.create_subscription(State, namespace + "/state", self._state_callback, 10)
        self.command_client = self.create_client(CommandLong, self.command_service)

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
            raise RuntimeError(f"等待服务 {self.command_service} 超时")

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
        for channel, pwm in zip(self.channels, self.stowed_pwm):
            try:
                self.set_servo(channel, pwm)
            except Exception as error:
                self.get_logger().error(f"收回 CH{channel} 失败：{error}")

    def run_test(self) -> None:
        self.wait_for_command_service()
        self.wait_for_fcu()

        self.get_logger().info("初始化：两个舵机移动到配置的收回位置")
        for channel, pwm in zip(self.channels, self.stowed_pwm):
            self.set_servo(channel, pwm)
        self.wait(1.0)

        for index, payload_index in enumerate(self.test_sequence, start=1):
            channel = self.channels[payload_index]
            release_pwm = self.release_pwm[payload_index]
            stowed_pwm = self.stowed_pwm[payload_index]
            duration = self.durations[payload_index]
            action_started = time.monotonic()
            self.get_logger().info(
                f"动作 {index}/4：CH{channel} 投放，PWM {release_pwm:.0f}，保持 {duration:.2f} 秒"
            )
            self.set_servo(channel, release_pwm)
            self.wait(duration)
            self.set_servo(channel, stowed_pwm)
            remaining = ACTION_INTERVAL_S - (time.monotonic() - action_started)
            if index < len(self.test_sequence) and remaining > 0.0:
                self.wait(remaining)

        self.get_logger().info("舵机测试完成：两个配置通道各投放两次")


def load_payload_config(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    payloads = data["payloads"]
    fields = {
        "channels": [int(value) for value in payloads["channels"]],
        "stowed_pwm": [float(value) for value in payloads["stowed_pwm"]],
        "release_pwm": [float(value) for value in payloads["release_pwm"]],
        "durations": [float(value) for value in payloads["release_duration_s"]],
    }
    if any(len(values) != 2 for values in fields.values()):
        raise ValueError("aircraft.yaml must define exactly two payloads")
    return fields


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--aircraft", type=Path, default=Path("config/aircraft.yaml"))
    parser.add_argument("--namespace", default="/mavros")
    args = parser.parse_args()
    config = load_payload_config(args.aircraft)
    rclpy.init()
    node = ServoCycleTest(namespace=args.namespace, **config)
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
