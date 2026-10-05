#!/usr/bin/env python3

import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, StreamRate
from rclpy.node import Node


class MavlinkStreamConfigurator(Node):
    def __init__(self):
        super().__init__("mavlink_stream_configurator")
        self.client = self.create_client(
            MessageInterval, "/mavros/set_message_interval"
        )
        self.stream_client = self.create_client(
            StreamRate, "/mavros/set_stream_rate"
        )
        self.connected = False
        self.state_subscription = self.create_subscription(
            State, "/mavros/state", self.state_callback, 10
        )
        # GLOBAL_POSITION_INT（33）提供 GPS 位置和飞控罗盘航向，
        # 后者用于 MAVROS 的 global_position/compass_hdg。原 X7 的 USB 链路
        # 默认不发送该消息，因此请求以 30 Hz 输出。
        self.requests = ((33, 30.0),)
        self.max_attempts = 10

    def state_callback(self, message):
        self.connected = message.connected

    def configure(self):
        self.get_logger().info("Waiting for FCU connection")
        while rclpy.ok() and not self.connected:
            rclpy.spin_once(self)
        self.get_logger().info(
            "Waiting for MAVROS message interval service"
        )
        self.client.wait_for_service()
        self.stream_client.wait_for_service()
        stream_request = StreamRate.Request()
        stream_request.stream_id = StreamRate.Request.STREAM_POSITION
        stream_request.message_rate = 30
        stream_request.on_off = True
        stream_future = self.stream_client.call_async(stream_request)
        rclpy.spin_until_future_complete(self, stream_future)
        if stream_future.result() is None:
            self.get_logger().warning("FCU rejected STREAM_POSITION at 30 Hz")
        else:
            self.get_logger().info("STREAM_POSITION configured at 30 Hz")
        for message_id, message_rate in self.requests:
            configured = False
            for attempt in range(1, self.max_attempts + 1):
                request = MessageInterval.Request()
                request.message_id = message_id
                request.message_rate = message_rate
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response is not None and response.success:
                    configured = True
                    self.get_logger().info(
                        f"MAVLink message {message_id} configured at "
                        f"{message_rate:.0f} Hz on attempt {attempt}"
                    )
                    break
                self.get_logger().warning(
                    f"MAVLink message {message_id} at {message_rate:.0f} Hz "
                    f"attempt {attempt}/{self.max_attempts} failed"
                )
                if attempt < self.max_attempts:
                    time.sleep(1.0)
            if not configured:
                raise RuntimeError(
                    f"FCU rejected MAVLink message {message_id} at "
                    f"{message_rate:.0f} Hz after {self.max_attempts} attempts"
                )


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkStreamConfigurator()
    exit_code = 0
    try:
        node.configure()
    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
