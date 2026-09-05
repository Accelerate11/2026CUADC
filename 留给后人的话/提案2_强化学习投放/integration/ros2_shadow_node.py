"""Optional ROS 2 wrapper for an explicitly supplied synchronized snapshot topic.

The existing state machine must publish the contract in integration/README.md.
Outputs text decisions only; never publishes to MAVROS or calls a release service.
"""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from drop_rl.config import load_config
    from drop_rl.integration import ShadowSupervisor
    from drop_rl.policies import QPolicy, Baseline

    class ShadowNode(Node):
        def __init__(self):
            super().__init__('cuadc_rl_drop_shadow')
            self.declare_parameter('config_path', '')
            self.declare_parameter('model_path', '')
            cfg = load_config(self.get_parameter('config_path').value or None)
            model = self.get_parameter('model_path').value
            policy = QPolicy.load(model, cfg) if model else Baseline('ballistic', cfg)
            self.supervisor = ShadowSupervisor(policy, cfg)
            self.pub = self.create_publisher(String, '/rl_drop/shadow_decision', 10)
            self.sub = self.create_subscription(String, '/rl_drop/snapshot', self.callback, 10)

        def callback(self, message):
            try:
                snapshot = json.loads(message.data)
                # Replace producer time with this node's ROS clock to catch queue delay.
                snapshot['now_s'] = self.get_clock().now().nanoseconds * 1e-9
                decision = self.supervisor.decide(snapshot)
            except (ValueError, TypeError) as error:
                decision = {'mode': 'shadow_only', 'candidate_release': False, 'reason': str(error)}
            result = String()
            result.data = json.dumps(decision, ensure_ascii=False)
            self.pub.publish(result)

    rclpy.init()
    node = ShadowNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
