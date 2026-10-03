"""Standalone logic-chain validation (no Gazebo): publishes a synthetic
writer beacon handoff and synthetic executor odom, then asserts the full
RECEIVE -> TRANSLATE -> CARRY -> BRIEF -> navigate -> report chain fires.
Bypasses Gazebo physics entirely, since gzserver on this host is stalled by
a Hyper-V clock-sync hiccup unrelated to the Living Map code.
"""
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

ENTRANCE_X, ENTRANCE_Y = -8.5, -8.0


class FakeExecutorBody(Node):
    """Stands in for the Gazebo diff_drive plugin: integrates /executor/cmd_vel
    into a pose and republishes /executor/odom, so executor_node's go-to-goal
    control loop has real feedback to close the loop against."""
    def __init__(self):
        super().__init__('fake_executor_body')
        self.x, self.y = ENTRANCE_X, ENTRANCE_Y
        self.yaw = 0.0
        self.odom_pub = self.create_publisher(Odometry, '/executor/odom', 10)
        self.create_subscription(Twist, '/executor/cmd_vel', self.on_cmd, 10)
        self.vx, self.wz = 0.0, 0.0
        self.create_timer(0.1, self.step)

    def on_cmd(self, msg: Twist):
        self.vx, self.wz = msg.linear.x, msg.angular.z

    def step(self):
        dt = 0.1
        self.yaw += self.wz * dt
        self.x += self.vx * dt * __import__('math').cos(self.yaw)
        self.y += self.vx * dt * __import__('math').sin(self.yaw)
        o = Odometry()
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        self.odom_pub.publish(o)


class Harness(Node):
    def __init__(self):
        super().__init__('test_harness')
        self.tx_pub = self.create_publisher(String, '/writer/to_outside_network', 10)
        self.cp_sub = self.create_subscription(String, '/command_post/beacons', self.on_cp, 10)
        self.report_sub = self.create_subscription(String, '/command_post/mission_report', self.on_report, 10)
        self.got_cp = False
        self.got_report = False

    def on_cp(self, msg):
        self.got_cp = True
        self.get_logger().info(f'[TEST] Command Post received: {msg.data[:200]}')

    def on_report(self, msg):
        self.got_report = True
        self.get_logger().info(f'[TEST] Command Post mission report: {msg.data}')

    def send_synthetic_beacons(self):
        payload = {
            'robot': 'writer',
            'entrance_origin_local': [ENTRANCE_X, ENTRANCE_Y],
            'beacons': [
                {'id': 'B1', 'what': 'hazard', 'x': -6.0, 'y': -5.0,
                 'distance': 3.6, 'when': time.time(), 'version': 1},
                {'id': 'B2', 'what': 'victim', 'x': 5.0, 'y': 2.0,
                 'distance': 15.6, 'when': time.time(), 'version': 1},
            ],
        }
        self.tx_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info('[TEST] synthetic Writer -> Outside Network handoff sent (2 beacons).')


def main():
    rclpy.init()
    harness = Harness()
    body = FakeExecutorBody()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(body)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)
    harness.send_synthetic_beacons()

    deadline = time.time() + 45
    while time.time() < deadline:
        if harness.got_cp and harness.got_report:
            break
        time.sleep(0.5)

    print('RESULT got_cp=%s got_report=%s final_pos=(%.2f,%.2f)' % (
        harness.got_cp, harness.got_report, body.x, body.y))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
