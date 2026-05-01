# /mnt/nvme/ros2_ws/src/swarm_arena/swarm_arena/target_tracker_node.py
import json
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class TargetTracker(Node):
    """
    Simula il campo visivo di un Observer.
    Pubblica osservazioni dei target visibili come JSON su 'target_observations'.
    """

    def __init__(self):
        super().__init__('target_tracker')

        self.declare_parameter('observer_name', 'observer_1')
        self.declare_parameter('detection_radius', 2.5)  # metri
        self.declare_parameter('publish_rate', 5.0)      # Hz

        self.observer_name = self.get_parameter('observer_name').value
        self.detection_radius = self.get_parameter('detection_radius').value
        publish_rate = self.get_parameter('publish_rate').value

        self.observer_pose = None  # (x, y) nel frame globale
        self.target_poses = {}     # name -> (x, y)

        # Sottoscrizione a /gazebo/model_states (frame globale, ground truth)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_model_states, 10
        )

        # Output: topic relativo, sotto namespace /observer_N/
        self.obs_pub = self.create_publisher(String, 'target_observations', 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_observations)
        self.get_logger().info(
            f'Tracker per {self.observer_name}, '
            f'raggio rilevamento {self.detection_radius} m'
        )

    def on_model_states(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name == self.observer_name:
                self.observer_pose = (pose.position.x, pose.position.y)
            elif name.startswith('target_'):
                self.target_poses[name] = (pose.position.x, pose.position.y)

    def publish_observations(self):
        if self.observer_pose is None or not self.target_poses:
            return

        ox, oy = self.observer_pose
        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        for target_name, (tx, ty) in self.target_poses.items():
            dist = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
            if dist > self.detection_radius:
                continue

            # Formato esatto richiesto dal SoW
            payload = {
                'target_id': target_name,
                'abs_x': round(tx, 3),
                'abs_y': round(ty, 3),
                'timestamp': round(timestamp, 3),
                'observed_by': self.observer_name,
                'distance': round(dist, 3),
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.obs_pub.publish(msg)


def main():
    rclpy.init()
    node = TargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()