import math
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist


class TargetMover(Node):
    """Muove i target lungo traiettorie casuali rimbalzando sui muri dell'arena."""

    def __init__(self):
        super().__init__('target_mover')

        self.targets = {
            'target_elephant': {'x': 1.5, 'y': -0.5, 'vx': 0.0, 'vy': 0.0},
            'target_zebra':    {'x': -1.0, 'y': 1.5, 'vx': 0.0, 'vy': 0.0},
            'target_giraffe':  {'x': 2.0, 'y': 1.0, 'vx': 0.0, 'vy': 0.0},
            'target_lion':     {'x': -2.0, 'y': -1.0, 'vx': 0.0, 'vy': 0.0},
            'target_antelope': {'x': 0.5, 'y': 2.0, 'vx': 0.0, 'vy': 0.0},
        }

        # Velocità iniziali random
        for t in self.targets.values():
            angle = random.uniform(0, 2 * math.pi)
            speed = random.uniform(0.05, 0.15)
            t['vx'] = speed * math.cos(angle)
            t['vy'] = speed * math.sin(angle)

        # Limiti arena (lascia 0.3m di margine dai muri a ±3)
        self.arena_min = -2.7
        self.arena_max = 2.7

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aspetto /gazebo/set_entity_state...')

        # Update a 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_targets)
        self.get_logger().info(f'Muovo {len(self.targets)} target')

    def update_targets(self):
        for name, t in self.targets.items():
            # Aggiorna posizione
            t['x'] += t['vx'] * self.dt
            t['y'] += t['vy'] * self.dt

            # Rimbalzo sui muri
            if t['x'] < self.arena_min or t['x'] > self.arena_max:
                t['vx'] *= -1
                t['x'] = max(self.arena_min, min(self.arena_max, t['x']))
            if t['y'] < self.arena_min or t['y'] > self.arena_max:
                t['vy'] *= -1
                t['y'] = max(self.arena_min, min(self.arena_max, t['y']))

            # Piccola perturbazione random per evitare moti troppo rettilinei
            if random.random() < 0.02:
                angle_change = random.uniform(-0.5, 0.5)
                cos_a, sin_a = math.cos(angle_change), math.sin(angle_change)
                vx, vy = t['vx'], t['vy']
                t['vx'] = cos_a * vx - sin_a * vy
                t['vy'] = sin_a * vx + cos_a * vy

            # Manda il comando a Gazebo
            req = SetEntityState.Request()
            state = EntityState()
            state.name = name
            state.pose.position.x = t['x']
            state.pose.position.y = t['y']
            state.pose.position.z = 0.15
            state.pose.orientation.w = 1.0
            req.state = state
            self.cli.call_async(req)


def main():
    rclpy.init()
    node = TargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()