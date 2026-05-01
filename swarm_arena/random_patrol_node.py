import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class RandomPatrol(Node):
    def __init__(self):
        super().__init__('random_patrol')

        # Parametri configurabili da launch
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('front_cone_deg', 30.0)

        self.v_lin = self.get_parameter('linear_speed').value
        self.v_ang = self.get_parameter('angular_speed').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.cone_deg = self.get_parameter('front_cone_deg').value

        # QoS BEST_EFFORT è obbligatorio per /scan di TurtleBot3 in Gazebo
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.on_scan, scan_qos
        )

        ## Stato corrente
        self.latest_scan = None
        self.turn_direction = 1
        self.turn_until = 0.0
        self.wander_until = 0.0
        self.stuck_since = None
        self.backing_until = 0.0

        # Loop di controllo a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Random patrol avviato: v={self.v_lin} m/s, '
            f'safe={self.safe_dist} m, cone=±{self.cone_deg}°'
        )

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def front_min_distance(self, scan: LaserScan):
        """Distanza minima nel cono frontale ±cone_deg gradi."""
        cone_rad = math.radians(self.cone_deg)
        n = len(scan.ranges)

        # Il LiDAR del TurtleBot3 ha angle_min=0, angle_max=2π (360°)
        # Il cono frontale corrisponde a [0, cone_rad] U [2π - cone_rad, 2π]
        idx_left = int(cone_rad / scan.angle_increment)
        idx_right = n - idx_left

        front = list(scan.ranges[:idx_left]) + list(scan.ranges[idx_right:])
        # Filtra letture invalide (inf, nan, 0)
        valid = [r for r in front
                 if scan.range_min < r < scan.range_max and not math.isnan(r)]

        return min(valid) if valid else float('inf')

    def best_turn_direction(self, scan: LaserScan):
        """Ritorna +1 se a sinistra c'è più spazio, -1 se a destra."""
        n = len(scan.ranges)
        # Sinistra = primo quarto (0° -> 90°), destra = ultimo quarto (270° -> 360°)
        left = [r for r in scan.ranges[: n // 4]
                if scan.range_min < r < scan.range_max]
        right = [r for r in scan.ranges[3 * n // 4:]
                 if scan.range_min < r < scan.range_max]

        avg_left = sum(left) / len(left) if left else 0.0
        avg_right = sum(right) / len(right) if right else 0.0
        return 1 if avg_left >= avg_right else -1

    def control_loop(self):
        if self.latest_scan is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()

        front_dist = self.front_min_distance(self.latest_scan)

        # Modalità "backup" attiva: continua a fare retromarcia
        if now < self.backing_until:
            cmd.linear.x = -0.08
            cmd.angular.z = self.v_ang * self.turn_direction * 0.5
            self.cmd_pub.publish(cmd)
            return

        if front_dist < self.safe_dist:
            # Se siamo bloccati a girare da più di 3 secondi, fai retromarcia
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > 3.0:
                self.get_logger().info('Stuck, backing up')
                self.turn_direction = random.choice([-1, 1])
                self.backing_until = now + 1.5
                self.stuck_since = None
                return

            if now > self.turn_until:
                self.turn_direction = self.best_turn_direction(self.latest_scan)
                self.turn_until = now + random.uniform(0.6, 1.4)

            cmd.linear.x = 0.0
            cmd.angular.z = self.v_ang * self.turn_direction
        else:
            # Strada libera: reset del timer "stuck"
            self.stuck_since = None
            cmd.linear.x = self.v_lin

            if now > self.wander_until:
                self.wander_bias = random.uniform(-0.3, 0.3)
                self.wander_until = now + random.uniform(4.0, 8.0)

            cmd.angular.z = getattr(self, 'wander_bias', 0.0)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RandomPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferma il robot prima di uscire
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()