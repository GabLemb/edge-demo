# /mnt/nvme/ros2_ws/src/swarm_arena/swarm_arena/auto_explorer_node.py
import math
import os
import random
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class AutoExplorer(Node):
    """
    Esploratore autonomo per il mapping.
    Strategia: muoviti sempre verso la direzione più libera (max range del LiDAR).
    Quando i muri sono vicini, ruota in posto verso il settore più aperto.
    Dopo `explore_duration` secondi, salva la mappa e termina.
    """

    def __init__(self):
        super().__init__('auto_explorer')

        # Parametri
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('explore_duration', 300.0)  # 3 minuti
        self.declare_parameter('map_save_path', '/mnt/nvme/ros2_ws/src/swarm_arena/maps/arena_map')

        self.v_lin = self.get_parameter('linear_speed').value
        self.v_ang = self.get_parameter('angular_speed').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.duration = self.get_parameter('explore_duration').value
        self.map_path = self.get_parameter('map_save_path').value

        # QoS BEST_EFFORT per i sensori Gazebo
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, sensor_qos)
        self.create_subscription(Odometry, 'odom', self.on_odom, sensor_qos)

        self.latest_scan = None
        self.start_time = time.time()
        self.last_log = 0.0
        self.saved = False

        # Tracking della copertura: griglia 0.5m sui punti odom visitati
        self.visited_cells = set()
        self.cell_size = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Auto-explorer avviato. Esplorerà per {self.duration}s, '
            f'poi salverà la mappa in {self.map_path}'
        )

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_x = x
        self.last_y = y

        # Estrai yaw dal quaternione
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Cella visitata
        cell = (round(x / self.cell_size), round(y / self.cell_size))
        self.visited_cells.add(cell)

    def best_direction(self, scan: LaserScan):
        """
        Sceglie la direzione bilanciando:
        - Spazio libero (range LiDAR)
        - Bias verso celle non visitate (frontier-like)
        - Random per evitare loop simmetrici al centro
        """
        n = len(scan.ranges)
        n_sectors = 12
        sector_size = n // n_sectors

        # Posizione corrente del robot (cella griglia)
        # Self.last_x, last_y aggiornati in on_odom (vedi sotto)
        rx = getattr(self, 'last_x', 0.0)
        ry = getattr(self, 'last_y', 0.0)
        ryaw = getattr(self, 'last_yaw', 0.0)

        sector_scores = []
        for i in range(n_sectors):
            start = i * sector_size
            end = start + sector_size
            ranges = [r for r in scan.ranges[start:end]
                      if scan.range_min < r < scan.range_max and not math.isnan(r)]
            if not ranges:
                sector_scores.append(-1.0)
                continue

            avg = sum(ranges) / len(ranges)
            min_r = min(ranges)

            # Penalizza se troppo vicino a un muro
            if min_r < self.safe_dist:
                sector_scores.append(-1.0)
                continue

            # Angolo del settore (frame robot)
            local_angle = scan.angle_min + (i + 0.5) * sector_size * scan.angle_increment
            while local_angle > math.pi:
                local_angle -= 2 * math.pi

            # Proietta una cella ipotetica 1.5m avanti in quella direzione (frame mondo)
            world_angle = ryaw + local_angle
            target_x = rx + 1.5 * math.cos(world_angle)
            target_y = ry + 1.5 * math.sin(world_angle)
            target_cell = (round(target_x / self.cell_size),
                           round(target_y / self.cell_size))

            # Score base = spazio libero
            score = avg

            # Bonus se la cella target NON è stata visitata
            if target_cell not in self.visited_cells:
                score *= 3.0

            # Penalità decrescente per celle nel raggio già visitate
            visited_nearby = sum(
                1 for c in self.visited_cells
                if abs(c[0] - target_cell[0]) <= 1 and abs(c[1] - target_cell[1]) <= 1
            )
            score /= (1.0 + 0.3 * visited_nearby)

            # Rumore random per rompere la simmetria
            score *= random.uniform(0.85, 1.15)

            sector_scores.append(score)

        best_sector = max(range(n_sectors), key=lambda i: sector_scores[i])

        if sector_scores[best_sector] < 0:
            # Tutti i settori sono bloccati, scegli random
            best_sector = random.randint(0, n_sectors - 1)

        angle = scan.angle_min + (best_sector + 0.5) * sector_size * scan.angle_increment
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle, sector_scores[best_sector]

    def front_min(self, scan: LaserScan, cone_deg=20):
        """Distanza minima nel cono frontale ±cone_deg."""
        cone_rad = math.radians(cone_deg)
        n = len(scan.ranges)
        idx_left = int(cone_rad / scan.angle_increment)
        idx_right = n - idx_left
        front = list(scan.ranges[:idx_left]) + list(scan.ranges[idx_right:])
        valid = [r for r in front
                 if scan.range_min < r < scan.range_max and not math.isnan(r)]
        return min(valid) if valid else float('inf')

    def control_loop(self):
        elapsed = time.time() - self.start_time

        # Log di progresso ogni 15s
        if elapsed - self.last_log >= 15.0:
            self.last_log = elapsed
            self.get_logger().info(
                f'Esplorazione: {elapsed:.0f}/{self.duration:.0f}s | '
                f'celle visitate: {len(self.visited_cells)}'
            )

        # Tempo scaduto: ferma e salva
        if elapsed >= self.duration:
            if not self.saved:
                self.stop_robot()
                self.save_map()
                self.saved = True
                self.get_logger().info('Esplorazione completata. Termino.')
                # Termina il nodo
                rclpy.shutdown()
            return

        if self.latest_scan is None:
            return

        cmd = Twist()
        target_angle, target_score = self.best_direction(self.latest_scan)
        front = self.front_min(self.latest_scan, cone_deg=20)

        # Se il davanti è bloccato, ruota in posto verso la direzione migliore
        if front < self.safe_dist:
            # Ruota nella direzione del settore migliore
            cmd.linear.x = 0.0
            cmd.angular.z = self.v_ang * (1.0 if target_angle >= 0 else -1.0)
        else:
            # Avanti, sterzando verso la direzione migliore
            cmd.linear.x = self.v_lin
            # Sterzata proporzionale all'errore angolare, saturata
            cmd.angular.z = max(-self.v_ang, min(self.v_ang, 1.5 * target_angle))

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def save_map(self):
        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        self.get_logger().info(f'Salvo la mappa in {self.map_path}...')
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self.map_path],
                capture_output=True, text=True, timeout=30
            )
            if result.returncode == 0:
                self.get_logger().info(f'Mappa salvata: {self.map_path}.pgm + .yaml')
            else:
                self.get_logger().error(
                    f'map_saver fallito (code {result.returncode}): {result.stderr}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver timeout dopo 30s')
        except Exception as e:
            self.get_logger().error(f'Errore salvataggio mappa: {e}')


def main():
    rclpy.init()
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.save_map()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()