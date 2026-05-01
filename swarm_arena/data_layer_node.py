# /mnt/nvme/ros2_ws/src/swarm_arena/swarm_arena/data_layer_node.py
import json
import os
import sqlite3
import threading
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, jsonify


class DataLayer(Node):
    """
    Mock del Myrmic DataLayer.
    - Legge le osservazioni locali (/observer_N/target_observations)
    - Le replica su un topic globale (/swarm/replicate)
    - Mantiene uno storico distribuito identico su tutti i nodi
    - Persiste su SQLite locale (/mnt/nvme/myrmic_mock/observer_N.db)
    - Espone HTTP /history e /status per la dashboard dello Step 6
    """

    def __init__(self):
        super().__init__('data_layer')

        self.declare_parameter('observer_name', 'observer_1')
        self.declare_parameter('http_port', 5001)
        self.declare_parameter('storage_dir', '/mnt/nvme/myrmic_mock')

        self.observer_name = self.get_parameter('observer_name').value
        self.http_port = self.get_parameter('http_port').value
        storage_dir = self.get_parameter('storage_dir').value

        os.makedirs(storage_dir, exist_ok=True)
        self.db_path = os.path.join(storage_dir, f'{self.observer_name}.db')

        # Storico in memoria. Chiave deduplica = (observed_by, timestamp, target_id)
        # Il valore è il dict completo dell'osservazione.
        self._lock = threading.Lock()
        self._observations = OrderedDict()

        self._init_db()
        self._load_from_db()

        # Topic locale (la propria osservazione) — sotto namespace /observer_N/
        self.create_subscription(
            String, 'target_observations', self.on_local_observation, 50
        )

        # Topic globale di replica — fuori namespace, condiviso da tutti
        self.replicate_pub = self.create_publisher(
            String, '/swarm/replicate', 50
        )
        self.create_subscription(
            String, '/swarm/replicate', self.on_replicated_observation, 50
        )

        # Avvia il server HTTP in un thread separato
        self._start_http_server()

        # Status periodico
        self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f'DataLayer attivo per {self.observer_name} | '
            f'DB: {self.db_path} | HTTP: 0.0.0.0:{self.http_port}'
        )

    # ------------------------------------------------------------------
    # Persistenza SQLite
    # ------------------------------------------------------------------

    def _init_db(self):
        conn = sqlite3.connect(self.db_path)
        conn.execute('''
            CREATE TABLE IF NOT EXISTS observations (
                observed_by TEXT NOT NULL,
                timestamp   REAL NOT NULL,
                target_id   TEXT NOT NULL,
                abs_x       REAL NOT NULL,
                abs_y       REAL NOT NULL,
                distance    REAL,
                PRIMARY KEY (observed_by, timestamp, target_id)
            )
        ''')
        conn.execute(
            'CREATE INDEX IF NOT EXISTS idx_target_time '
            'ON observations(target_id, timestamp)'
        )
        conn.commit()
        conn.close()

    def _load_from_db(self):
        conn = sqlite3.connect(self.db_path)
        rows = conn.execute(
            'SELECT observed_by, timestamp, target_id, abs_x, abs_y, distance '
            'FROM observations ORDER BY timestamp'
        ).fetchall()
        conn.close()
        for r in rows:
            key = (r[0], r[1], r[2])
            self._observations[key] = {
                'observed_by': r[0], 'timestamp': r[1],
                'target_id': r[2], 'abs_x': r[3], 'abs_y': r[4],
                'distance': r[5],
            }
        if rows:
            self.get_logger().info(
                f'Recuperate {len(rows)} osservazioni dal DB locale'
            )

    def _persist(self, obs):
        # SQLite non è thread-safe sullo stesso connection -> apri/chiudi al volo
        try:
            conn = sqlite3.connect(self.db_path, timeout=2.0)
            conn.execute(
                'INSERT OR IGNORE INTO observations '
                '(observed_by, timestamp, target_id, abs_x, abs_y, distance) '
                'VALUES (?, ?, ?, ?, ?, ?)',
                (obs['observed_by'], obs['timestamp'], obs['target_id'],
                 obs['abs_x'], obs['abs_y'], obs.get('distance'))
            )
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().warn(f'SQLite error: {e}')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def on_local_observation(self, msg: String):
        """Osservazione propria → ingest locale + broadcast agli altri nodi."""
        try:
            obs = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._ingest(obs)
        # Replica agli altri (ricevono anche se stessi, ma deduplicano)
        out = String()
        out.data = msg.data
        self.replicate_pub.publish(out)

    def on_replicated_observation(self, msg: String):
        """Osservazione ricevuta dal topic globale (anche dai propri publish)."""
        try:
            obs = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._ingest(obs)

    def _ingest(self, obs):
        """Inserisce l'osservazione se non già presente (dedup per chiave)."""
        key = (obs.get('observed_by'), obs.get('timestamp'), obs.get('target_id'))
        if None in key:
            return
        with self._lock:
            if key in self._observations:
                return
            self._observations[key] = obs
        self._persist(obs)

    def _log_status(self):
        with self._lock:
            n_total = len(self._observations)
            by_target = {}
            for obs in self._observations.values():
                t = obs['target_id']
                by_target[t] = by_target.get(t, 0) + 1
        summary = ', '.join(f'{k}:{v}' for k, v in sorted(by_target.items()))
        self.get_logger().info(f'[{self.observer_name}] tot={n_total} | {summary}')

    # ------------------------------------------------------------------
    # HTTP server (Flask in thread separato)
    # ------------------------------------------------------------------

    def _start_http_server(self):
        app = Flask(__name__)

        @app.route('/status')
        def status():
            with self._lock:
                return jsonify({
                    'observer': self.observer_name,
                    'total_observations': len(self._observations),
                    'db_path': self.db_path,
                })

        @app.route('/history')
        def history():
            with self._lock:
                by_target = {}
                for obs in self._observations.values():
                    by_target.setdefault(obs['target_id'], []).append(obs)
                for t in by_target:
                    by_target[t].sort(key=lambda o: o['timestamp'])
                return jsonify({
                    'observer': self.observer_name,
                    'targets': by_target,
                })

        @app.route('/health')
        def health():
            return jsonify({'observer': self.observer_name, 'status': 'ok'})

        import logging
        import socket
        logging.getLogger('werkzeug').setLevel(logging.ERROR)

        # Test esplicito del bind PRIMA di lanciare Flask, così se la porta
        # è occupata vediamo subito l'errore nel log del nodo invece che nel thread.
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            test_sock.bind(('0.0.0.0', self.http_port))
            test_sock.close()
        except OSError as e:
            self.get_logger().error(
                f'IMPOSSIBILE bindare porta {self.http_port}: {e}. '
                f'Probabile zombie da launch precedente. '
                f'Esegui: pkill -9 -f data_layer'
            )
            return

        def _run():
            try:
                self.get_logger().info(
                    f'HTTP starting on 0.0.0.0:{self.http_port}'
                )
                app.run(
                    host='0.0.0.0', port=self.http_port,
                    debug=False, use_reloader=False, threaded=True,
                )
            except Exception as e:
                self.get_logger().error(f'HTTP server crashed: {e!r}')

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()


def main():
    rclpy.init()
    node = DataLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()