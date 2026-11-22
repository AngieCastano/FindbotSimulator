"""
Nodo comm_relay: el enlace de datos entre el robot y la estación remota.

Cada mensaje que cruza el enlace (sensores del robot → estación; mandos de la estación → robot)
pasa por este nodo, que lo entrega con el retraso, o lo pierde, que corresponde a la señal WiFi
medida (/comm/link, usar_msgs/LinkQuality) según usar_station.link_model. Sin /comm/link
(comm:=false) el enlace es ideal (solo la latencia base).

Los tópicos que cruzan están en config/link.yaml (con su tasa máxima en el origen y su
prioridad: el control no espera detrás del video). En la estación todo vive bajo /station/*.
Opcionalmente registra cada mensaje en un CSV (log_csv) para medir latencias y pérdidas.

No lee el ground truth: decide solo con el RSSI, como haría la radio real.
"""

import csv
import heapq
import itertools

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from usar_msgs.msg import LinkQuality
from usar_station.link_model import DEFAULTS, LinkChannel
import yaml

QOS = {
    'reliable': QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    'best_effort': QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
    'latched': QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL),
}


class CommRelay(Node):
    """Reenvía los mensajes de config/link.yaml a través del canal simulado."""

    def __init__(self):
        super().__init__('comm_relay')
        self.declare_parameter('links_file', '')
        self.declare_parameter('link_topic', '/comm/link')
        self.declare_parameter('link_timeout', 1.0)
        self.declare_parameter('log_csv', '')
        self.declare_parameter('seed', 0)
        self.declare_parameter('scheduler_period', 0.002)
        with open(self.get_parameter('links_file').value, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        params = {**DEFAULTS, **config.get('channel', {})}
        params['rates'] = [tuple(r) for r in params['rates']]
        self.channel = LinkChannel(params, np.random.default_rng(
            int(self.get_parameter('seed').value)))
        self.rssi = None
        self.rssi_time = None
        self.queue = []
        self.counter = itertools.count()
        self.stats = {'sent': 0, 'delivered': 0, 'lost': 0}
        self.subs = []
        self.last_sent = {}
        for link in config['links']:
            msg_type = get_message(link['type'])
            qos = QOS[link.get('qos', 'reliable')]
            pub = self.create_publisher(msg_type, link['to'], qos)
            self.subs.append(self.create_subscription(
                msg_type, link['from'],
                lambda msg, link=link, pub=pub: self._on_message(msg, link, pub), qos))
        self.create_subscription(LinkQuality, self.get_parameter('link_topic').value,
                                 self._on_link, 10)
        self.create_timer(float(self.get_parameter('scheduler_period').value), self._deliver)
        self.create_timer(10.0, self._report)
        self.log = None
        path = self.get_parameter('log_csv').value
        if path:
            self.log_file = open(path, 'w', newline='', encoding='utf-8')
            self.log = csv.writer(self.log_file)
            self.log.writerow(['sent', 'arrival', 'topic', 'bytes', 'rssi_dbm'])
        self.get_logger().info(f'Enlace robot ↔ estación: {len(config["links"])} tópicos')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_link(self, msg):
        self.rssi, self.rssi_time = float(msg.rssi_dbm), self._now()

    def _current_rssi(self, now):
        if self.rssi_time is None:
            return None                       # sin modelo de señal: enlace ideal
        if now - self.rssi_time > float(self.get_parameter('link_timeout').value):
            return -200.0                     # la radio dejó de reportar: sin enlace
        return self.rssi

    def _on_message(self, msg, link, pub):
        now = self._now()
        # Tasa máxima en el origen (el robot no intenta enviar más de lo que el enlace aguanta)
        max_rate = float(link.get('max_rate', 0.0))
        if max_rate > 0.0:
            last = self.last_sent.get(link['from'])
            if last is not None and now - last < 1.0 / max_rate:
                return
            self.last_sent[link['from']] = now
        size = len(serialize_message(msg))
        rssi = self._current_rssi(now)
        arrival = self.channel.send(now, size, rssi, bool(link.get('compress', False)),
                                    link.get('priority') == 'high')
        self.stats['sent'] += 1
        if self.log is not None:
            self.log.writerow([f'{now:.4f}', '' if arrival is None else f'{arrival:.4f}',
                               link['from'], size, '' if rssi is None else f'{rssi:.1f}'])
        if arrival is None:
            self.stats['lost'] += 1
            return
        heapq.heappush(self.queue, (arrival, next(self.counter), pub, msg))

    def _deliver(self):
        now = self._now()
        while self.queue and self.queue[0][0] <= now:
            _, _, pub, msg = heapq.heappop(self.queue)
            pub.publish(msg)
            self.stats['delivered'] += 1

    def _report(self):
        s = self.stats
        rssi = self._current_rssi(self._now())
        self.get_logger().info(
            f'Enlace: {s["sent"]} enviados, {s["delivered"]} entregados, {s["lost"]} perdidos'
            + ('' if rssi is None else f'; RSSI {rssi:.0f} dBm'))
        if self.log is not None:
            self.log_file.flush()

    def destroy_node(self):
        """Cierra el registro antes de terminar."""
        if self.log is not None:
            self.log_file.close()
            self.log = None
        super().destroy_node()


def main():
    rclpy.init()
    node = CommRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
