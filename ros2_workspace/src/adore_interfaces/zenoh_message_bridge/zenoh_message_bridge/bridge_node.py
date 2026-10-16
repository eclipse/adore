import os
import queue
import struct
import threading
import time
import traceback
import uuid
import yaml
import zenoh
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from .utils import load_msg_type, msg_to_bytes, bytes_to_msg, msg_to_json, json_to_msg, msg_to_cdr_json, cdr_json_to_msg

_STR_TYPE = 'std_msgs/msg/String'

def _wire_type(ros_type: str, fmt: str) -> str:
    """Transport type: json uses raw bytes (no fixed type), cdr_json wraps in std_msgs/msg/String."""
    return _STR_TYPE if fmt in ('json', 'cdr_json') else ros_type

def _serializer(ros_type: str, fmt: str):
    if fmt == 'json':
        return lambda msg, rt=ros_type: msg_to_json(msg, rt)
    if fmt == 'cdr_json':
        return lambda msg, rt=ros_type: msg_to_cdr_json(msg, rt)
    return msg_to_bytes

def _deserializer(msg_type, fmt: str):
    if fmt == 'json':
        return lambda data, mt=msg_type: json_to_msg(data, mt)
    if fmt == 'cdr_json':
        return lambda data, mt=msg_type: cdr_json_to_msg(data, mt)
    return lambda data, mt=msg_type: bytes_to_msg(data, mt)

def _make_gid() -> bytes:
    """Generate a 16-byte publisher GID from a random UUID."""
    return uuid.uuid4().bytes

def _make_attachment(seq: int, gid: bytes) -> bytes:
    """
    rmw_zenoh_cpp attachment format:
      8 bytes - sequence number (int64 LE)
      8 bytes - source timestamp (ns since epoch, int64 LE)
      1 byte  - GID length (always 16)
     16 bytes - publisher GID
    """
    ts = time.time_ns()
    return struct.pack('<qq', seq, ts) + bytes([len(gid)]) + gid

def _type_hash(ros_type: str) -> str:
    import json
    pkg, interface, name = ros_type.split('/')
    ros_distro = os.environ.get('ROS_DISTRO', '')
    json_path = f'/opt/ros/{ros_distro}/share/{pkg}/{interface}/{name}.json'
    try:
        with open(json_path) as f:
            for entry in json.load(f).get('type_hashes', []):
                if entry.get('type_name') == ros_type:
                    return entry['hash_string']
    except (FileNotFoundError, KeyError, ValueError):
        pass
    return 'TypeHashNotSupported'

_RMW_ZENOH_LV_PREFIX = '@ros2_lv'

def _mangle(name: str) -> str:
    return name.replace('/', '%', 1) if name.startswith('/') else name

def _strip_slash(name: str) -> str:
    return name.lstrip('/')

def _dds_type(ros_type: str) -> str:
    pkg, interface, name = ros_type.split('/')
    return f'{pkg}::{interface}::dds_::{name}_'

def _qos_to_keyexpr(qos: QoSProfile) -> str:
    rel = '1' if qos.reliability == ReliabilityPolicy.RELIABLE else '2'
    depth = str(qos.depth)
    dur = '1' if qos.durability == DurabilityPolicy.TRANSIENT_LOCAL else ''
    return f':{rel}:,{depth}:,:{dur},:,,'

def _topic_keyexpr(domain_id: int, topic: str, ros_type: str, rmw_target: str) -> str:
    hash_str = _type_hash(ros_type) if rmw_target == 'jazzy' else 'TypeHashNotSupported'
    return f'{domain_id}/{_strip_slash(topic)}/{_dds_type(ros_type)}/{hash_str}'

def _topic_sub_keyexpr(domain_id: int, topic: str, ros_type: str) -> str:
    """Wildcard hash segment to match publishers from any ROS2 distro."""
    return f'{domain_id}/{_strip_slash(topic)}/{_dds_type(ros_type)}/*'

def _liveliness_keyexpr(
    domain_id: int,
    session_id: str,
    pub_id: int,
    node_name: str,
    topic: str,
    ros_type: str,
    qos: QoSProfile,
    rmw_target: str,
) -> str:
    hash_str = _type_hash(ros_type) if rmw_target == 'jazzy' else 'TypeHashNotSupported'
    return (
        f'{_RMW_ZENOH_LV_PREFIX}/{domain_id}/{session_id}'
        f'/0/{pub_id}/MP/%/%/{_mangle(node_name)}'
        f'/{_mangle(topic)}/{_dds_type(ros_type)}/{hash_str}/{_qos_to_keyexpr(qos)}'
    )

_DURABILITY = {
    'volatile':        DurabilityPolicy.VOLATILE,
    'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
}
_RELIABILITY = {
    'best_effort': ReliabilityPolicy.BEST_EFFORT,
    'reliable':    ReliabilityPolicy.RELIABLE,
}

def _qos_from_mapping(mapping: dict, default_reliability: str = 'reliable') -> QoSProfile:
    return QoSProfile(
        depth=mapping.get('qos_depth', 1),
        durability=_DURABILITY.get(mapping.get('qos_durability', 'volatile'), DurabilityPolicy.VOLATILE),
        reliability=_RELIABILITY.get(mapping.get('qos_reliability', default_reliability),
                                     _RELIABILITY[default_reliability]),
        history=HistoryPolicy.KEEP_LAST,
    )

_MAPPING_KEYS = ('ros2_to_zenoh', 'zenoh_to_ros2')

def _merge_configs(configs: list) -> dict:
    """Merge in file order: mappings are concatenated, other keys taken from the first file setting them."""
    merged = {}
    for config in configs:
        for key, value in config.items():
            if key in _MAPPING_KEYS:
                merged.setdefault(key, []).extend(value or [])
            elif key not in merged:
                merged[key] = value
    return merged

def _duplicate_topics(config: dict) -> list:
    """Topics mapped more than once in the same direction."""
    duplicates = []
    for key in _MAPPING_KEYS:
        seen = set()
        for mapping in config.get(key, []):
            topic = mapping.get('ros_topic')
            if topic in seen:
                duplicates.append(topic)
            seen.add(topic)
    return duplicates

# Bounded, so a burst on the zenoh side cannot starve the ros2_to_zenoh subscriptions
_MAX_PUBLISHES_PER_TICK = 100


class ROS2ZenohBridge(Node):
    def __init__(self):
        super().__init__('zenoh_bridge_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('config_paths', [''])
        self.declare_parameter('zenoh_config_path', '')
        self.declare_parameter('zenoh_router', 'tcp/localhost:7447')

        self.config = self._load_config()
        if self.config is None:
            raise RuntimeError('no usable bridge configuration')

        self._ros_domain_id   = int(self.config.get('ros_domain_id', os.environ.get('ROS_DOMAIN', '0')))
        self._zenoh_bridge_id = int(self.config.get('zenoh_bridge_id', 0))
        self._rmw_target      = self.config.get('rmw_target', 'humble')

        self.zenoh_session = None
        self.zenoh_subs = []
        self.zenoh_pubs = {}
        self.zenoh_lv_tokens = []
        self.ros_subs = []
        self.ros_pubs = {}
        self._z2r_queue = queue.Queue()
        self._shutdown_event = threading.Event()

        self._setup_zenoh()
        self._setup_ros2_to_zenoh()
        self._setup_zenoh_to_ros2()
        self.create_timer(0.01, self._drain_z2r_queue)

    def _config_paths(self) -> list:
        """Paths from 'config_paths' when set, otherwise the single 'config_path'."""
        paths = [p for p in self.get_parameter('config_paths').get_parameter_value().string_array_value if p]
        if paths:
            return paths
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        return [config_path] if config_path else []

    def _load_config(self):
        """Load every configured file and merge them. Returns None if any of them is unusable."""
        paths = self._config_paths()
        if not paths:
            self.get_logger().error("No config file given, set config_path or config_paths")
            return None

        configs = []
        for path in paths:
            if not os.path.exists(path):
                self.get_logger().error(f"Config file not found: {path}")
                return None
            try:
                with open(path, 'r') as f:
                    configs.append(yaml.safe_load(f) or {})
            except Exception as e:
                self.get_logger().error(f"Failed to load config {path}: {e}")
                return None
            self.get_logger().info(f'Config loaded: {path}')

        config = _merge_configs(configs)
        for topic in _duplicate_topics(config):
            self.get_logger().warn(f'{topic} is mapped more than once and will be bridged repeatedly')
        return config

    def _setup_zenoh(self):
        zenoh_config_path = self.get_parameter('zenoh_config_path').get_parameter_value().string_value

        if zenoh_config_path and os.path.exists(zenoh_config_path):
            self.get_logger().info(f'Loading Zenoh config: {zenoh_config_path}')
            z_config = zenoh.Config.from_file(zenoh_config_path)
        else:
            if zenoh_config_path:
                self.get_logger().warn(f'Zenoh config not found: {zenoh_config_path}, falling back to endpoint param')
            endpoint = self.get_parameter('zenoh_router').get_parameter_value().string_value
            z_config = zenoh.Config()
            z_config.insert_json5('connect/endpoints', f'["{endpoint}"]')

        self.zenoh_session = zenoh.open(z_config)
        self._session_id = str(self.zenoh_session.info.zid()).replace('-', '')
        self.get_logger().info(f'Zenoh session opened: {self._session_id}')

    def _setup_ros2_to_zenoh(self):
        node_name = self.get_name()
        for pub_id, mapping in enumerate(self.config.get('ros2_to_zenoh', [])):
            ros_topic = mapping['ros_topic']
            ros_type  = mapping.get('msg_type', _STR_TYPE)
            msg_type  = load_msg_type(ros_type)
            domain_id = int(mapping.get('domain_id', self._zenoh_bridge_id))
            fmt       = mapping.get('format', 'cdr')
            qos       = _qos_from_mapping(mapping, default_reliability='reliable')
            wtype     = _wire_type(ros_type, fmt)
            serialize = _serializer(ros_type, fmt)

            custom_key = mapping.get('zenoh_key')
            zenoh_key = custom_key or _topic_keyexpr(domain_id, ros_topic, wtype, self._rmw_target)
            pub = self.zenoh_session.declare_publisher(zenoh_key)
            self.zenoh_pubs[ros_topic] = pub

            # A custom key is not an rmw_zenoh_cpp topic, so do not advertise one.
            if not custom_key:
                lv_key = _liveliness_keyexpr(domain_id, self._session_id, pub_id, node_name, ros_topic, wtype, qos, self._rmw_target)
                token = self.zenoh_session.liveliness().declare_token(lv_key)
                self.zenoh_lv_tokens.append(token)

            gid = _make_gid()
            seq = [0]
            self.get_logger().info(f'R2Z: {ros_topic} -> {zenoh_key} [{fmt}]')
            def cb(msg, p=pub, k=zenoh_key, g=gid, s=seq, ser=serialize):
                attachment = _make_attachment(s[0], g)
                s[0] += 1
                p.put(ser(msg), attachment=attachment)
                self.get_logger().debug(f'R2Z sent: {k}')
            self.ros_subs.append(self.create_subscription(msg_type, ros_topic, cb, qos))

    def _setup_zenoh_to_ros2(self):
        for mapping in self.config.get('zenoh_to_ros2', []):
            ros_topic = mapping['ros_topic']
            ros_type  = mapping.get('msg_type', _STR_TYPE)
            domain_id = int(mapping.get('domain_id', self._zenoh_bridge_id))
            fmt       = mapping.get('format', 'cdr')
            wtype     = _wire_type(ros_type, fmt)
            m_type    = load_msg_type(ros_type)
            qos       = _qos_from_mapping(mapping, default_reliability='best_effort')
            z_key     = mapping.get('zenoh_key') or _topic_sub_keyexpr(domain_id, ros_topic, wtype)
            deserialize = _deserializer(m_type, fmt)
            pub = self.create_publisher(m_type, ros_topic, qos)
            self.ros_pubs[ros_topic] = pub
            self.get_logger().info(f'Z2R: {z_key} -> {ros_topic} [{fmt}]')
            def z_cb(sample, p=pub, t=ros_topic, deser=deserialize):
                if self._shutdown_event.is_set():
                    return
                try:
                    self._z2r_queue.put((p, deser(sample.payload.to_bytes())))
                    self.get_logger().debug(f'Z2R received: {t}')
                except Exception as e:
                    self.get_logger().error(f'Deser failed on {t}: {e}')
            self.zenoh_subs.append(self.zenoh_session.declare_subscriber(z_key, z_cb))

    def _drain_z2r_queue(self):
        for _ in range(_MAX_PUBLISHES_PER_TICK):
            try:
                pub, msg = self._z2r_queue.get_nowait()
            except queue.Empty:
                return
            try:
                pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Publish failed on {pub.topic_name}: {e}')

    def shutdown(self):
        self._shutdown_event.set()
        for s in self.zenoh_subs:
            s.undeclare()
        for t in self.zenoh_lv_tokens:
            t.undeclare()
        if self.zenoh_session:
            self.zenoh_session.close()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ROS2ZenohBridge()
    except Exception:
        traceback.print_exc()
        rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0
