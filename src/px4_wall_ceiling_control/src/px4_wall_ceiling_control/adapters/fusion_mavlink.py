"""Named PARAM_REQUEST_READ/PARAM_SET via the existing MAVROS link.

No serial device is opened, no full parameter pull, no cache-only readback.
MAVLink has no parameter transaction ID: serialize exchanges, filter target,
name/type/value and pre-dispatch packets, and retain write intent on timeout.
"""
import math
import struct
import threading
import time

from px4_wall_ceiling_control.domain.wall_sensor_fusion import FLOW, RANGE


def decode_integer_parameter(message, target_system, target_component):
    if (message.msgid != 22 or message.sysid != target_system
            or message.compid != target_component or message.framing_status != 1):
        return None
    payload = struct.pack('<{}Q'.format(len(message.payload64)), *message.payload64)[:message.len]
    # MAVLink2 may trim zero tails. PARAM_VALUE is 25 bytes before trimming.
    if len(payload) < 8 or len(payload) > 25:
        return None
    payload = payload.ljust(25, b'\0')
    name = payload[8:24].split(b'\0', 1)[0].decode('ascii', errors='replace')
    if name not in (FLOW, RANGE) or payload[24] != 6:  # MAV_PARAM_TYPE_INT32 (bytewise PX4 encoding)
        return None
    value = struct.unpack('<i', payload[:4])[0]
    return (name, value) if 0 <= value <= 2147483647 else None


class MavrosParameterTransport:
    def __init__(self, namespace):
        import rospy
        from mavros_msgs.msg import Mavlink
        from pymavlink.dialects.v20 import common
        self.ros, self.message_type = rospy, Mavlink
        self.timeout = float(rospy.get_param('~parameter_reply_timeout_s', .60))
        self.retries = int(rospy.get_param('~parameter_reply_retries', 2))
        if not .1 <= self.timeout <= 1. or not 0 <= self.retries <= 2:
            raise ValueError('invalid targeted parameter reply budget')
        self.target_system = int(rospy.get_param(namespace+'/target_system_id', 1))
        self.target_component = int(rospy.get_param(namespace+'/target_component_id', 1))
        source_system = int(rospy.get_param('~parameter_source_system_id', 255))
        source_component = int(rospy.get_param('~parameter_source_component_id', 191))
        if not all(1 <= v <= 255 for v in (self.target_system, self.target_component, source_system, source_component)):
            raise ValueError('invalid MAVLink source/target identity')
        self.encoder = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
        self.condition = threading.Condition()
        self.responses = []
        self.publisher = rospy.Publisher('/mavlink/to', Mavlink, queue_size=5)
        self.subscriber = rospy.Subscriber('/mavlink/from', Mavlink, self._on_message,
                                           queue_size=100, tcp_nodelay=True)

    def _on_message(self, message):
        received = time.monotonic()  # before waiting for our short memory lock
        value = decode_integer_parameter(message, self.target_system, self.target_component)
        if value is None:
            return
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0:
            return
        with self.condition:
            self.responses.append((received, stamp, value[0], value[1]))
            self.responses = self.responses[-32:]
            self.condition.notify_all()

    def _publish(self, parameter, expected):
        if expected is None:
            packet = self.encoder.param_request_read_encode(self.target_system,
                self.target_component, parameter.encode('ascii'), -1)
        else:
            # PARAM_SET transports int32 bits in the float field, not a numeric cast.
            encoded = struct.unpack('<f', struct.pack('<i', expected))[0]
            packet = self.encoder.param_set_encode(self.target_system,
                self.target_component, parameter.encode('ascii'), encoded, 6)
        packet.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq+1) & 255
        header = packet.get_header()
        raw = packet.get_payload()
        padded = raw+b'\0'*((-len(raw)) % 8)
        message = self.message_type()
        message.header.stamp = self.ros.Time.now()
        message.framing_status, message.magic = 1, 253
        message.len, message.seq = header.mlen, header.seq
        message.sysid, message.compid, message.msgid = header.srcSystem, header.srcComponent, header.msgId
        message.checksum = packet.get_crc()
        message.payload64 = list(struct.unpack('<{}Q'.format(len(padded)//8), padded))
        self.publisher.publish(message)

    def _exchange(self, parameter, expected=None):
        for _attempt in range(self.retries+1):
            with self.condition:
                self.responses.clear()
                began, ros_began = time.monotonic(), self.ros.Time.now().to_sec()
                self._publish(parameter, expected)
                deadline = began+self.timeout
                while True:
                    for received, stamp, name, value in self.responses:
                        if (received >= began and stamp >= ros_began and name == parameter
                                and (expected is None or value == expected)):
                            return value
                    remaining = deadline-time.monotonic()
                    if remaining <= 0 or self.ros.is_shutdown():
                        break
                    self.condition.wait(remaining)
        raise RuntimeError('fresh PARAM_VALUE timeout: '+parameter)

    def execute(self, operation):
        if operation.kind == 'set':
            if operation.parameter not in (FLOW, RANGE) or not 0 <= operation.value <= 2147483647:
                raise ValueError('unexpected parameter write')
            value = self._exchange(operation.parameter, operation.value)
            if type(value) is not int or value != operation.value:
                raise ValueError('PARAM_SET acknowledgement mismatch')
            return {operation.parameter: value}
        return {parameter: self._exchange(parameter) for parameter in (FLOW, RANGE)}
