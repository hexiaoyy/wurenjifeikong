"""
MAVLink 数据仿真模块
模拟MAVLink协议的心跳包、姿态数据、GPS数据等
"""

import json
import time
import math
import random
from dataclasses import dataclass, field
from typing import List, Dict, Optional


# MAVLink 消息ID常量
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_RC_CHANNELS = 65
MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62
MAVLINK_MSG_ID_VFR_HUD = 74
MAVLINK_MSG_ID_RAW_IMU = 27


@dataclass
class MAVLinkPacket:
    """MAVLink数据包"""
    msg_id: int
    msg_name: str
    sysid: int = 1
    compid: int = 1
    seq: int = 0
    timestamp: float = 0
    payload: Dict = field(default_factory=dict)
    raw_hex: str = ""


class MAVLinkSimulator:
    """MAVLink仿真器"""

    def __init__(self):
        self.seq_counter = 0
        self.heartbeat_seq = 0
        self.running = False
        self.flight_mode = "STABILIZED"
        self.armed = False

        # 无人机状态
        self.state = {
            "lat": 32.1080,
            "lng": 118.7993,
            "alt": 0.0,
            "heading": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
            "ground_speed": 0.0,
            "air_speed": 0.0,
            "climb_rate": 0.0,
            "battery_voltage": 22.2,
            "battery_remaining": 100,
            "throttle": 0,
            "gps_fix": 0,
            "gps_satellites": 0,
            "flight_mode": "STABILIZED",
            "armed": False,
        }

        self.message_log: List[MAVLinkPacket] = []
        self.max_log_size = 200

    def _next_seq(self):
        self.seq_counter = (self.seq_counter + 1) % 256
        return self.seq_counter

    def _create_packet(self, msg_id, msg_name, payload):
        return MAVLinkPacket(
            msg_id=msg_id,
            msg_name=msg_name,
            seq=self._next_seq(),
            timestamp=time.time(),
            payload=payload,
        )

    def generate_heartbeat(self):
        """生成心跳包"""
        self.heartbeat_seq += 1
        packet = self._create_packet(
            MAVLINK_MSG_ID_HEARTBEAT,
            "HEARTBEAT",
            {
                "type": "QUADROTOR",
                "autopilot": "PX4",
                "base_mode": 217 if self.armed else 89,
                "custom_mode": 0,
                "system_status": 4 if self.armed else 3,
                "mavlink_version": 3,
            },
        )
        self._log(packet)
        return packet

    def generate_attitude(self):
        """生成姿态数据"""
        noise = random.gauss(0, 0.5)
        self.state["pitch"] = max(-30, min(30, self.state["pitch"] + noise * 0.1))
        self.state["roll"] = max(-30, min(30, self.state["roll"] + noise * 0.1))
        self.state["yaw"] = (self.state["yaw"] + random.gauss(0, 1)) % 360

        packet = self._create_packet(
            MAVLINK_MSG_ID_ATTITUDE,
            "ATTITUDE",
            {
                "time_boot_ms": int(time.time() * 1000),
                "pitch": round(self.state["pitch"], 2),
                "roll": round(self.state["roll"], 2),
                "yaw": round(self.state["yaw"], 2),
                "pitchspeed": round(random.gauss(0, 0.5), 3),
                "rollspeed": round(random.gauss(0, 0.5), 3),
                "yawspeed": round(random.gauss(0, 0.3), 3),
            },
        )
        self._log(packet)
        return packet

    def generate_gps_position(self, target_lat=None, target_lng=None, target_alt=None):
        """生成GPS位置数据"""
        if self.armed:
            # 模拟位置变化
            if target_lat and target_lng:
                self.state["lat"] += (target_lat - self.state["lat"]) * 0.02
                self.state["lng"] += (target_lng - self.state["lng"]) * 0.02
            if target_alt is not None:
                self.state["alt"] += (target_alt - self.state["alt"]) * 0.05
            self.state["heading"] = (self.state["heading"] + random.gauss(0, 2)) % 360
            self.state["ground_speed"] = abs(random.gauss(5, 1))
            self.state["air_speed"] = abs(random.gauss(5.5, 0.8))
            self.state["climb_rate"] = random.gauss(0, 0.3)
            self.state["gps_fix"] = 3
            self.state["gps_satellites"] = random.randint(10, 14)
        else:
            self.state["ground_speed"] = 0
            self.state["air_speed"] = 0
            self.state["climb_rate"] = 0

        packet = self._create_packet(
            MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            "GLOBAL_POSITION_INT",
            {
                "time_boot_ms": int(time.time() * 1000),
                "lat": int(self.state["lat"] * 1e7),
                "lon": int(self.state["lng"] * 1e7),
                "alt": int(self.state["alt"] * 1000),
                "relative_alt": int((self.state["alt"] - 10) * 1000),
                "vx": int(self.state["ground_speed"] * 100 * math.cos(math.radians(self.state["heading"]))),
                "vy": int(self.state["ground_speed"] * 100 * math.sin(math.radians(self.state["heading"]))),
                "vz": int(self.state["climb_rate"] * 100),
                "hdg": int(self.state["heading"] * 100),
            },
        )
        self._log(packet)
        return packet

    def generate_sys_status(self):
        """生成系统状态"""
        if self.armed:
            self.state["battery_voltage"] = max(18, self.state["battery_voltage"] - random.uniform(0, 0.01))
            self.state["battery_remaining"] = max(0, self.state["battery_remaining"] - random.uniform(0, 0.05))

        packet = self._create_packet(
            MAVLINK_MSG_ID_SYS_STATUS,
            "SYS_STATUS",
            {
                "onboard_control_sensors_present": 0x3FFF,
                "onboard_control_sensors_enabled": 0x3FFF,
                "onboard_control_sensors_health": 0x3FFF,
                "load": random.randint(10, 30),
                "voltage_battery": round(self.state["battery_voltage"], 3),
                "current_battery": random.randint(5, 15) if self.armed else 0,
                "battery_remaining": int(self.state["battery_remaining"]),
                "drop_rate_comm": 0,
                "errors_comm": 0,
            },
        )
        self._log(packet)
        return packet

    def generate_vfr_hud(self):
        """生成VFR HUD数据"""
        packet = self._create_packet(
            MAVLINK_MSG_ID_VFR_HUD,
            "VFR_HUD",
            {
                "airspeed": round(self.state["air_speed"], 1),
                "groundspeed": round(self.state["ground_speed"], 1),
                "heading": int(self.state["heading"]),
                "throttle": self.state["throttle"],
                "alt": round(self.state["alt"], 1),
                "climb": round(self.state["climb_rate"], 1),
            },
        )
        self._log(packet)
        return packet

    def generate_all(self):
        """生成所有类型的数据包"""
        packets = []
        packets.append(self.generate_heartbeat())
        packets.append(self.generate_attitude())
        packets.append(self.generate_gps_position())
        packets.append(self.generate_sys_status())
        packets.append(self.generate_vfr_hud())
        return packets

    def arm(self):
        """解锁电机"""
        self.armed = True
        self.state["armed"] = True
        self.state["throttle"] = 50

    def disarm(self):
        """锁定电机"""
        self.armed = False
        self.state["armed"] = False
        self.state["throttle"] = 0

    def set_mode(self, mode):
        self.flight_mode = mode
        self.state["flight_mode"] = mode

    def _log(self, packet):
        self.message_log.append(packet)
        if len(self.message_log) > self.max_log_size:
            self.message_log = self.message_log[-self.max_log_size:]

    def get_log(self, limit=50):
        return self.message_log[-limit:]

    def get_log_table_data(self, limit=50):
        """获取适合st.dataframe的表格数据"""
        log = self.get_log(limit)
        rows = []
        for pkt in reversed(log):
            rows.append({
                "时间戳": time.strftime("%H:%M:%S", time.localtime(pkt.timestamp)),
                "消息ID": f"0x{pkt.msg_id:02X}",
                "消息名称": pkt.msg_name,
                "序列号": pkt.seq,
                "摘要": json.dumps(pkt.payload, ensure_ascii=False)[:80],
            })
        return rows
