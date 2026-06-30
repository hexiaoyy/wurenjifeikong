"""
通信拓扑模块
绘制 GCS-OBC-FCU 通信拓扑结构图
"""

import json
import time
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class CommNode:
    """通信节点"""
    name: str
    node_type: str  # GCS, OBC, FCU, SENSOR, GPS, ESC
    ip: str = ""
    port: int = 0
    status: str = "online"  # online, offline, warning
    x: float = 0
    y: float = 0


@dataclass
class CommLink:
    """通信链路"""
    source: str
    target: str
    protocol: str = "MAVLink"
    baud_rate: int = 57600
    status: str = "active"  # active, disconnected
    latency_ms: float = 0


@dataclass
class MAVLinkMessage:
    """MAVLink报文"""
    msg_id: int
    msg_name: str
    source: str
    target: str
    timestamp: float
    data: dict = field(default_factory=dict)


class CommTopology:
    """通信拓扑管理"""

    def __init__(self):
        self.nodes: List[CommNode] = []
        self.links: List[CommLink] = []
        self.message_log: List[MAVLinkMessage] = []
        self.heartbeat_count = 0
        self.last_heartbeat = 0

        self._init_default_topology()

    def _init_default_topology(self):
        """初始化默认的GCS-OBC-FCU拓扑结构"""
        self.nodes = [
            CommNode("地面站 (GCS)", "GCS", "192.168.1.100", 14550, "online", 200, 400),
            CommNode("机载计算机 (OBC)", "OBC", "192.168.1.10", 14551, "online", 400, 400),
            CommNode("飞控 (FCU)", "FCU", "10.0.0.1", 14550, "online", 600, 400),
            CommNode("GPS模块", "SENSOR", "", 0, "online", 700, 250),
            CommNode("IMU传感器", "SENSOR", "", 0, "online", 700, 400),
            CommNode("电子调速器 (ESC)", "SENSOR", "", 0, "online", 700, 550),
            CommNode("气压计", "SENSOR", "", 0, "online", 600, 250),
            CommNode("图传模块", "SENSOR", "", 0, "online", 600, 550),
        ]

        self.links = [
            CommLink("地面站 (GCS)", "机载计算机 (OBC)", "MAVLink/UDP", 115200),
            CommLink("机载计算机 (OBC)", "飞控 (FCU)", "MAVLink/UART", 57600),
            CommLink("飞控 (FCU)", "GPS模块", "UART", 115200),
            CommLink("飞控 (FCU)", "IMU传感器", "UART", 115200),
            CommLink("飞控 (FCU)", "电子调速器 (ESC)", "PWM/DShot", 0),
            CommLink("飞控 (FCU)", "气压计", "UART", 115200),
            CommLink("机载计算机 (OBC)", "图传模块", "HDMI/RTSP", 0),
        ]

    def add_heartbeat(self, source="FCU"):
        """添加心跳包"""
        now = time.time()
        self.heartbeat_count += 1
        self.last_heartbeat = now

        msg = MAVLinkMessage(
            msg_id=0,
            msg_name="HEARTBEAT",
            source=source,
            target="ALL",
            timestamp=now,
            data={"seq": self.heartbeat_count, "type": "QUADROTOR",
                  "autopilot": "PX4", "base_mode": 217, "custom_mode": 0},
        )
        self.message_log.append(msg)

        # 更新链路延迟(模拟)
        for link in self.links:
            if link.source == source or link.target == source:
                link.latency_ms = 5 + (hash(str(now)) % 20) / 10.0

        return msg

    def generate_status_data(self):
        """生成当前状态数据"""
        now = time.time()
        return {
            "nodes": [
                {
                    "name": n.name,
                    "type": n.node_type,
                    "ip": n.ip,
                    "port": n.port,
                    "status": n.status,
                    "x": n.x, "y": n.y,
                }
                for n in self.nodes
            ],
            "links": [
                {
                    "source": l.source,
                    "target": l.target,
                    "protocol": l.protocol,
                    "baud_rate": l.baud_rate,
                    "status": l.status,
                    "latency_ms": round(l.latency_ms, 1),
                }
                for l in self.links
            ],
            "stats": {
                "heartbeat_count": self.heartbeat_count,
                "last_heartbeat": self.last_heartbeat,
                "uptime": now - (self.last_heartbeat - 300) if self.last_heartbeat > 0 else 0,
            },
        }

    def get_recent_messages(self, limit=50):
        """获取最近的报文记录"""
        return self.message_log[-limit:]

    def get_node_status_summary(self):
        """获取节点状态摘要"""
        online = sum(1 for n in self.nodes if n.status == "online")
        offline = sum(1 for n in self.nodes if n.status == "offline")
        return {
            "total": len(self.nodes),
            "online": online,
            "offline": offline,
        }

    def generate_topology_html(self):
        """生成拓扑图HTML(SVG)"""
        nodes = self.nodes
        links = self.links

        # 节点颜色映射
        type_colors = {
            "GCS": "#2196F3",
            "OBC": "#4CAF50",
            "FCU": "#FF9800",
            "SENSOR": "#9C27B0",
        }
        status_colors = {
            "online": "#4CAF50",
            "offline": "#F44336",
            "warning": "#FF9800",
        }

        svg_width = 800
        svg_height = 500

        lines_html = []

        # 绘制连接线
        for link in links:
            src = next((n for n in nodes if n.name == link.source), None)
            tgt = next((n for n in nodes if n.name == link.target), None)
            if src and tgt:
                link_color = "#4CAF50" if link.status == "active" else "#F44336"
                opacity = 0.6 if link.status == "active" else 0.3
                lines_html.append(
                    f'<line x1="{src.x}" y1="{src.y}" x2="{tgt.x}" y2="{tgt.y}" '
                    f'stroke="{link_color}" stroke-width="2" opacity="{opacity}" '
                    f'marker-end="url(#arrowhead)"/>'
                )
                # 链路标签
                mid_x = (src.x + tgt.x) / 2
                mid_y = (src.y + tgt.y) / 2
                latency = link.latency_ms
                lines_html.append(
                    f'<text x="{mid_x}" y="{mid_y - 5}" fill="#666" font-size="9" '
                    f'text-anchor="middle">{link.protocol} | {latency:.1f}ms</text>'
                )

        # 绘制节点
        for node in nodes:
            color = type_colors.get(node.node_type, "#607D8B")
            border_color = status_colors.get(node.status, "#999")
            # 节点背景
            lines_html.append(
                f'<rect x="{node.x - 60}" y="{node.y - 25}" width="120" height="50" '
                f'rx="8" ry="8" fill="{color}" stroke="{border_color}" stroke-width="2" '
                f'opacity="0.9"/>'
            )
            # 状态指示灯
            lines_html.append(
                f'<circle cx="{node.x - 45}" cy="{node.y - 10}" r="5" fill="{border_color}"/>'
            )
            # 节点名称
            lines_html.append(
                f'<text x="{node.x}" y="{node.y + 2}" fill="white" font-size="11" '
                f'font-weight="bold" text-anchor="middle">{node.name}</text>'
            )
            # IP信息
            if node.ip:
                lines_html.append(
                    f'<text x="{node.x}" y="{node.y + 16}" fill="white" font-size="9" '
                    f'text-anchor="middle" opacity="0.8">{node.ip}:{node.port}</text>'
                )

        # 箭头标记定义
        arrow_def = (
            '<defs><marker id="arrowhead" markerWidth="10" markerHeight="7" '
            'refX="10" refY="3.5" orient="auto">'
            '<polygon points="0 0, 10 3.5, 0 7" fill="#4CAF50" opacity="0.6"/>'
            '</marker></defs>'
        )

        # 图例
        legend = (
            '<g transform="translate(20, 460)">'
            '<rect x="0" y="0" width="12" height="12" rx="2" fill="#2196F3"/>'
            '<text x="18" y="11" fill="#333" font-size="10">GCS</text>'
            '<rect x="60" y="0" width="12" height="12" rx="2" fill="#4CAF50"/>'
            '<text x="78" y="11" fill="#333" font-size="10">OBC</text>'
            '<rect x="120" y="0" width="12" height="12" rx="2" fill="#FF9800"/>'
            '<text x="138" y="11" fill="#333" font-size="10">FCU</text>'
            '<rect x="180" y="0" width="12" height="12" rx="2" fill="#9C27B0"/>'
            '<text x="198" y="11" fill="#333" font-size="10">传感器</text>'
            '<circle cx="265" cy="6" r="5" fill="#4CAF50"/>'
            '<text x="275" y="11" fill="#333" font-size="10">在线</text>'
            '<circle cx="310" cy="6" r="5" fill="#F44336"/>'
            '<text x="320" y="11" fill="#333" font-size="10">离线</text>'
            '</g>'
        )

        svg = (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}" '
            f'viewBox="0 0 {svg_width} {svg_height}">'
            f'<rect width="{svg_width}" height="{svg_height}" fill="#FAFAFA" rx="12"/>'
            f'{arrow_def}'
            f'<text x="400" y="30" fill="#333" font-size="16" font-weight="bold" '
            f'text-anchor="middle">GCS - OBC - FCU 通信拓扑结构</text>'
            f'{"".join(lines_html)}'
            f'{legend}'
            f'</svg>'
        )

        return svg
