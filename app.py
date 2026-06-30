"""
UAV Flight Planning & Monitoring System
无人机飞行规划与监控系统 - Streamlit可视化界面 (单文件版本)

功能模块:
  3.1 地图定位模块 - OpenStreetMap显示、WGS84/GCJ02坐标转换
  3.2 障碍物与航线规划模块 - 多边形圈选、航线规划(飞越/绕飞/最优)
  3.3 飞行监控模块 - MAVLink数据仿真、实时状态监控
  3.4 通信链路展示模块 - GCS-OBC-FCU拓扑图、MAVLink报文显示

运行方式: streamlit run app_single.py
"""

# ============================================================
# 标准库与第三方库导入
# ============================================================
import math
import json
import os
import sys
import time
import random
import heapq
from dataclasses import dataclass, field
from typing import List, Dict

import folium
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from folium.plugins import Draw

# ============================================================
# 坐标转换 (coord_transform.py)
# ============================================================
_PI = math.pi
_A = 6378245.0
_EE = 0.00669342162296594


def _out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)


def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * _PI) + 40.0 * math.sin(lat / 3.0 * _PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * _PI) + 320 * math.sin(lat * _PI / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * _PI) + 20.0 * math.sin(2.0 * lng * _PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * _PI) + 40.0 * math.sin(lng / 3.0 * _PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * _PI) + 300.0 * math.sin(lng / 30.0 * _PI)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lng, lat):
    if _out_of_china(lng, lat):
        return lng, lat
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * _PI
    magic = math.sin(radlat)
    magic = 1 - _EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1 - _EE)) / (magic * sqrtmagic) * _PI)
    dlng = (dlng * 180.0) / (_A / sqrtmagic * math.cos(radlat) * _PI)
    return lng + dlng, lat + dlat


def gcj02_to_wgs84(lng, lat):
    if _out_of_china(lng, lat):
        return lng, lat
    wgs_lng, wgs_lat = lng, lat
    for _ in range(5):
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        wgs_lng += lng - gcj_lng
        wgs_lat += lat - gcj_lat
    return wgs_lng, wgs_lat


NJVT_CENTER_WGS84 = (118.6405, 32.1592)
NJVT_CENTER_GCJ02 = wgs84_to_gcj02(*NJVT_CENTER_WGS84)

# ============================================================
# Obstacle & ObstacleManager (obstacle_manager.py)
# ============================================================
OBSTACLE_FILE = "obstacles_data.json"


class Obstacle:
    def __init__(self, name, coordinates, height, color="#FF4444"):
        self.name = name
        self.coordinates = coordinates
        self.height = height
        self.color = color

    def to_dict(self):
        return {"name": self.name, "coordinates": self.coordinates, "height": self.height, "color": self.color}

    @classmethod
    def from_dict(cls, data):
        return cls(name=data["name"], coordinates=[tuple(c) for c in data["coordinates"]], height=data["height"], color=data.get("color", "#FF4444"))

    @property
    def centroid(self):
        if not self.coordinates:
            return 0, 0
        lngs = [c[0] for c in self.coordinates]
        lats = [c[1] for c in self.coordinates]
        return sum(lngs) / len(lngs), sum(lats) / len(lats)

    @property
    def latlng_list(self):
        return [list(reversed(c)) for c in self.coordinates]


class ObstacleManager:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, name, coordinates, height, color="#FF4444"):
        obstacle = Obstacle(name, coordinates, height, color)
        self.obstacles.append(obstacle)
        return obstacle

    def remove_obstacle(self, index):
        if 0 <= index < len(self.obstacles):
            removed = self.obstacles.pop(index)
            self.save_to_file()
            return removed
        return None

    def update_obstacle_height(self, index, new_height):
        if 0 <= index < len(self.obstacles):
            self.obstacles[index].height = new_height

    def save_to_file(self, filepath=None):
        path = filepath or OBSTACLE_FILE
        data = [obs.to_dict() for obs in self.obstacles]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def load_from_file(self, filepath=None):
        path = filepath or OBSTACLE_FILE
        if not os.path.exists(path):
            return False
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.obstacles = [Obstacle.from_dict(d) for d in data]
            return True
        except (json.JSONDecodeError, KeyError):
            return False

    def to_json_string(self):
        return json.dumps([obs.to_dict() for obs in self.obstacles], ensure_ascii=False, indent=2)

# ============================================================
# FlightPlanner (flight_planner.py)
# ============================================================


class FlightPlanner:
    def __init__(self):
        self.obstacles = []
        self.flight_height = 50
        self.safety_radius = 10

    def set_parameters(self, flight_height, safety_radius):
        self.flight_height = flight_height
        self.safety_radius = safety_radius

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def _lnglat_to_meters(self, lng, lat, ref_lng, ref_lat):
        dx = (lng - ref_lng) * 111320 * math.cos(ref_lat * math.pi / 180)
        dy = (lat - ref_lat) * 110540
        return dx, dy

    def _meters_to_lnglat(self, dx, dy, ref_lng, ref_lat):
        lng = ref_lng + dx / (111320 * math.cos(ref_lat * math.pi / 180))
        lat = ref_lat + dy / 110540
        return lng, lat

    def _build_obstacle_polygons(self, ref_lng, ref_lat):
        polygons = []
        for obs in self.obstacles:
            if len(obs.coordinates) < 3:
                continue
            meter_coords = [self._lnglat_to_meters(lng, lat, ref_lng, ref_lat) for lng, lat in obs.coordinates]
            poly = Polygon(meter_coords)
            if not poly.is_valid:
                poly = poly.buffer(0)
            buffered = poly.buffer(self.safety_radius)
            polygons.append(buffered)
        return polygons

    def _get_ref_point(self):
        if not self.obstacles:
            return 118.6405, 32.1592
        lngs = [c[0] for obs in self.obstacles for c in obs.coordinates]
        lats = [c[1] for obs in self.obstacles for c in obs.coordinates]
        return sum(lngs) / len(lngs), sum(lats) / len(lats)

    def plan_straight(self, start, end):
        path = [start, end]
        total_dist = self._calc_distance(start, end)
        return {"name": "直线飞越", "path": path, "distance": total_dist, "waypoints": [start, end], "clears_obstacles": self._check_clearance(path)}

    def plan_avoid_left(self, start, end):
        return self._plan_avoid(start, end, direction="left")

    def plan_avoid_right(self, start, end):
        return self._plan_avoid(start, end, direction="right")

    def plan_optimal(self, start, end):
        ref_lng, ref_lat = self._get_ref_point()
        polygons = self._build_obstacle_polygons(ref_lng, ref_lat)
        if not polygons:
            return self.plan_straight(start, end)
        merged = unary_union(polygons)
        sx, sy = self._lnglat_to_meters(start[0], start[1], ref_lng, ref_lat)
        ex, ey = self._lnglat_to_meters(end[0], end[1], ref_lng, ref_lat)
        direct = LineString([(sx, sy), (ex, ey)])
        if not direct.intersects(merged):
            return self.plan_straight(start, end)
        waypoints_m = self._dijkstra_path(Point(sx, sy), Point(ex, ey), merged)
        waypoints = [self._meters_to_lnglat(wx, wy, ref_lng, ref_lat) for wx, wy in waypoints_m]
        total_dist = sum(self._calc_distance(waypoints[i], waypoints[i+1]) for i in range(len(waypoints) - 1))
        return {"name": "最优路径(最短弧线)", "path": waypoints, "distance": total_dist, "waypoints": waypoints, "clears_obstacles": self._check_clearance(waypoints)}

    def _plan_avoid(self, start, end, direction="left"):
        ref_lng, ref_lat = self._get_ref_point()
        polygons = self._build_obstacle_polygons(ref_lng, ref_lat)
        if not polygons:
            return self.plan_straight(start, end)
        merged = unary_union(polygons)
        sx, sy = self._lnglat_to_meters(start[0], start[1], ref_lng, ref_lat)
        ex, ey = self._lnglat_to_meters(end[0], end[1], ref_lng, ref_lat)
        direct = LineString([(sx, sy), (ex, ey)])
        if not direct.intersects(merged):
            return self.plan_straight(start, end)
        sign = 1 if direction == "left" else -1
        avoidance_dist = self.safety_radius + 15
        waypoints_m = [(sx, sy)]
        path_line = LineString([(sx, sy), (ex, ey)])
        intersecting = [p for p in polygons if path_line.intersects(p)]
        if not intersecting:
            return self.plan_straight(start, end)
        current = Point(sx, sy)
        for poly in sorted(intersecting, key=lambda p: p.centroid.distance(current)):
            dx = ex - current.x
            dy = ey - current.y
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            nx = -dy / length * sign
            ny = dx / length * sign
            nearest = poly.boundary.interpolate(poly.boundary.project(current))
            cx = nearest.x + nx * avoidance_dist
            cy = nearest.y + ny * avoidance_dist
            tangent_angle = math.atan2(ny, nx)
            for i in range(1, 9):
                angle = tangent_angle + (math.pi * i / 9) * (-sign)
                ax = cx + avoidance_dist * math.cos(angle)
                ay = cy + avoidance_dist * math.sin(angle)
                if not merged.contains(Point(ax, ay)):
                    waypoints_m.append((ax, ay))
        waypoints_m.append((ex, ey))
        waypoints = [self._meters_to_lnglat(wx, wy, ref_lng, ref_lat) for wx, wy in waypoints_m]
        total_dist = sum(self._calc_distance(waypoints[i], waypoints[i+1]) for i in range(len(waypoints) - 1))
        label = "左绕飞" if direction == "left" else "右绕飞"
        return {"name": label, "path": waypoints, "distance": total_dist, "waypoints": waypoints, "clears_obstacles": self._check_clearance(waypoints)}

    def _dijkstra_path(self, start, end, obstacle_union):
        nodes = [start, end]
        geoms = obstacle_union.geoms if hasattr(obstacle_union, "geoms") else [obstacle_union]
        for geom in geoms:
            if geom.geom_type == "Polygon" and geom.exterior:
                for coord in list(geom.exterior.coords)[::3]:
                    nodes.append(Point(coord))
        n = len(nodes)
        adj = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                line = LineString([(nodes[i].x, nodes[i].y), (nodes[j].x, nodes[j].y)])
                if not obstacle_union.crosses(line) and not obstacle_union.contains(line):
                    d = nodes[i].distance(nodes[j])
                    adj[i].append((j, d))
                    adj[j].append((i, d))
        dist = [float("inf")] * n
        prev = [-1] * n
        dist[0] = 0
        pq = [(0, 0)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == 1:
                break
            for v, w in adj[u]:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        path_nodes = []
        cur = 1
        while cur != -1:
            path_nodes.append((nodes[cur].x, nodes[cur].y))
            cur = prev[cur]
        path_nodes.reverse()
        return path_nodes if len(path_nodes) > 1 else [(start.x, start.y), (end.x, end.y)]

    def _calc_distance(self, p1, p2):
        R = 6371000
        lat1, lng1 = math.radians(p1[1]), math.radians(p1[0])
        lat2, lng2 = math.radians(p2[1]), math.radians(p2[0])
        dlat = lat2 - lat1
        dlng = lng2 - lng1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlng/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def _check_clearance(self, path):
        if not self.obstacles or len(path) < 2:
            return True
        ref_lng, ref_lat = self._get_ref_point()
        polygons = self._build_obstacle_polygons(ref_lng, ref_lat)
        if not polygons:
            return True
        merged = unary_union(polygons)
        for i in range(len(path) - 1):
            sx, sy = self._lnglat_to_meters(path[i][0], path[i][1], ref_lng, ref_lat)
            ex, ey = self._lnglat_to_meters(path[i+1][0], path[i+1][1], ref_lng, ref_lat)
            if merged.intersects(LineString([(sx, sy), (ex, ey)])):
                return False
        return True

    def plan_all(self, start, end):
        results = [self.plan_straight(start, end), self.plan_avoid_left(start, end), self.plan_avoid_right(start, end), self.plan_optimal(start, end)]
        results.sort(key=lambda r: r["distance"])
        for i, r in enumerate(results):
            r["rank"] = i + 1
        return results

# ============================================================
# MapUtils (map_utils.py)
# ============================================================
DEFAULT_CENTER_WGS84 = (118.6405, 32.1592)
DEFAULT_ZOOM = 17


class MapUtils:
    @staticmethod
    def create_base_map(center=None, zoom=None, use_gcj02=False):
        if center is None:
            center = DEFAULT_CENTER_WGS84
        if zoom is None:
            zoom = DEFAULT_ZOOM
        map_center = [center[1], center[0]]
        m = folium.Map(location=map_center, zoom_start=zoom, tiles="OpenStreetMap", attr="OpenStreetMap contributors")
        folium.TileLayer(tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", attr="Esri World Imagery", name="卫星地图", overlay=False, control=True).add_to(m)
        folium.TileLayer(tiles="https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", attr="OpenTopoMap", name="地形图", overlay=False, control=True).add_to(m)
        folium.LayerControl().add_to(m)
        draw = Draw(draw_options={"polygon": True, "polyline": False, "rectangle": False, "circle": False, "circlemarker": False, "marker": True}, edit_options={"edit": True, "remove": True})
        m.add_child(draw)
        return m

    @staticmethod
    def add_obstacle_polygons(m, obstacles, use_gcj02=False):
        colors = ["#FF4444", "#FF8800", "#FFAA00", "#FF6666", "#CC3300"]
        for i, obs in enumerate(obstacles):
            coords = obs.latlng_list
            color = obs.color or colors[i % len(colors)]
            popup_html = f'<div style="font-family:sans-serif;min-width:150px;"><b>{obs.name}</b><br>高度: {obs.height}m<br>顶点数: {len(obs.coordinates)}</div>'
            folium.Polygon(locations=coords, color=color, fill=True, fill_color=color, fill_opacity=0.3, weight=2, popup=folium.Popup(popup_html, max_width=200), tooltip=obs.name).add_to(m)
            centroid = obs.centroid
            folium.Marker(location=[centroid[1], centroid[0]], icon=folium.DivIcon(html=f'<div style="font-size:10px;color:{color};font-weight:bold;text-shadow:-1px 0 white,0 1px white,1px 0 white,0 -1px white;">{obs.name} ({obs.height}m)</div>')).add_to(m)

    @staticmethod
    def add_flight_path(m, plan_result, color="#2196F3", use_gcj02=False):
        waypoints = plan_result["waypoints"]
        if len(waypoints) < 2:
            return
        coords = [[wp[1], wp[0]] for wp in waypoints]
        folium.PolyLine(locations=coords, color=color, weight=3, opacity=0.8, popup=f"{plan_result['name']} - 距离: {plan_result['distance']:.1f}m").add_to(m)
        for i, wp in enumerate(waypoints):
            icon_color = "green" if i == 0 else ("red" if i == len(waypoints) - 1 else "blue")
            folium.Marker(location=[wp[1], wp[0]], icon=folium.DivIcon(html=f'<div style="background-color:{icon_color};color:white;border-radius:50%;width:20px;height:20px;display:flex;align-items:center;justify-content:center;font-size:10px;font-weight:bold;">{i}</div>'), tooltip=f"航点 {i}: ({wp[0]:.6f}, {wp[1]:.6f})").add_to(m)
        folium.Marker(location=[waypoints[0][1], waypoints[0][0]], popup=f"起点 A ({waypoints[0][0]:.6f}, {waypoints[0][1]:.6f})", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
        folium.Marker(location=[waypoints[-1][1], waypoints[-1][0]], popup=f"终点 B ({waypoints[-1][0]:.6f}, {waypoints[-1][1]:.6f})", icon=folium.Icon(color="red", icon="flag-checkered", prefix="fa")).add_to(m)

    @staticmethod
    def add_safety_buffer(m, obstacles, safety_radius, use_gcj02=False):
        for obs in obstacles:
            centroid = obs.centroid
            folium.Circle(location=[centroid[1], centroid[0]], radius=safety_radius, color="#FF9800", fill=True, fill_color="#FF9800", fill_opacity=0.1, weight=1, dash_array="5, 5", popup=f"安全半径: {safety_radius}m").add_to(m)

# ============================================================
# 通信拓扑数据类 (comm_topology.py)
# ============================================================


@dataclass
class CommNode:
    name: str
    node_type: str
    ip: str = ""
    port: int = 0
    status: str = "online"
    x: float = 0
    y: float = 0


@dataclass
class CommLink:
    source: str
    target: str
    protocol: str = "MAVLink"
    baud_rate: int = 57600
    status: str = "active"
    latency_ms: float = 0


@dataclass
class MAVLinkMessage:
    msg_id: int
    msg_name: str
    source: str
    target: str
    timestamp: float
    data: dict = field(default_factory=dict)


class CommTopology:
    def __init__(self):
        self.nodes: List[CommNode] = []
        self.links: List[CommLink] = []
        self.message_log: List[MAVLinkMessage] = []
        self.heartbeat_count = 0
        self.last_heartbeat = 0
        self._init_default_topology()

    def _init_default_topology(self):
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
        now = time.time()
        self.heartbeat_count += 1
        self.last_heartbeat = now
        msg = MAVLinkMessage(msg_id=0, msg_name="HEARTBEAT", source=source, target="ALL", timestamp=now, data={"seq": self.heartbeat_count, "type": "QUADROTOR", "autopilot": "PX4", "base_mode": 217, "custom_mode": 0})
        self.message_log.append(msg)
        for link in self.links:
            if link.source == source or link.target == source:
                link.latency_ms = 5 + (hash(str(now)) % 20) / 10.0
        return msg

    def get_recent_messages(self, limit=50):
        return self.message_log[-limit:]

    def get_node_status_summary(self):
        online = sum(1 for n in self.nodes if n.status == "online")
        offline = sum(1 for n in self.nodes if n.status == "offline")
        return {"total": len(self.nodes), "online": online, "offline": offline}

    def generate_topology_html(self):
        nodes = self.nodes
        links = self.links
        type_colors = {"GCS": "#2196F3", "OBC": "#4CAF50", "FCU": "#FF9800", "SENSOR": "#9C27B0"}
        status_colors = {"online": "#4CAF50", "offline": "#F44336", "warning": "#FF9800"}
        svg_width = 800
        svg_height = 500
        lines_html = []
        for link in links:
            src = next((n for n in nodes if n.name == link.source), None)
            tgt = next((n for n in nodes if n.name == link.target), None)
            if src and tgt:
                link_color = "#4CAF50" if link.status == "active" else "#F44336"
                opacity = 0.6 if link.status == "active" else 0.3
                lines_html.append(f'<line x1="{src.x}" y1="{src.y}" x2="{tgt.x}" y2="{tgt.y}" stroke="{link_color}" stroke-width="2" opacity="{opacity}" marker-end="url(#arrowhead)"/>')
                mid_x = (src.x + tgt.x) / 2
                mid_y = (src.y + tgt.y) / 2
                latency = link.latency_ms
                lines_html.append(f'<text x="{mid_x}" y="{mid_y - 5}" fill="#666" font-size="9" text-anchor="middle">{link.protocol} | {latency:.1f}ms</text>')
        for node in nodes:
            color = type_colors.get(node.node_type, "#607D8B")
            border_color = status_colors.get(node.status, "#999")
            lines_html.append(f'<rect x="{node.x - 60}" y="{node.y - 25}" width="120" height="50" rx="8" ry="8" fill="{color}" stroke="{border_color}" stroke-width="2" opacity="0.9"/>')
            lines_html.append(f'<circle cx="{node.x - 45}" cy="{node.y - 10}" r="5" fill="{border_color}"/>')
            lines_html.append(f'<text x="{node.x}" y="{node.y + 2}" fill="white" font-size="11" font-weight="bold" text-anchor="middle">{node.name}</text>')
            if node.ip:
                lines_html.append(f'<text x="{node.x}" y="{node.y + 16}" fill="white" font-size="9" text-anchor="middle" opacity="0.8">{node.ip}:{node.port}</text>')
        arrow_def = '<defs><marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto"><polygon points="0 0, 10 3.5, 0 7" fill="#4CAF50" opacity="0.6"/></marker></defs>'
        legend = '<g transform="translate(20, 460)"><rect x="0" y="0" width="12" height="12" rx="2" fill="#2196F3"/><text x="18" y="11" fill="#333" font-size="10">GCS</text><rect x="60" y="0" width="12" height="12" rx="2" fill="#4CAF50"/><text x="78" y="11" fill="#333" font-size="10">OBC</text><rect x="120" y="0" width="12" height="12" rx="2" fill="#FF9800"/><text x="138" y="11" fill="#333" font-size="10">FCU</text><rect x="180" y="0" width="12" height="12" rx="2" fill="#9C27B0"/><text x="198" y="11" fill="#333" font-size="10">传感器</text><circle cx="265" cy="6" r="5" fill="#4CAF50"/><text x="275" y="11" fill="#333" font-size="10">在线</text><circle cx="310" cy="6" r="5" fill="#F44336"/><text x="320" y="11" fill="#333" font-size="10">离线</text></g>'
        svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}"><rect width="{svg_width}" height="{svg_height}" fill="#FAFAFA" rx="12"/>{arrow_def}<text x="400" y="30" fill="#333" font-size="16" font-weight="bold" text-anchor="middle">GCS - OBC - FCU 通信拓扑结构</text>{"".join(lines_html)}{legend}</svg>'
        return svg

# ============================================================
# MAVLink 仿真器 (mavlink_sim.py)
# ============================================================
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_VFR_HUD = 74


@dataclass
class MAVLinkPacket:
    msg_id: int
    msg_name: str
    sysid: int = 1
    compid: int = 1
    seq: int = 0
    timestamp: float = 0
    payload: Dict = field(default_factory=dict)
    raw_hex: str = ""


class MAVLinkSimulator:
    def __init__(self):
        self.seq_counter = 0
        self.heartbeat_seq = 0
        self.running = False
        self.flight_mode = "STABILIZED"
        self.armed = False
        self.state = {"lat": 32.1592, "lng": 118.6405, "alt": 0.0, "heading": 0.0, "pitch": 0.0, "roll": 0.0, "yaw": 0.0, "ground_speed": 0.0, "air_speed": 0.0, "climb_rate": 0.0, "battery_voltage": 22.2, "battery_remaining": 100, "throttle": 0, "gps_fix": 0, "gps_satellites": 0, "flight_mode": "STABILIZED", "armed": False}
        self.message_log: List[MAVLinkPacket] = []
        self.max_log_size = 200

    def _next_seq(self):
        self.seq_counter = (self.seq_counter + 1) % 256
        return self.seq_counter

    def _create_packet(self, msg_id, msg_name, payload):
        return MAVLinkPacket(msg_id=msg_id, msg_name=msg_name, seq=self._next_seq(), timestamp=time.time(), payload=payload)

    def generate_heartbeat(self):
        self.heartbeat_seq += 1
        packet = self._create_packet(MAVLINK_MSG_ID_HEARTBEAT, "HEARTBEAT", {"type": "QUADROTOR", "autopilot": "PX4", "base_mode": 217 if self.armed else 89, "custom_mode": 0, "system_status": 4 if self.armed else 3, "mavlink_version": 3})
        self._log(packet)
        return packet

    def generate_attitude(self):
        noise = random.gauss(0, 0.5)
        self.state["pitch"] = max(-30, min(30, self.state["pitch"] + noise * 0.1))
        self.state["roll"] = max(-30, min(30, self.state["roll"] + noise * 0.1))
        self.state["yaw"] = (self.state["yaw"] + random.gauss(0, 1)) % 360
        packet = self._create_packet(MAVLINK_MSG_ID_ATTITUDE, "ATTITUDE", {"time_boot_ms": int(time.time()*1000), "pitch": round(self.state["pitch"],2), "roll": round(self.state["roll"],2), "yaw": round(self.state["yaw"],2), "pitchspeed": round(random.gauss(0,0.5),3), "rollspeed": round(random.gauss(0,0.5),3), "yawspeed": round(random.gauss(0,0.3),3)})
        self._log(packet)
        return packet

    def generate_gps_position(self, target_lat=None, target_lng=None, target_alt=None):
        if self.armed:
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
        packet = self._create_packet(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT", {"time_boot_ms": int(time.time()*1000), "lat": int(self.state["lat"]*1e7), "lon": int(self.state["lng"]*1e7), "alt": int(self.state["alt"]*1000), "relative_alt": int((self.state["alt"]-10)*1000), "vx": int(self.state["ground_speed"]*100*math.cos(math.radians(self.state["heading"]))), "vy": int(self.state["ground_speed"]*100*math.sin(math.radians(self.state["heading"]))), "vz": int(self.state["climb_rate"]*100), "hdg": int(self.state["heading"]*100)})
        self._log(packet)
        return packet

    def generate_sys_status(self):
        if self.armed:
            self.state["battery_voltage"] = max(18, self.state["battery_voltage"] - random.uniform(0, 0.01))
            self.state["battery_remaining"] = max(0, self.state["battery_remaining"] - random.uniform(0, 0.05))
        packet = self._create_packet(MAVLINK_MSG_ID_SYS_STATUS, "SYS_STATUS", {"onboard_control_sensors_present": 0x3FFF, "onboard_control_sensors_enabled": 0x3FFF, "onboard_control_sensors_health": 0x3FFF, "load": random.randint(10, 30), "voltage_battery": round(self.state["battery_voltage"], 3), "current_battery": random.randint(5, 15) if self.armed else 0, "battery_remaining": int(self.state["battery_remaining"]), "drop_rate_comm": 0, "errors_comm": 0})
        self._log(packet)
        return packet

    def generate_vfr_hud(self):
        packet = self._create_packet(MAVLINK_MSG_ID_VFR_HUD, "VFR_HUD", {"airspeed": round(self.state["air_speed"],1), "groundspeed": round(self.state["ground_speed"],1), "heading": int(self.state["heading"]), "throttle": self.state["throttle"], "alt": round(self.state["alt"],1), "climb": round(self.state["climb_rate"],1)})
        self._log(packet)
        return packet

    def generate_all(self):
        return [self.generate_heartbeat(), self.generate_attitude(), self.generate_gps_position(), self.generate_sys_status(), self.generate_vfr_hud()]

    def arm(self):
        self.armed = True
        self.state["armed"] = True
        self.state["throttle"] = 50

    def disarm(self):
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
        log = self.get_log(limit)
        rows = []
        for pkt in reversed(log):
            rows.append({"时间戳": time.strftime("%H:%M:%S", time.localtime(pkt.timestamp)), "消息ID": f"0x{pkt.msg_id:02X}", "消息名称": pkt.msg_name, "序列号": pkt.seq, "摘要": json.dumps(pkt.payload, ensure_ascii=False)[:80]})
        return rows

# ============================================================
# Streamlit 应用
# ============================================================
import streamlit as st
from streamlit_folium import st_folium


def render_map(m, key=None, height=600):
    """MapUtils.render_map 的独立函数版本，避免在类定义中依赖streamlit"""
    return st_folium(m, key=key, height=height, use_container_width=True)


# 页面配置(必须在第一个st命令之前)
st.set_page_config(
    page_title="无人机飞行规划与监控系统",
    page_icon="🛩️",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
    .stApp { max-width: 100%; }
    .block-container { padding-top: 1rem; }
</style>
""", unsafe_allow_html=True)


def init_session_state():
    defaults = {
        "page": "map",
        "obstacle_mgr": None,
        "flight_planner": FlightPlanner(),
        "comm_topology": CommTopology(),
        "mavlink_sim": MAVLinkSimulator(),
        "monitor_running": False,
        "selected_plan": None,
        "point_a": list(NJVT_CENTER_WGS84),
        "point_b": [118.6435, 32.1572],
        "flight_height": 50.0,
        "safety_radius": 10.0,
    }
    for key, val in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = val
    if st.session_state.obstacle_mgr is None:
        mgr = ObstacleManager()
        mgr.load_from_file()
        st.session_state.obstacle_mgr = mgr


def sidebar_navigation():
    st.sidebar.title("导航菜单")
    st.sidebar.markdown("---")
    pages = {
        "map": "3.1 地图定位模块",
        "obstacle": "3.2 障碍物与航线规划",
        "monitor": "3.3 飞行监控模块",
        "comm": "3.4 通信链路展示",
    }
    selected = st.sidebar.radio(
        "功能模块",
        list(pages.keys()),
        format_func=lambda x: pages[x],
        index=list(pages.keys()).index(st.session_state.get("page", "map")),
    )
    st.session_state.page = selected
    st.sidebar.markdown("---")
    st.sidebar.caption("南京科技职业学院\n无人机飞行规划与监控系统 v1.0")


# ============================================================
# 3.1 地图定位模块
# ============================================================
def page_map():
    st.header("3.1 地图定位模块")
    st.markdown("基于 OpenStreetMap 的校园地图显示，支持 WGS-84/GCJ-02 坐标系转换")

    col1, col2 = st.columns([2, 1])

    with col1:
        use_gcj02 = st.checkbox("使用 GCJ-02 坐标系（火星坐标系）", value=False)
        center = NJVT_CENTER_GCJ02 if use_gcj02 else NJVT_CENTER_WGS84

        m = MapUtils.create_base_map(center=center)
        folium.Marker(
            location=[center[1], center[0]],
            popup=f"校园中心 ({center[0]:.6f}, {center[1]:.6f})",
            icon=folium.Icon(color="blue", icon="info-sign"),
        ).add_to(m)

        obs_mgr = st.session_state.obstacle_mgr
        if obs_mgr.obstacles:
            MapUtils.add_obstacle_polygons(m, obs_mgr.obstacles)

        if st.session_state.point_b:
            pa = st.session_state.point_a
            pb = st.session_state.point_b
            folium.Marker(location=[pa[1], pa[0]], popup=f"起点A", icon=folium.Icon(color="green", icon="play")).add_to(m)
            folium.Marker(location=[pb[1], pb[0]], popup=f"终点B", icon=folium.Icon(color="red", icon="flag-checkered")).add_to(m)

        result = render_map(m, key="main_map", height=500)
        if result and result.get("last_clicked"):
            c = result["last_clicked"]
            if c.get("lat") and c.get("lng"):
                st.info(f"点击位置: 纬度={c['lat']:.6f}, 经度={c['lng']:.6f}")

    with col2:
        st.subheader("坐标转换工具")
        st.markdown("**WGS-84 -> GCJ-02**")
        c1 = st.columns(2)
        wgs_lng = c1[0].number_input("经度", value=118.6405, key="wgs_lng", format="%.6f")
        wgs_lat = c1[1].number_input("纬度", value=32.1592, key="wgs_lat", format="%.6f")
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        st.success(f"GCJ-02: ({gcj_lng:.6f}, {gcj_lat:.6f})")

        st.markdown("**GCJ-02 -> WGS-84**")
        c2 = st.columns(2)
        gi_lng = c2[0].number_input("经度", value=round(gcj_lng, 6), key="gcj_lng_in", format="%.6f")
        gi_lat = c2[1].number_input("纬度", value=round(gcj_lat, 6), key="gcj_lat_in", format="%.6f")
        rl, ra = gcj02_to_wgs84(gi_lng, gi_lat)
        st.success(f"WGS-84: ({rl:.6f}, {ra:.6f})")

        st.markdown("---")
        st.subheader("坐标参考点")
        st.code(f"南京科技职业学院(校园中心)\nWGS-84: ({NJVT_CENTER_WGS84[0]}, {NJVT_CENTER_WGS84[1]})\nGCJ-02: ({NJVT_CENTER_GCJ02[0]:.6f}, {NJVT_CENTER_GCJ02[1]:.6f})")

        st.markdown("---")
        st.subheader("设置航点坐标")
        c3 = st.columns(2)
        a_lng = c3[0].number_input("A点经度", value=118.6385, key="a_lng", format="%.6f")
        a_lat = c3[0].number_input("A点纬度", value=32.1612, key="a_lat", format="%.6f")
        b_lng = c3[1].number_input("B点经度", value=118.6435, key="b_lng", format="%.6f")
        b_lat = c3[1].number_input("B点纬度", value=32.1572, key="b_lat", format="%.6f")
        if st.button("确认航点", use_container_width=True):
            st.session_state.point_a = [a_lng, a_lat]
            st.session_state.point_b = [b_lng, b_lat]
            st.success("航点已设置!")
            st.rerun()


# ============================================================
# 3.2 障碍物与航线规划模块
# ============================================================
def page_obstacle():
    st.header("3.2 障碍物与航线规划模块")
    obs_mgr = st.session_state.obstacle_mgr

    tab1, tab2, tab3 = st.tabs(["障碍物管理", "航线规划", "JSON数据"])

    with tab1:
        col_left, col_right = st.columns([2, 1])

        with col_left:
            st.subheader("3.2.1 多边形圈选障碍物")
            m = MapUtils.create_base_map()
            if obs_mgr.obstacles:
                MapUtils.add_obstacle_polygons(m, obs_mgr.obstacles)
            render_map(m, key="obstacle_draw_map", height=500)

            st.markdown("---")
            st.subheader("手动添加障碍物")
            with st.form("add_obstacle_form"):
                obs_name = st.text_input("障碍物名称", value=f"障碍物{len(obs_mgr.obstacles) + 1}")
                obs_height = st.number_input("高度(米)", min_value=0, max_value=500, value=30)
                coords_text = st.text_area(
                    "坐标列表(每行: 经度,纬度)",
                    value="118.7980,32.1085\n118.7985,32.1083\n118.7988,32.1086\n118.7983,32.1088",
                    height=100,
                )
                if st.form_submit_button("添加障碍物", use_container_width=True):
                    try:
                        coords = []
                        for line in coords_text.strip().split("\n"):
                            if line.strip():
                                lng, lat = line.strip().split(",")
                                coords.append((float(lng), float(lat)))
                        if len(coords) >= 3:
                            obs_mgr.add_obstacle(obs_name, coords, obs_height)
                            obs_mgr.save_to_file()
                            st.success(f"已添加障碍物: {obs_name}")
                            st.rerun()
                        else:
                            st.error("多边形至少需要3个顶点")
                    except (ValueError, IndexError):
                        st.error("坐标格式错误，请使用: 经度,纬度")

        with col_right:
            st.subheader("已标记障碍物")
            if obs_mgr.obstacles:
                for i, obs in enumerate(obs_mgr.obstacles):
                    with st.expander(f"{obs.name} ({obs.height}m)"):
                        st.write(f"顶点数: {len(obs.coordinates)}")
                        new_h = st.number_input("调整高度", min_value=0, max_value=500, value=int(obs.height), key=f"h_{i}")
                        if st.button("更新高度", key=f"upd_{i}"):
                            obs_mgr.update_obstacle_height(i, new_h)
                            obs_mgr.save_to_file()
                            st.success("高度已更新")
                        if st.button("删除", key=f"del_{i}"):
                            obs_mgr.remove_obstacle(i)
                            st.rerun()
                st.markdown("---")
                if st.button("清空所有", use_container_width=True):
                    obs_mgr.obstacles.clear()
                    obs_mgr.save_to_file()
                    st.rerun()
            else:
                st.info("暂无障碍物，请手动添加")

    with tab2:
        st.subheader("3.2.3 飞行参数设置")
        c1, c2, c3 = st.columns(3)
        st.session_state.flight_height = c1.number_input("飞行高度(m)", min_value=1, max_value=500, value=50, key="fh")
        st.session_state.safety_radius = c2.number_input("安全半径(m)", min_value=1, max_value=100, value=10, key="sr")
        c3.markdown(f"**航点**<br>A: `({st.session_state.point_a[0]:.4f}, {st.session_state.point_a[1]:.4f})`<br>"
                    f"B: `({st.session_state.point_b[0]:.4f}, {st.session_state.point_b[1]:.4f})`")

        if not st.session_state.point_b:
            st.warning("请先设置B航点")
            return

        planner = st.session_state.flight_planner
        planner.set_parameters(st.session_state.flight_height, st.session_state.safety_radius)
        planner.set_obstacles(obs_mgr.obstacles)
        start = tuple(st.session_state.point_a)
        end = tuple(st.session_state.point_b)

        if st.button("生成所有航线方案", use_container_width=True, type="primary"):
            with st.spinner("正在规划航线..."):
                st.session_state.selected_plan = planner.plan_all(start, end)

        if st.session_state.selected_plan:
            results = st.session_state.selected_plan
            st.subheader("3.2.4 航线规划结果对比")
            cols = st.columns(len(results))
            colors = ["#4CAF50", "#2196F3", "#FF9800", "#9C27B0"]

            for idx, (r, color) in enumerate(zip(results, colors)):
                with cols[idx]:
                    icon = "✅" if r["clears_obstacles"] else "⚠️"
                    st.markdown(f"**#{r['rank']} {r['name']}** {icon}")
                    st.metric("距离", f"{r['distance']:.1f} m")
                    st.metric("航点", len(r["waypoints"]))
                    if st.button(f"选择", key=f"p_{idx}", use_container_width=True):
                        st.session_state.current_display_plan = r
                        st.rerun()

            if "current_display_plan" in st.session_state:
                sel = st.session_state.current_display_plan
                st.markdown("---")
                cm, ci = st.columns([2, 1])
                with cm:
                    mp = MapUtils.create_base_map()
                    MapUtils.add_obstacle_polygons(mp, obs_mgr.obstacles)
                    MapUtils.add_safety_buffer(mp, obs_mgr.obstacles, st.session_state.safety_radius)
                    MapUtils.add_flight_path(mp, sel, color=colors[results.index(sel) % len(colors)])
                    render_map(mp, key="plan_disp", height=500)
                with ci:
                    st.subheader(f"方案: {sel['name']}")
                    st.write(f"距离: **{sel['distance']:.1f}m**")
                    st.write(f"航点: **{len(sel['waypoints'])}**")
                    st.write(f"安全: {'✅' if sel['clears_obstacles'] else '❌'}")
                    for i, wp in enumerate(sel["waypoints"]):
                        st.code(f"WP{i}: ({wp[0]:.6f}, {wp[1]:.6f})")

    with tab3:
        st.subheader("3.2.2 障碍物 JSON 数据")
        js = obs_mgr.to_json_string()
        st.code(js, language="json")
        st.download_button("下载 JSON", js.encode("utf-8"), "obstacles_data.json", "application/json")

        st.markdown("---")
        uploaded = st.file_uploader("上传 JSON 文件", type=["json"])
        if uploaded:
            try:
                data = json.load(uploaded)
                obs_mgr.obstacles.clear()
                for d in data:
                    obs_mgr.obstacles.append(Obstacle.from_dict(d))
                obs_mgr.save_to_file()
                st.success(f"已导入 {len(data)} 个障碍物")
                st.rerun()
            except Exception as e:
                st.error(f"导入失败: {e}")


# ============================================================
# 3.3 飞行监控模块
# ============================================================
def page_monitor():
    st.header("3.3 飞行监控模块")
    sim = st.session_state.mavlink_sim

    # 控制栏
    c1, c2, c3 = st.columns(3)
    with c1:
        if not st.session_state.monitor_running:
            if st.button("▶ 开始监控", use_container_width=True, type="primary"):
                sim.arm()
                sim.running = True
                st.session_state.monitor_running = True
        else:
            if st.button("⏹ 停止监控", use_container_width=True):
                sim.disarm()
                sim.running = False
                st.session_state.monitor_running = False

    with c2:
        mode = st.selectbox("飞行模式", ["STABILIZED", "AUTO", "GUIDED", "LOITER", "RTL", "LAND"], key="fm_sel")
        if st.button("切换模式", use_container_width=True):
            sim.set_mode(mode)

    with c3:
        st.markdown(f"心跳: `{sim.heartbeat_seq}` | "
                    f"状态: `{'运行' if sim.running else '停止'}` | "
                    f"电机: `{'解锁' if sim.armed else '锁定'}`")

    if st.session_state.monitor_running:
        sim.generate_all()

    # 仪表盘
    st.markdown("---")
    state = sim.state
    mc = st.columns(6)
    for col, (label, val) in zip(mc, [
        ("纬度", f"{state['lat']:.6f}°"), ("经度", f"{state['lng']:.6f}°"),
        ("高度", f"{state['alt']:.1f}m"), ("地速", f"{state['ground_speed']:.1f}m/s"),
        ("航向", f"{state['heading']:.0f}°"), ("电池", f"{state['battery_remaining']:.0f}%"),
    ]):
        col.metric(label, val)

    d1, d2, d3 = st.columns(3)
    with d1:
        st.subheader("姿态")
        st.markdown(f"Pitch: **{state['pitch']:.2f}°**\nRoll: **{state['roll']:.2f}°**\nYaw: **{state['yaw']:.2f}°**")
    with d2:
        st.subheader("导航")
        st.markdown(f"GPS: **{'3D Fix' if state['gps_fix'] == 3 else 'No Fix'}**\n"
                    f"卫星: **{state['gps_satellites']}**\n"
                    f"垂直速度: **{state['climb_rate']:.2f}m/s**")
    with d3:
        st.subheader("电源")
        bc = "🟢" if state["battery_remaining"] > 30 else ("🟡" if state["battery_remaining"] > 15 else "🔴")
        st.markdown(f"电压: **{state['battery_voltage']:.2f}V** {bc}\n"
                    f"剩余: **{state['battery_remaining']:.0f}%**\n"
                    f"油门: **{state['throttle']}%**\n"
                    f"模式: **{state['flight_mode']}**")

    # 监控地图
    st.markdown("---")
    st.subheader("实时位置地图")
    mm = MapUtils.create_base_map(center=(state["lng"], state["lat"]))
    folium.Marker(
        location=[state["lat"], state["lng"]],
        popup=f"无人机 | 高度:{state['alt']:.1f}m 速度:{state['ground_speed']:.1f}m/s",
        icon=folium.DivIcon(html='<div style="font-size:24px;">✈️</div>', icon_size=(30, 30)),
    ).add_to(mm)

    obs_mgr = st.session_state.obstacle_mgr
    if obs_mgr.obstacles:
        MapUtils.add_obstacle_polygons(mm, obs_mgr.obstacles)
    if st.session_state.get("current_display_plan"):
        MapUtils.add_flight_path(mm, st.session_state.current_display_plan)
    render_map(mm, key="mon_map", height=400)

    # MAVLink 报文日志
    st.markdown("---")
    st.subheader("MAVLink 报文日志")
    log_data = sim.get_log_table_data(30)
    if log_data:
        st.dataframe(log_data, use_container_width=True, hide_index=True)
    else:
        st.info("暂无报文数据")

    # 自动刷新(使用st_autorefresh方式)
    if st.session_state.monitor_running:
        # 使用streamlit原生方式: interval参数控制刷新
        st_autorefresh = st.columns([1])[0]
        st.markdown('<meta http-equiv="refresh" content="2">', unsafe_allow_html=True)


# ============================================================
# 3.4 通信链路展示模块
# ============================================================
def page_comm():
    st.header("3.4 通信链路展示模块")
    comm = st.session_state.comm_topology

    tab1, tab2 = st.tabs(["3.4.1 通信拓扑", "3.4.2 MAVLink数据流"])

    with tab1:
        st.subheader("GCS - OBC - FCU 通信拓扑结构")
        st.components.v1.html(comm.generate_topology_html(), height=530)

        st.markdown("---")
        ss = comm.get_node_status_summary()
        s1, s2, s3 = st.columns(3)
        s1.metric("总节点", ss["total"])
        s2.metric("在线", ss["online"])
        s3.metric("离线", ss["offline"])

        for node in comm.nodes:
            icon = "🟢" if node.status == "online" else "🔴"
            with st.expander(f"{icon} {node.name} [{node.node_type}]"):
                ci = st.columns(3)
                ci[0].write(f"IP: `{node.ip}`")
                ci[1].write(f"端口: `{node.port}`")
                ci[2].write(f"状态: `{node.status}`")

        st.markdown("---")
        st.subheader("链路信息")
        lc = st.columns(min(len(comm.links), 4))
        for i, link in enumerate(comm.links):
            with lc[i % len(lc)]:
                lat = link.latency_ms
                lc_icon = "🟢" if lat < 10 else ("🟡" if lat < 20 else "🔴")
                st.markdown(f"**{link.source[:4]} -> {link.target[:4]}**\n"
                            f"协议: `{link.protocol}`\n"
                            f"延迟: {lc_icon} `{lat:.1f}ms`")

    with tab2:
        st.subheader("3.4.2 MAVLink 数据流与报文显示")

        # ---- 数据流控制面板 ----
        ctrl1, ctrl2, ctrl3, ctrl4 = st.columns(4)
        with ctrl1:
            auto_stream = st.toggle("自动发送数据流", value=False, key="auto_stream_toggle")
        with ctrl2:
            stream_rate = st.selectbox("发送频率", ["1Hz", "2Hz", "5Hz", "10Hz"], index=1, key="stream_rate_sel")
        with ctrl3:
            filter_msg = st.multiselect(
                "消息类型筛选",
                ["ALL", "HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS", "VFR_HUD"],
                default=["ALL"],
                key="msg_filter_sel",
            )
        with ctrl4:
            if st.button("清空报文记录", use_container_width=True):
                comm.heartbeat_count = 0
                comm.message_log.clear()
                st.rerun()

        # 自动数据流: 根据频率生成各类MAVLink消息
        if auto_stream:
            rate_map = {"1Hz": 1.0, "2Hz": 0.5, "5Hz": 0.2, "10Hz": 0.1}
            interval = rate_map.get(stream_rate, 0.5)
            sim = st.session_state.mavlink_sim

            # 生成完整数据流并注入到通信拓扑的报文记录
            packets = sim.generate_all()
            for pkt in packets:
                comm_msg = MAVLinkMessage(
                    msg_id=pkt.msg_id,
                    msg_name=pkt.msg_name,
                    source="FCU",
                    target="GCS",
                    timestamp=pkt.timestamp,
                    data=pkt.payload,
                )
                comm.message_log.append(comm_msg)
                if len(comm.message_log) > 500:
                    comm.message_log = comm.message_log[-500:]
            comm.heartbeat_count = sim.heartbeat_seq
            comm.last_heartbeat = time.time()

            # 自动刷新
            st.markdown(f'<meta http-equiv="refresh" content="{int(interval)}">', unsafe_allow_html=True)

        # ---- 数据流统计面板 ----
        st.markdown("---")
        s1, s2, s3, s4, s5 = st.columns(5)
        total_msgs = len(comm.message_log)
        s1.metric("总报文数", total_msgs)

        # 按类型统计
        msg_counts = {}
        for msg in comm.message_log:
            msg_counts[msg.msg_name] = msg_counts.get(msg.msg_name, 0) + 1
        s2.metric("消息类型数", len(msg_counts))
        s3.metric("心跳包", msg_counts.get("HEARTBEAT", 0))
        s4.metric("姿态数据", msg_counts.get("ATTITUDE", 0))
        s5.metric("GPS位置", msg_counts.get("GLOBAL_POSITION_INT", 0))

        # ---- 数据流方向可视化 ----
        st.markdown("---")
        st.subheader("数据流方向示意")
        flow_cols = st.columns(3)
        # FCU -> OBC 数据流
        fcu_msgs = [m for m in comm.message_log if m.source == "FCU"]
        obc_msgs = [m for m in comm.message_log if m.source == "OBC"]

        with flow_cols[0]:
            st.markdown("**FCU -> OBC** (传感器数据)")
            st.progress(min(1.0, len(fcu_msgs) / 100))
            st.caption(f"已发送 {len(fcu_msgs)} 条报文")
            flow_detail_fcu = {}
            for m in fcu_msgs[-50:]:
                flow_detail_fcu[m.msg_name] = flow_detail_fcu.get(m.msg_name, 0) + 1
            if flow_detail_fcu:
                for name, count in sorted(flow_detail_fcu.items(), key=lambda x: -x[1]):
                    st.markdown(f"  - `{name}`: {count}")

        with flow_cols[1]:
            st.markdown("**OBC -> GCS** (转发数据)")
            st.progress(min(1.0, len(obc_msgs) / 100))
            st.caption(f"已发送 {len(obc_msgs)} 条报文")

        with flow_cols[2]:
            st.markdown("**GCS -> OBC** (指令)")
            gcs_msgs = [m for m in comm.message_log if m.source == "GCS"]
            st.progress(min(1.0, len(gcs_msgs) / 100))
            st.caption(f"已发送 {len(gcs_msgs)} 条报文")

        # ---- 数据流时序图(文本形式) ----
        st.markdown("---")
        st.subheader("数据流时序图")
        st.markdown("展示最近报文在各节点间的传递顺序")

        recent_all = comm.get_recent_messages(15)
        if recent_all:
            timeline_html = """
            <div style="font-family: monospace; font-size: 11px; overflow-x: auto; padding: 10px;
                        background: #1e1e1e; color: #d4d4d4; border-radius: 8px; line-height: 1.6;">
            <div style="display: flex; gap: 40px; margin-bottom: 8px; color: #888;">
                <span style="width:80px;">时间</span>
                <span style="width:60px;">方向</span>
                <span style="width:120px;">消息类型</span>
                <span style="width:200px;">关键字段</span>
            </div>
            """
            for msg in reversed(recent_all):
                t = time.strftime("%H:%M:%S", time.localtime(msg.timestamp))
                direction = f"{msg.source[:3]}->{msg.target[:3]}"
                # 消息类型颜色
                color_map = {
                    "HEARTBEAT": "#569cd6",
                    "ATTITUDE": "#ce9178",
                    "GLOBAL_POSITION_INT": "#6a9955",
                    "SYS_STATUS": "#dcdcaa",
                    "VFR_HUD": "#c586c0",
                }
                color = color_map.get(msg.msg_name, "#d4d4d4")

                # 提取关键字段
                keys = []
                if "type" in msg.data:
                    keys.append(f"type={msg.data['type']}")
                if "pitch" in msg.data:
                    keys.append(f"P={msg.data['pitch']}°")
                if "roll" in msg.data:
                    keys.append(f"R={msg.data['roll']}°")
                if "alt" in msg.data:
                    keys.append(f"alt={msg.data['alt']}")
                if "voltage_battery" in msg.data:
                    keys.append(f"V={msg.data['voltage_battery']}V")
                if "groundspeed" in msg.data:
                    keys.append(f"GS={msg.data['groundspeed']}m/s")
                key_str = " | ".join(keys[:4])

                timeline_html += (
                    f'<div style="display: flex; gap: 40px;">'
                    f'<span style="width:80px; color:#888;">{t}</span>'
                    f'<span style="width:60px; color:#4ec9b0;">{direction}</span>'
                    f'<span style="width:120px; color:{color}; font-weight:bold;">{msg.msg_name}</span>'
                    f'<span style="width:200px; color:#9cdcfe;">{key_str}</span>'
                    f'</div>'
                )
            timeline_html += "</div>"
            st.components.v1.html(timeline_html, height=320)
        else:
            st.info("暂无数据流记录，请开启自动发送或手动发送心跳包")

        # ---- 手动发送控制 ----
        st.markdown("---")
        st.subheader("手动发送 MAVLink 报文")
        m1, m2, m3 = st.columns(3)
        with m1:
            if st.button("发送心跳包", use_container_width=True, type="primary"):
                comm.add_heartbeat()
                st.rerun()
        with m2:
            if st.button("发送姿态数据", use_container_width=True):
                sim = st.session_state.mavlink_sim
                pkt = sim.generate_attitude()
                comm.message_log.append(MAVLinkMessage(
                    msg_id=pkt.msg_id, msg_name=pkt.msg_name,
                    source="FCU", target="GCS", timestamp=pkt.timestamp, data=pkt.payload,
                ))
                st.rerun()
        with m3:
            if st.button("发送GPS位置", use_container_width=True):
                sim = st.session_state.mavlink_sim
                pkt = sim.generate_gps_position()
                comm.message_log.append(MAVLinkMessage(
                    msg_id=pkt.msg_id, msg_name=pkt.msg_name,
                    source="FCU", target="GCS", timestamp=pkt.timestamp, data=pkt.payload,
                ))
                st.rerun()

        # ---- 报文详细历史表格 ----
        st.markdown("---")
        st.subheader("报文历史记录")
        display_msgs = comm.get_recent_messages(50)
        if "ALL" not in filter_msg:
            display_msgs = [m for m in display_msgs if m.msg_name in filter_msg]
        if display_msgs:
            rows = []
            for msg in reversed(display_msgs):
                direction = f"{msg.source} -> {msg.target}"
                data_str = json.dumps(msg.data, ensure_ascii=False)
                rows.append({
                    "时间": time.strftime("%H:%M:%S", time.localtime(msg.timestamp)),
                    "方向": direction,
                    "MsgID": f"0x{msg.msg_id:02X}",
                    "消息名称": msg.msg_name,
                    "数据内容": data_str[:80] + ("..." if len(data_str) > 80 else ""),
                })
            st.dataframe(rows, use_container_width=True, hide_index=True, height=400)
        else:
            st.info("暂无报文记录")

        # ---- 最近报文原始JSON ----
        if comm.message_log:
            with st.expander("查看最新报文原始数据 (JSON)"):
                latest = comm.message_log[-1]
                st.json({
                    "msg_id": latest.msg_id,
                    "msg_name": latest.msg_name,
                    "source": latest.source,
                    "target": latest.target,
                    "timestamp": latest.timestamp,
                    "payload": latest.data,
                })


# ============================================================
# 主入口
# ============================================================
def main():
    init_session_state()
    sidebar_navigation()
    page = st.session_state.page
    if page == "map":
        page_map()
    elif page == "obstacle":
        page_obstacle()
    elif page == "monitor":
        page_monitor()
    elif page == "comm":
        page_comm()


if __name__ == "__main__":
    main()
