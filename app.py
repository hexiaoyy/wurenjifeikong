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


NJVT_CENTER_WGS84 = (118.7620, 32.2450)
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
            return 118.7620, 32.2450
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
DEFAULT_CENTER_WGS84 = (118.7620, 32.2450)
DEFAULT_ZOOM = 17


class MapUtils:
    @staticmethod
    def create_base_map(center=None, zoom=None, use_gcj02=False, layer_type="标准地图"):
        if center is None:
            center = DEFAULT_CENTER_WGS84
        if zoom is None:
            zoom = DEFAULT_ZOOM
        map_center = [center[1], center[0]]
        
        # 图层配置
        tiles_map = {
            "标准地图": ("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", "OpenStreetMap contributors"),
            "卫星地图": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", "Esri World Imagery"),
            "地形图": ("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", "OpenTopoMap"),
        }
        tiles_url, tiles_attr = tiles_map.get(layer_type, tiles_map["标准地图"])
        
        m = folium.Map(location=map_center, zoom_start=zoom, tiles=tiles_url, attr=tiles_attr)
        
        # 添加其他图层到 LayerControl
        for name, (url, attr) in tiles_map.items():
            if url != tiles_url:
                folium.TileLayer(tiles=url, attr=attr, name=name, overlay=False, control=True).add_to(m)
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
        self.state = {"lat": 32.2450, "lng": 118.7620, "alt": 0.0, "heading": 0.0, "pitch": 0.0, "roll": 0.0, "yaw": 0.0, "ground_speed": 0.0, "air_speed": 0.0, "climb_rate": 0.0, "battery_voltage": 22.2, "battery_remaining": 100, "throttle": 0, "gps_fix": 0, "gps_satellites": 0, "flight_mode": "STABILIZED", "armed": False}
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


# ============================================================
# SVG 仪表盘绘制函数
# ============================================================

def _attitude_indicator_svg(pitch, roll):
    """绘制姿态指示器（人工地平线）SVG，根据pitch/roll角度偏移地平线"""
    # pitch: 俯仰角（度），roll: 横滚角（度）
    pitch_offset = max(-50, min(50, pitch)) * 0.8  # 限制偏移范围
    roll_angle = max(-60, min(60, roll))          # 限制横滚范围

    svg = f'''
    <svg viewBox="0 0 220 220" width="220" height="220" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <clipPath id="aiClip">
          <circle cx="110" cy="110" r="95"/>
        </clipPath>
      </defs>
      <!-- 外圈 -->
      <circle cx="110" cy="110" r="98" fill="none" stroke="#00d4ff" stroke-width="2" opacity="0.6"/>
      <!-- 背景圆 -->
      <circle cx="110" cy="110" r="95" fill="#0d1117"/>
      <!-- 天空和地面（用clipPath裁切，通过rotate和translate实现姿态变化） -->
      <g clip-path="url(#aiClip)">
        <g transform="rotate({roll_angle}, 110, 110)">
          <!-- 天空（蓝色） -->
          <rect x="0" y="0" width="220" height="110" fill="#1a5276" transform="translate(0, {pitch_offset})"/>
          <!-- 地面（棕色） -->
          <rect x="0" y="110" width="220" height="220" fill="#7d5a3c" transform="translate(0, {pitch_offset})"/>
          <!-- 地平线 -->
          <line x1="0" y1="110" x2="220" y2="110" stroke="white" stroke-width="2" transform="translate(0, {pitch_offset})"/>
          <!-- 俯仰刻度线 -->
          <line x1="90" y1="80" x2="110" y2="80" stroke="white" stroke-width="1" opacity="0.5" transform="translate(0, {pitch_offset})"/>
          <line x1="90" y1="95" x2="110" y2="95" stroke="white" stroke-width="1" opacity="0.7" transform="translate(0, {pitch_offset})"/>
          <line x1="90" y1="125" x2="110" y2="125" stroke="white" stroke-width="1" opacity="0.7" transform="translate(0, {pitch_offset})"/>
          <line x1="90" y1="140" x2="110" y2="140" stroke="white" stroke-width="1" opacity="0.5" transform="translate(0, {pitch_offset})"/>
        </g>
      </g>
      <!-- 固定十字准线 -->
      <line x1="70" y1="110" x2="95" y2="110" stroke="#f59e0b" stroke-width="2"/>
      <line x1="125" y1="110" x2="150" y2="110" stroke="#f59e0b" stroke-width="2"/>
      <circle cx="110" cy="110" r="3" fill="#f59e0b"/>
      <!-- 横滚角弧线 -->
      <path d="M 30 110 A 80 80 0 0 1 40 55" fill="none" stroke="#00d4ff" stroke-width="1.5" opacity="0.7"/>
      <path d="M 180 110 A 80 80 0 0 0 170 55" fill="none" stroke="#00d4ff" stroke-width="1.5" opacity="0.7"/>
      <!-- 横滚指示三角 -->
      <polygon points="110,18 105,28 115,28" fill="#f59e0b" transform="rotate({roll_angle}, 110, 110)"/>
      <!-- 数据标注 -->
      <text x="110" y="210" text-anchor="middle" fill="#a0a0a0" font-size="10" font-family="monospace">P:{pitch:.1f}° R:{roll:.1f}°</text>
    </svg>'''
    return svg


def _battery_svg(percentage, voltage):
    """绘制电池状态条SVG，根据电量百分比显示不同颜色"""
    pct = max(0, min(100, percentage))
    # 颜色阈值：>30% 绿色，>15% 黄色，<=15% 红色
    if pct > 30:
        bar_color = "#10b981"
    elif pct > 15:
        bar_color = "#f59e0b"
    else:
        bar_color = "#ef4444"

    bar_width = pct * 1.5  # 最大150px

    svg = f'''
    <svg viewBox="0 0 200 60" width="200" height="60" xmlns="http://www.w3.org/2000/svg">
      <!-- 电池外壳 -->
      <rect x="10" y="10" width="160" height="35" rx="4" ry="4" fill="none" stroke="#a0a0a0" stroke-width="2"/>
      <!-- 电池正极端子 -->
      <rect x="170" y="20" width="8" height="15" rx="2" ry="2" fill="#a0a0a0"/>
      <!-- 电池填充 -->
      <rect x="12" y="12" width="{bar_width}" height="31" rx="3" ry="3" fill="{bar_color}" opacity="0.85">
        <animate attributeName="opacity" values="0.85;1;0.85" dur="2s" repeatCount="indefinite"/>
      </rect>
      <!-- 百分比文字 -->
      <text x="90" y="33" text-anchor="middle" fill="white" font-size="14" font-weight="bold" font-family="monospace">{pct:.0f}%</text>
      <!-- 电压标注 -->
      <text x="90" y="57" text-anchor="middle" fill="#a0a0a0" font-size="10" font-family="monospace">{voltage:.2f}V</text>
    </svg>'''
    return svg


def _heading_svg(heading):
    """绘制航向罗盘SVG，指针指向当前航向"""
    hdg = heading % 360
    # 将航向角转为SVG旋转角（0°=北朝上，顺时针）
    svg = f'''
    <svg viewBox="0 0 200 200" width="200" height="200" xmlns="http://www.w3.org/2000/svg">
      <!-- 外圈 -->
      <circle cx="100" cy="100" r="92" fill="none" stroke="#00d4ff" stroke-width="2" opacity="0.5"/>
      <circle cx="100" cy="100" r="88" fill="#0d1117" opacity="0.8"/>
      <!-- 罗盘刻度 - 旋转整个刻度盘，使当前航向对准上方 -->
      <g transform="rotate({-hdg}, 100, 100)">
        <!-- 主方向标注 N/E/S/W -->
        <text x="100" y="28" text-anchor="middle" fill="#ef4444" font-size="16" font-weight="bold" font-family="sans-serif">N</text>
        <text x="100" y="183" text-anchor="middle" fill="#a0a0a0" font-size="12" font-family="sans-serif">S</text>
        <text x="178" y="104" text-anchor="middle" fill="#a0a0a0" font-size="12" font-family="sans-serif">E</text>
        <text x="22" y="104" text-anchor="middle" fill="#a0a0a0" font-size="12" font-family="sans-serif">W</text>
        <!-- 30度刻度线 -->
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(30, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(60, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(120, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(150, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(210, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(240, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(300, 100, 100)"/>
        <line x1="100" y1="16" x2="100" y2="22" stroke="#a0a0a0" stroke-width="1.5" transform="rotate(330, 100, 100)"/>
        <!-- 10度小刻度 -->
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(10, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(20, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(40, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(50, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(70, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(80, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(100, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(110, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(130, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(140, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(160, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(170, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(190, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(200, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(220, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(230, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(250, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(260, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(280, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(290, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(310, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(320, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(340, 100, 100)"/>
        <line x1="100" y1="18" x2="100" y2="24" stroke="#606060" stroke-width="0.8" transform="rotate(350, 100, 100)"/>
      </g>
      <!-- 固定指示箭头（始终朝上，指向当前航向） -->
      <polygon points="100,35 93,52 107,52" fill="#f59e0b"/>
      <line x1="100" y1="52" x2="100" y2="75" stroke="#f59e0b" stroke-width="2"/>
      <!-- 中心圆点 -->
      <circle cx="100" cy="100" r="4" fill="#0d1117" stroke="#f59e0b" stroke-width="1.5"/>
      <!-- 航向数字标注 -->
      <text x="100" y="196" text-anchor="middle" fill="#a0a0a0" font-size="10" font-family="monospace">HDG: {hdg:.0f}°</text>
    </svg>'''
    return svg


# ============================================================
# 暗色科技风CSS主题
# ============================================================

_DARK_THEME_CSS = '''
/* ===== 全局暗色主题 ===== */
[data-testid="stAppViewContainer"] {
    background: #0d1117 !important;
    color: #e6edf3 !important;
}

/* 主内容区背景 */
.stMain [data-testid="stVerticalBlockBorderWrapper"] {
    background: transparent !important;
}

.block-container {
    padding-top: 1rem !important;
    padding-bottom: 2rem !important;
}

/* ===== 侧边栏深蓝渐变 ===== */
[data-testid="stSidebar"] {
    background: linear-gradient(180deg, #0f1b2d 0%, #1a2940 100%) !important;
    border-right: 1px solid rgba(0, 212, 255, 0.15) !important;
}
[data-testid="stSidebar"] * {
    color: #e6edf3 !important;
}
[data-testid="stSidebar"] .stRadio label {
    color: #c9d1d9 !important;
}
[data-testid="stSidebar"] .stRadio label:hover {
    color: #00d4ff !important;
}
[data-testid="stSidebar"] .stRadio [aria-checked="true"] label {
    color: #00d4ff !important;
    font-weight: bold;
}
[data-testid="stSidebar"] .stCaption,
[data-testid="stSidebar"] p {
    color: #8b949e !important;
}
[data-testid="stSidebar"] hr {
    border-color: rgba(0, 212, 255, 0.2) !important;
}

/* ===== 标题渐变色 ===== */
.gradient-title {
    background: linear-gradient(90deg, #00d4ff, #7c3aed);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
    font-size: 1.6rem;
    font-weight: 700;
    padding: 12px 20px;
    border-radius: 10px;
}

/* ===== 通用卡片容器（玻璃态） ===== */
.uav-card {
    background: rgba(22, 33, 62, 0.75);
    border: 1px solid rgba(0, 212, 255, 0.2);
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 16px;
    box-shadow: 0 4px 24px rgba(0, 0, 0, 0.4),
                inset 0 1px 0 rgba(255, 255, 255, 0.05);
    backdrop-filter: blur(10px);
    transition: border-color 0.3s ease;
}
.uav-card:hover {
    border-color: rgba(0, 212, 255, 0.45);
}

/* 带渐变边框的卡片 */
.uav-card-bordered {
    background: rgba(22, 33, 62, 0.8);
    border: 1px solid transparent;
    border-image: linear-gradient(135deg, #00d4ff, #7c3aed) 1;
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 16px;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
}
/* border-image 不支持圆角，用 outline 替代 */
.uav-card-gradient {
    background: rgba(22, 33, 62, 0.8);
    border: 1px solid rgba(0, 212, 255, 0.25);
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 16px;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
    outline: 2px solid transparent;
    outline-offset: -1px;
    background-clip: padding-box;
    position: relative;
}

/* ===== Metric 卡片玻璃态 ===== */
[data-testid="stMetric"] {
    background: rgba(22, 33, 62, 0.6) !important;
    border: 1px solid rgba(0, 212, 255, 0.15) !important;
    border-radius: 10px !important;
    padding: 12px 16px !important;
    box-shadow: 0 2px 12px rgba(0, 0, 0, 0.3),
                inset 0 1px 0 rgba(255, 255, 255, 0.05) !important;
    backdrop-filter: blur(8px);
}
[data-testid="stMetric"] label {
    color: #8b949e !important;
    font-size: 0.8rem !important;
}
[data-testid="stMetric"] [data-testid="stMetricValue"] {
    color: #00d4ff !important;
}

/* ===== 按钮样式 ===== */
.stButton > button {
    background: linear-gradient(135deg, #00d4ff 0%, #7c3aed 100%) !important;
    color: white !important;
    border: none !important;
    border-radius: 8px !important;
    font-weight: 600 !important;
    box-shadow: 0 4px 15px rgba(0, 212, 255, 0.3) !important;
    transition: all 0.3s ease !important;
}
.stButton > button:hover {
    box-shadow: 0 6px 25px rgba(0, 212, 255, 0.5) !important;
    transform: translateY(-1px);
}
.stButton > button:active {
    transform: translateY(0px);
}
/* 次要按钮 */
.stButton > button:not([class*="primary"]) {
    background: linear-gradient(135deg, #1a2940 0%, #0f1b2d 100%) !important;
    border: 1px solid rgba(0, 212, 255, 0.3) !important;
}
/* 停止按钮特殊样式（红色） */
.stButton > button[kind="stop"],
.stButton > button[value="⏹ 停止监控"] {
    background: linear-gradient(135deg, #ef4444 0%, #b91c1c 100%) !important;
}

/* ===== 数据表格深色主题 ===== */
[data-testid="stDataFrame"] {
    background: rgba(22, 33, 62, 0.6) !important;
    border: 1px solid rgba(0, 212, 255, 0.15) !important;
    border-radius: 10px !important;
}
[data-testid="stDataFrame"] table {
    background: transparent !important;
    color: #e6edf3 !important;
}
[data-testid="stDataFrame"] th {
    background: rgba(0, 212, 255, 0.1) !important;
    color: #00d4ff !important;
    border-bottom: 1px solid rgba(0, 212, 255, 0.2) !important;
}
[data-testid="stDataFrame"] td {
    color: #c9d1d9 !important;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05) !important;
}

/* ===== Tabs 深色主题 ===== */
.stTabs [data-baseweb="tab-list"] {
    gap: 4px;
    background: rgba(22, 33, 62, 0.4);
    border-radius: 10px;
    padding: 4px;
}
.stTabs [data-baseweb="tab"] {
    border-radius: 8px !important;
    color: #8b949e !important;
}
.stTabs [data-baseweb="tab-highlight"] {
    background-color: #00d4ff !important;
    border-radius: 8px !important;
}
.stTabs [aria-selected="true"] {
    color: white !important;
    font-weight: 600 !important;
}

/* ===== Selectbox / Checkbox 深色 ===== */
.stSelectbox, .stCheckbox {
    color: #e6edf3 !important;
}

/* ===== Expander ===== */
.streamlit-expanderHeader {
    background: rgba(22, 33, 62, 0.5) !important;
    color: #00d4ff !important;
    border: 1px solid rgba(0, 212, 255, 0.15) !important;
    border-radius: 8px !important;
}

/* ===== 渐变分隔线 ===== */
.uav-divider {
    height: 2px;
    background: linear-gradient(90deg, transparent, #00d4ff, #7c3aed, transparent);
    border: none;
    margin: 20px 0;
    border-radius: 1px;
}

/* ===== section subheader 图标样式 ===== */
.uav-subheader {
    color: #00d4ff;
    font-size: 1.05rem;
    font-weight: 600;
    padding: 8px 0 4px 0;
}

/* ===== 进度条 ===== */
.stProgress > div > div > div {
    background: linear-gradient(90deg, #00d4ff, #7c3aed) !important;
}

/* ===== Form 输入框 ===== */
.stTextInput, .stNumberInput, .stTextArea, .stSelectbox {
    background: rgba(22, 33, 62, 0.6) !important;
}

/* ===== Code block ===== */
.stCodeBlock {
    background: rgba(13, 17, 23, 0.9) !important;
    border: 1px solid rgba(0, 212, 255, 0.15) !important;
    border-radius: 8px !important;
}

/* ===== Success/Warning/Error 消息 ===== */
.stSuccess, [data-testid="stAlert"][data-baseweb="notification"][kind="success"] {
    background: rgba(16, 185, 129, 0.15) !important;
    border-left: 3px solid #10b981 !important;
    color: #10b981 !important;
}
.stWarning, [data-testid="stAlert"][data-baseweb="notification"][kind="warning"] {
    background: rgba(245, 158, 11, 0.15) !important;
    border-left: 3px solid #f59e0b !important;
    color: #f59e0b !important;
}
.stError, [data-testid="stAlert"][data-baseweb="notification"][kind="error"] {
    background: rgba(239, 68, 68, 0.15) !important;
    border-left: 3px solid #ef4444 !important;
    color: #ef4444 !important;
}
.stInfo, [data-testid="stAlert"][data-baseweb="notification"][kind="info"] {
    background: rgba(0, 212, 255, 0.1) !important;
    border-left: 3px solid #00d4ff !important;
    color: #00d4ff !important;
}

/* ===== 侧边栏状态指示器 ===== */
.sidebar-status-dot {
    display: inline-block;
    width: 8px;
    height: 8px;
    border-radius: 50%;
    margin-right: 6px;
    animation: pulse-dot 2s infinite;
}
.sidebar-status-dot.online { background: #10b981; box-shadow: 0 0 6px #10b981; }
.sidebar-status-dot.offline { background: #ef4444; box-shadow: 0 0 6px #ef4444; }

@keyframes pulse-dot {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.4; }
}

/* ===== 侧边栏导航项图标间距 ===== */
.nav-item-icon {
    margin-right: 8px;
    font-size: 1.1em;
}

/* ===== Sidebar 顶部 Logo 区域 ===== */
.sidebar-logo {
    text-align: center;
    padding: 16px 0 12px 0;
}
.sidebar-logo svg {
    filter: drop-shadow(0 0 8px rgba(0, 212, 255, 0.4));
}
.sidebar-title {
    background: linear-gradient(90deg, #00d4ff, #7c3aed);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
    font-size: 1.15rem;
    font-weight: 700;
    letter-spacing: 1px;
}

/* ===== Tooltip / Popover ===== */
[data-testid="stPopover"] {
    background: #1a2940 !important;
}
'''


# ============================================================
# 通用页面标题渲染
# ============================================================

def _render_page_title(title_text):
    """渲染渐变色页面标题"""
    st.markdown(f'''
    <div class="gradient-title">{title_text}</div>
    ''', unsafe_allow_html=True)
    st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)


def _render_subheader(icon, text):
    """渲染带图标的section subheader"""
    st.markdown(f'''
    <div class="uav-subheader">{icon} {text}</div>
    ''', unsafe_allow_html=True)


# 页面配置(必须在第一个st命令之前)
st.set_page_config(
    page_title="无人机飞行规划与监控系统",
    page_icon="🛩️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# 注入全局暗色主题CSS
st.markdown(f'''<style>{_DARK_THEME_CSS}</style>''', unsafe_allow_html=True)


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
        "point_b": [118.7650, 32.2430],
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
    """升级版侧边栏：SVG Logo + 渐变标题 + 图标导航 + 状态指示器"""
    # 顶部Logo区域
    st.sidebar.markdown('''
    <div class="sidebar-logo">
        <svg viewBox="0 0 120 80" width="80" height="55" xmlns="http://www.w3.org/2000/svg">
            <!-- 无人机机身 -->
            <ellipse cx="60" cy="45" rx="18" ry="8" fill="#00d4ff" opacity="0.8"/>
            <rect x="55" y="32" width="10" height="26" rx="3" fill="#00d4ff" opacity="0.6"/>
            <!-- 机臂 -->
            <line x1="30" y1="40" x2="42" y2="44" stroke="#7c3aed" stroke-width="3" stroke-linecap="round"/>
            <line x1="78" y1="44" x2="90" y2="40" stroke="#7c3aed" stroke-width="3" stroke-linecap="round"/>
            <line x1="42" y1="48" x2="35" y2="55" stroke="#7c3aed" stroke-width="3" stroke-linecap="round"/>
            <line x1="78" y1="48" x2="85" y2="55" stroke="#7c3aed" stroke-width="3" stroke-linecap="round"/>
            <!-- 电机 -->
            <circle cx="28" cy="38" r="5" fill="none" stroke="#00d4ff" stroke-width="1.5"/>
            <circle cx="92" cy="38" r="5" fill="none" stroke="#00d4ff" stroke-width="1.5"/>
            <circle cx="33" cy="56" r="5" fill="none" stroke="#00d4ff" stroke-width="1.5"/>
            <circle cx="87" cy="56" r="5" fill="none" stroke="#00d4ff" stroke-width="1.5"/>
            <!-- 螺旋桨 -->
            <ellipse cx="28" cy="38" rx="12" ry="3" fill="rgba(0,212,255,0.25)" stroke="rgba(0,212,255,0.5)" stroke-width="0.5"/>
            <ellipse cx="92" cy="38" rx="12" ry="3" fill="rgba(0,212,255,0.25)" stroke="rgba(0,212,255,0.5)" stroke-width="0.5"/>
            <ellipse cx="33" cy="56" rx="12" ry="3" fill="rgba(0,212,255,0.25)" stroke="rgba(0,212,255,0.5)" stroke-width="0.5"/>
            <ellipse cx="87" cy="56" rx="12" ry="3" fill="rgba(0,212,255,0.25)" stroke="rgba(0,212,255,0.5)" stroke-width="0.5"/>
            <!-- LED灯 -->
            <circle cx="20" cy="36" r="2" fill="#10b981"/>
            <circle cx="100" cy="36" r="2" fill="#ef4444"/>
            <circle cx="25" cy="54" r="2" fill="#f59e0b"/>
            <circle cx="95" cy="54" r="2" fill="#f59e0b"/>
            <!-- 天线 -->
            <line x1="60" y1="32" x2="60" y2="20" stroke="#a0a0a0" stroke-width="1"/>
            <circle cx="60" cy="18" r="2" fill="#00d4ff" opacity="0.7"/>
        </svg>
        <div class="sidebar-title">UAV Flight System</div>
    </div>
    ''', unsafe_allow_html=True)

    st.sidebar.markdown('<hr style="border-color: rgba(0,212,255,0.2);">', unsafe_allow_html=True)

    # 导航菜单（带图标前缀）
    pages = {
        "map": "🗺️  3.1 地图定位模块",
        "obstacle": "🚧  3.2 障碍物与航线规划",
        "monitor": "📡  3.3 飞行监控模块",
        "comm": "🔗  3.4 通信链路展示",
    }
    selected = st.sidebar.radio(
        "功能模块",
        list(pages.keys()),
        format_func=lambda x: pages[x],
        index=list(pages.keys()).index(st.session_state.get("page", "map")),
    )
    st.session_state.page = selected

    st.sidebar.markdown('<hr style="border-color: rgba(0,212,255,0.2);">', unsafe_allow_html=True)

    # 底部系统状态指示器
    is_running = st.session_state.get("monitor_running", False)
    status_class = "online" if is_running else "offline"
    status_text = "系统运行中" if is_running else "系统待机"
    st.sidebar.markdown(f'''
    <div style="padding: 8px 4px; font-size: 0.85rem;">
        <span class="sidebar-status-dot {status_class}"></span>
        <span style="color: #8b949e;">{status_text}</span>
    </div>
    ''', unsafe_allow_html=True)

    st.sidebar.caption("南京科技职业学院\n无人机飞行规划与监控系统 v1.0")


# ============================================================
# 3.1 地图定位模块
# ============================================================
def page_map():
    _render_page_title("3.1 地图定位模块")
    st.markdown("基于 OpenStreetMap 的校园地图显示，支持 WGS-84/GCJ-02 坐标系转换")

    col1, col2 = st.columns([2, 1])

    with col1:
        use_gcj02 = st.checkbox("使用 GCJ-02 坐标系（火星坐标系）", value=False)
        layer_type = st.selectbox(
            "地图图层",
            ["标准地图", "卫星地图", "地形图"],
            format_func=lambda x: {"标准地图": "🗺️ 标准地图 (OpenStreetMap)", "卫星地图": "🛰️ 卫星地图 (Esri)", "地形图": "⛰️ 地形图 (OpenTopoMap)"}[x],
            key="map_layer_sel",
        )
        center = NJVT_CENTER_GCJ02 if use_gcj02 else NJVT_CENTER_WGS84

        m = MapUtils.create_base_map(center=center, layer_type=layer_type)
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
        # 坐标转换工具 - 卡片容器包裹
        st.markdown('''
        <div class="uav-card">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 10px;">
                🔄 坐标转换工具
            </div>
        ''', unsafe_allow_html=True)
        st.markdown("**WGS-84 -> GCJ-02**")
        c1 = st.columns(2)
        wgs_lng = c1[0].number_input("经度", value=118.7620, key="wgs_lng", format="%.6f")
        wgs_lat = c1[1].number_input("纬度", value=32.2450, key="wgs_lat", format="%.6f")
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        st.success(f"GCJ-02: ({gcj_lng:.6f}, {gcj_lat:.6f})")

        st.markdown("**GCJ-02 -> WGS-84**")
        c2 = st.columns(2)
        gi_lng = c2[0].number_input("经度", value=round(gcj_lng, 6), key="gcj_lng_in", format="%.6f")
        gi_lat = c2[1].number_input("纬度", value=round(gcj_lat, 6), key="gcj_lat_in", format="%.6f")
        rl, ra = gcj02_to_wgs84(gi_lng, gi_lat)
        st.success(f"WGS-84: ({rl:.6f}, {ra:.6f})")
        st.markdown('</div>', unsafe_allow_html=True)

        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)

        # 坐标参考点 - 带图标卡片
        st.markdown('''
        <div class="uav-card">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 10px;">
                📍 坐标参考点
            </div>
        ''', unsafe_allow_html=True)
        st.code(f"南京科技职业学院(校园中心)\nWGS-84: ({NJVT_CENTER_WGS84[0]}, {NJVT_CENTER_WGS84[1]})\nGCJ-02: ({NJVT_CENTER_GCJ02[0]:.6f}, {NJVT_CENTER_GCJ02[1]:.6f})")
        st.markdown('</div>', unsafe_allow_html=True)

        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)

        # 设置航点坐标
        st.markdown('''
        <div class="uav-card">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 10px;">
                🎯 设置航点坐标
            </div>
        ''', unsafe_allow_html=True)
        c3 = st.columns(2)
        a_lng = c3[0].number_input("A点经度", value=118.7600, key="a_lng", format="%.6f")
        a_lat = c3[0].number_input("A点纬度", value=32.2470, key="a_lat", format="%.6f")
        b_lng = c3[1].number_input("B点经度", value=118.7650, key="b_lng", format="%.6f")
        b_lat = c3[1].number_input("B点纬度", value=32.2430, key="b_lat", format="%.6f")
        if st.button("确认航点", use_container_width=True):
            st.session_state.point_a = [a_lng, a_lat]
            st.session_state.point_b = [b_lng, b_lat]
            st.success("航点已设置!")
            st.rerun()
        st.markdown('</div>', unsafe_allow_html=True)


# ============================================================
# 3.2 障碍物与航线规划模块
# ============================================================
def page_obstacle():
    _render_page_title("3.2 障碍物与航线规划模块")
    obs_mgr = st.session_state.obstacle_mgr

    tab1, tab2, tab3 = st.tabs(["障碍物管理", "航线规划", "JSON数据"])

    with tab1:
        col_left, col_right = st.columns([2, 1])

        with col_left:
            _render_subheader("🚧", "3.2.1 多边形圈选障碍物")
            m = MapUtils.create_base_map()
            if obs_mgr.obstacles:
                MapUtils.add_obstacle_polygons(m, obs_mgr.obstacles)
            render_map(m, key="obstacle_draw_map", height=500)

            st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
            _render_subheader("➕", "手动添加障碍物")
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
            _render_subheader("📋", "已标记障碍物")
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
                st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
                if st.button("清空所有", use_container_width=True):
                    obs_mgr.obstacles.clear()
                    obs_mgr.save_to_file()
                    st.rerun()
            else:
                st.info("暂无障碍物，请手动添加")

    with tab2:
        _render_subheader("⚙️", "3.2.3 飞行参数设置")
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
            _render_subheader("📊", "3.2.4 航线规划结果对比")
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
                st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
                cm, ci = st.columns([2, 1])
                with cm:
                    mp = MapUtils.create_base_map()
                    MapUtils.add_obstacle_polygons(mp, obs_mgr.obstacles)
                    MapUtils.add_safety_buffer(mp, obs_mgr.obstacles, st.session_state.safety_radius)
                    MapUtils.add_flight_path(mp, sel, color=colors[results.index(sel) % len(colors)])
                    render_map(mp, key="plan_disp", height=500)
                with ci:
                    _render_subheader("📋", f"方案: {sel['name']}")
                    st.write(f"距离: **{sel['distance']:.1f}m**")
                    st.write(f"航点: **{len(sel['waypoints'])}**")
                    st.write(f"安全: {'✅' if sel['clears_obstacles'] else '❌'}")
                    for i, wp in enumerate(sel["waypoints"]):
                        st.code(f"WP{i}: ({wp[0]:.6f}, {wp[1]:.6f})")

    with tab3:
        _render_subheader("📄", "3.2.2 障碍物 JSON 数据")
        js = obs_mgr.to_json_string()
        st.code(js, language="json")
        st.download_button("下载 JSON", js.encode("utf-8"), "obstacles_data.json", "application/json")

        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
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
    _render_page_title("3.3 飞行监控模块")
    sim = st.session_state.mavlink_sim

    # 控制栏 - 带渐变边框的卡片
    st.markdown('''
    <div class="uav-card" style="border-image: linear-gradient(135deg, #00d4ff, #7c3aed) 1;">
    ''', unsafe_allow_html=True)
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
    st.markdown('</div>', unsafe_allow_html=True)

    if st.session_state.monitor_running:
        sim.generate_all()

    # 仪表盘 - 6个metric用3x2网格布局在卡片内
    st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
    state = sim.state

    st.markdown('''
    <div class="uav-card">
        <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 12px;">
            📊 核心飞行参数
        </div>
    ''', unsafe_allow_html=True)
    mc = st.columns(6)
    for col, (label, val) in zip(mc, [
        ("纬度", f"{state['lat']:.6f}°"), ("经度", f"{state['lng']:.6f}°"),
        ("高度", f"{state['alt']:.1f}m"), ("地速", f"{state['ground_speed']:.1f}m/s"),
        ("航向", f"{state['heading']:.0f}°"), ("电池", f"{state['battery_remaining']:.0f}%"),
    ]):
        col.metric(label, val)
    st.markdown('</div>', unsafe_allow_html=True)

    # SVG仪表盘区域：姿态指示器 / 电池状态 / 航向罗盘
    d1, d2, d3 = st.columns(3)

    with d1:
        st.markdown('''
        <div class="uav-card" style="text-align: center;">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 8px;">
                🧭 姿态指示器
            </div>
        ''', unsafe_allow_html=True)
        # 渲染SVG姿态指示器
        ai_svg = _attitude_indicator_svg(state["pitch"], state["roll"])
        st.components.v1.html(ai_svg, height=230)
        st.markdown(f"Pitch: **{state['pitch']:.2f}°**  |  Roll: **{state['roll']:.2f}°**  |  Yaw: **{state['yaw']:.2f}°**")
        st.markdown('</div>', unsafe_allow_html=True)

    with d2:
        st.markdown('''
        <div class="uav-card" style="text-align: center;">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 8px;">
                🔋 电池状态
            </div>
        ''', unsafe_allow_html=True)
        # 渲染SVG电池状态条
        bat_svg = _battery_svg(state["battery_remaining"], state["battery_voltage"])
        st.components.v1.html(bat_svg, height=70)
        bc = "🟢" if state["battery_remaining"] > 30 else ("🟡" if state["battery_remaining"] > 15 else "🔴")
        st.markdown(f"剩余: **{state['battery_remaining']:.0f}%** {bc}")
        st.markdown(f"油门: **{state['throttle']}%**")
        st.markdown(f"模式: **{state['flight_mode']}**")
        st.markdown('</div>', unsafe_allow_html=True)

    with d3:
        st.markdown('''
        <div class="uav-card" style="text-align: center;">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 8px;">
                🧭 航向罗盘
            </div>
        ''', unsafe_allow_html=True)
        # 渲染SVG航向罗盘
        hdg_svg = _heading_svg(state["heading"])
        st.components.v1.html(hdg_svg, height=210)
        st.markdown('</div>', unsafe_allow_html=True)

    # 导航信息面板
    st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
    st.markdown('''
    <div class="uav-card">
        <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 8px;">
            📡 导航状态
        </div>
    ''', unsafe_allow_html=True)
    nav_cols = st.columns(3)
    nav_cols[0].metric("GPS状态", "3D Fix" if state['gps_fix'] == 3 else "No Fix")
    nav_cols[1].metric("卫星数", state['gps_satellites'])
    nav_cols[2].metric("垂直速度", f"{state['climb_rate']:.2f}m/s")
    st.markdown('</div>', unsafe_allow_html=True)

    # 监控地图
    st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
    _render_subheader("🗺️", "实时位置地图")
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
    st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
    _render_subheader("📝", "MAVLink 报文日志")
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
    _render_page_title("3.4 通信链路展示模块")
    comm = st.session_state.comm_topology

    tab1, tab2 = st.tabs(["3.4.1 通信拓扑", "3.4.2 MAVLink数据流"])

    with tab1:
        _render_subheader("🌐", "GCS - OBC - FCU 通信拓扑结构")
        # 拓扑图区域添加渐变边框装饰
        st.markdown('''
        <div class="uav-card" style="padding: 10px; border-image: linear-gradient(135deg, #00d4ff, #7c3aed) 1;">
        ''', unsafe_allow_html=True)
        st.components.v1.html(comm.generate_topology_html(), height=530)
        st.markdown('</div>', unsafe_allow_html=True)

        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)

        # 节点状态统计 - 卡片包裹
        ss = comm.get_node_status_summary()
        st.markdown('''
        <div class="uav-card">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 12px;">
                📊 节点状态总览
            </div>
        ''', unsafe_allow_html=True)
        s1, s2, s3 = st.columns(3)
        s1.metric("总节点", ss["total"])
        s2.metric("在线", ss["online"])
        s3.metric("离线", ss["offline"])
        st.markdown('</div>', unsafe_allow_html=True)

        for node in comm.nodes:
            icon = "🟢" if node.status == "online" else "🔴"
            with st.expander(f"{icon} {node.name} [{node.node_type}]"):
                ci = st.columns(3)
                ci[0].write(f"IP: `{node.ip}`")
                ci[1].write(f"端口: `{node.port}`")
                ci[2].write(f"状态: `{node.status}`")

        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        _render_subheader("📡", "链路信息")
        # 链路信息用卡片包裹
        st.markdown('''
        <div class="uav-card">
        ''', unsafe_allow_html=True)
        lc = st.columns(min(len(comm.links), 4))
        for i, link in enumerate(comm.links):
            with lc[i % len(lc)]:
                lat = link.latency_ms
                lc_icon = "🟢" if lat < 10 else ("🟡" if lat < 20 else "🔴")
                st.markdown(f"**{link.source[:4]} -> {link.target[:4]}**\n"
                            f"协议: `{link.protocol}`\n"
                            f"延迟: {lc_icon} `{lat:.1f}ms`")
        st.markdown('</div>', unsafe_allow_html=True)

    with tab2:
        _render_subheader("📡", "3.4.2 MAVLink 数据流与报文显示")

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
        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        st.markdown('''
        <div class="uav-card">
            <div style="color: #00d4ff; font-weight: 600; font-size: 0.95rem; margin-bottom: 12px;">
                📈 数据流统计
            </div>
        ''', unsafe_allow_html=True)
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
        st.markdown('</div>', unsafe_allow_html=True)

        # ---- 数据流方向可视化 ----
        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        _render_subheader("🔀", "数据流方向示意")
        st.markdown('''
        <div class="uav-card">
        ''', unsafe_allow_html=True)
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
        st.markdown('</div>', unsafe_allow_html=True)

        # ---- 数据流时序图(文本形式) ----
        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        _render_subheader("⏱️", "数据流时序图")
        st.markdown("展示最近报文在各节点间的传递顺序")

        recent_all = comm.get_recent_messages(15)
        if recent_all:
            timeline_html = '''
            <div style="font-family: monospace; font-size: 11px; overflow-x: auto; padding: 10px;
                        background: #1e1e1e; color: #d4d4d4; border-radius: 8px; line-height: 1.6;
                        border: 1px solid rgba(0,212,255,0.15);">
            <div style="display: flex; gap: 40px; margin-bottom: 8px; color: #888;">
                <span style="width:80px;">时间</span>
                <span style="width:60px;">方向</span>
                <span style="width:120px;">消息类型</span>
                <span style="width:200px;">关键字段</span>
            </div>
            '''
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
        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        _render_subheader("📤", "手动发送 MAVLink 报文")
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
        st.markdown('<hr class="uav-divider">', unsafe_allow_html=True)
        _render_subheader("📋", "报文历史记录")
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
