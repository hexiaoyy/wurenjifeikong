"""
障碍物管理模块
支持多边形圈选障碍物、高度设置、JSON文件保存与加载(记忆功能)
"""

import json
import os
import streamlit as st

OBSTACLE_FILE = "obstacles_data.json"


class Obstacle:
    """障碍物数据类"""

    def __init__(self, name, coordinates, height, color="#FF4444"):
        self.name = name
        self.coordinates = coordinates  # [(lng, lat), ...] 多边形顶点
        self.height = height  # 障碍物高度(米)
        self.color = color

    def to_dict(self):
        return {
            "name": self.name,
            "coordinates": self.coordinates,
            "height": self.height,
            "color": self.color,
        }

    @staticmethod
    def from_dict(data):
        return Obstacle(
            name=data["name"],
            coordinates=[tuple(c) for c in data["coordinates"]],
            height=data["height"],
            color=data.get("color", "#FF4444"),
        )

    @property
    def centroid(self):
        """计算多边形质心"""
        if not self.coordinates:
            return 0, 0
        lngs = [c[0] for c in self.coordinates]
        lats = [c[1] for c in self.coordinates]
        return sum(lngs) / len(lngs), sum(lats) / len(lats)

    @property
    def latlng_list(self):
        """返回适合folium使用的坐标列表"""
        return [list(reversed(c)) for c in self.coordinates]  # (lng,lat) -> [lat,lng]


class ObstacleManager:
    """障碍物管理器"""

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

    def get_obstacle(self, index):
        if 0 <= index < len(self.obstacles):
            return self.obstacles[index]
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
        data = [obs.to_dict() for obs in self.obstacles]
        return json.dumps(data, ensure_ascii=False, indent=2)

    @staticmethod
    def get_or_create(session_key="obstacle_manager"):
        """从session_state获取或创建ObstacleManager(含自动加载)"""
        if session_key not in st.session_state:
            mgr = ObstacleManager()
            mgr.load_from_file()
            st.session_state[session_key] = mgr
        return st.session_state[session_key]
