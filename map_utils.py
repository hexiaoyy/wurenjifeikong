"""
地图工具模块
处理OpenStreetMap显示、图层切换、地图标记等
"""

import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
from .coord_transform import wgs84_to_gcj02, gcj02_to_wgs84

# 南京科技职业学院中心坐标(WGS84)
DEFAULT_CENTER_WGS84 = (118.7993, 32.1080)
DEFAULT_ZOOM = 17


class MapUtils:
    """地图工具类"""

    @staticmethod
    def create_base_map(center=None, zoom=None, use_gcj02=False):
        """创建基础地图"""
        if center is None:
            if use_gcj02:
                center = wgs84_to_gcj02(*DEFAULT_CENTER_WGS84)
            else:
                center = DEFAULT_CENTER_WGS84
        if zoom is None:
            zoom = DEFAULT_ZOOM

        # 坐标转换: folium需要(lat, lng)
        if use_gcj02:
            map_center = [center[1], center[0]]  # gcj02 (lng,lat) -> [lat, lng]
        else:
            map_center = [center[1], center[0]]

        m = folium.Map(
            location=map_center,
            zoom_start=zoom,
            tiles="OpenStreetMap",
            attr="OpenStreetMap contributors",
        )

        # 添加图层控件
        folium.TileLayer(
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri World Imagery",
            name="卫星地图",
            overlay=False,
            control=True,
        ).add_to(m)

        folium.TileLayer(
            tiles="https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
            attr="OpenTopoMap",
            name="地形图",
            overlay=False,
            control=True,
        ).add_to(m)

        folium.LayerControl().add_to(m)

        # 添加绘制工具(多边形圈选障碍物)
        draw = Draw(
            draw_options={
                "polygon": True,
                "polyline": False,
                "rectangle": False,
                "circle": False,
                "circlemarker": False,
                "marker": True,
            },
            edit_options={"edit": True, "remove": True},
        )
        m.add_child(draw)

        return m

    @staticmethod
    def add_obstacle_polygons(m, obstacles, use_gcj02=False):
        """在地图上添加障碍物多边形"""
        colors = ["#FF4444", "#FF8800", "#FFAA00", "#FF6666", "#CC3300"]
        for i, obs in enumerate(obstacles):
            coords = obs.latlng_list
            color = obs.color or colors[i % len(colors)]

            popup_html = f"""
            <div style="font-family: sans-serif; min-width: 150px;">
                <b>{obs.name}</b><br>
                高度: {obs.height}m<br>
                顶点数: {len(obs.coordinates)}
            </div>
            """

            folium.Polygon(
                locations=coords,
                color=color,
                fill=True,
                fill_color=color,
                fill_opacity=0.3,
                weight=2,
                popup=folium.Popup(popup_html, max_width=200),
                tooltip=obs.name,
            ).add_to(m)

            # 在质心添加标签
            centroid = obs.centroid
            label_lat, label_lng = centroid[1], centroid[0]
            folium.Marker(
                location=[label_lat, label_lng],
                icon=folium.DivIcon(
                    html=f'<div style="font-size:10px; color:{color}; font-weight:bold; '
                         f'text-shadow: -1px 0 white, 0 1px white, 1px 0 white, 0 -1px white;">'
                         f'{obs.name} ({obs.height}m)</div>'
                ),
            ).add_to(m)

    @staticmethod
    def add_flight_path(m, plan_result, color="#2196F3", use_gcj02=False):
        """在地图上添加飞行路径"""
        waypoints = plan_result["waypoints"]
        if len(waypoints) < 2:
            return

        # 转换坐标
        coords = [[wp[1], wp[0]] for wp in waypoints]  # (lng,lat) -> [lat,lng]

        # 添加路径线
        folium.PolyLine(
            locations=coords,
            color=color,
            weight=3,
            opacity=0.8,
            popup=f"{plan_result['name']} - 距离: {plan_result['distance']:.1f}m",
        ).add_to(m)

        # 添加航点标记
        for i, wp in enumerate(waypoints):
            icon_color = "green" if i == 0 else ("red" if i == len(waypoints) - 1 else "blue")
            folium.Marker(
                location=[wp[1], wp[0]],
                icon=folium.DivIcon(
                    html=f'<div style="background-color:{icon_color}; color:white; '
                         f'border-radius:50%; width:20px; height:20px; '
                         f'display:flex; align-items:center; justify-content:center; '
                         f'font-size:10px; font-weight:bold;">{i}</div>'
                ),
                tooltip=f"航点 {i}: ({wp[0]:.6f}, {wp[1]:.6f})",
            ).add_to(m)

        # 起终点标记
        folium.Marker(
            location=[waypoints[0][1], waypoints[0][0]],
            popup=f"起点 A ({waypoints[0][0]:.6f}, {waypoints[0][1]:.6f})",
            icon=folium.Icon(color="green", icon="play", prefix="fa"),
        ).add_to(m)

        folium.Marker(
            location=[waypoints[-1][1], waypoints[-1][0]],
            popup=f"终点 B ({waypoints[-1][0]:.6f}, {waypoints[-1][1]:.6f})",
            icon=folium.Icon(color="red", icon="flag-checkered", prefix="fa"),
        ).add_to(m)

    @staticmethod
    def add_safety_buffer(m, obstacles, safety_radius, use_gcj02=False):
        """添加安全半径缓冲区可视化"""
        for obs in obstacles:
            centroid = obs.centroid
            folium.Circle(
                location=[centroid[1], centroid[0]],
                radius=safety_radius,
                color="#FF9800",
                fill=True,
                fill_color="#FF9800",
                fill_opacity=0.1,
                weight=1,
                dash_array="5, 5",
                popup=f"安全半径: {safety_radius}m",
            ).add_to(m)

    @staticmethod
    def render_map(m, key=None, height=600):
        """渲染地图到Streamlit"""
        return st_folium(m, key=key, height=height, use_container_width=True)
