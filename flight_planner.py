"""
航线规划模块
支持直线飞越、左绕飞、右绕飞和最优路径(最短弧线)规划
包含安全半径计算,确保飞行路径与障碍物保持安全距离
"""

import math
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union


class FlightPlanner:
    """航线规划器"""

    def __init__(self):
        self.obstacles = []  # Obstacle对象列表
        self.flight_height = 50  # 默认飞行高度(米)
        self.safety_radius = 10  # 默认安全半径(米)

    def set_parameters(self, flight_height, safety_radius):
        self.flight_height = flight_height
        self.safety_radius = safety_radius

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def _lnglat_to_meters(self, lng, lat, ref_lng, ref_lat):
        """经纬度差转近似米数(适用于小范围)"""
        dx = (lng - ref_lng) * 111320 * math.cos(ref_lat * math.pi / 180)
        dy = (lat - ref_lat) * 110540
        return dx, dy

    def _meters_to_lnglat(self, dx, dy, ref_lng, ref_lat):
        """米数转经纬度差(近似)"""
        lng = ref_lng + dx / (111320 * math.cos(ref_lat * math.pi / 180))
        lat = ref_lat + dy / 110540
        return lng, lat

    def _point_to_segment_distance(self, px, py, ax, ay, bx, by):
        """点到线段的距离"""
        abx, aby = bx - ax, by - ay
        apx, apy = px - ax, py - ay
        ab_sq = abx * abx + aby * aby
        if ab_sq == 0:
            return math.hypot(apx, apy)
        t = max(0, min(1, (apx * abx + apy * aby) / ab_sq))
        proj_x = ax + t * abx
        proj_y = ay + t * aby
        return math.hypot(px - proj_x, py - proj_y)

    def _build_obstacle_polygons(self):
        """构建shapely多边形(含安全缓冲区)"""
        polygons = []
        for obs in self.obstacles:
            coords = obs.coordinates
            if len(coords) < 3:
                continue
            # 将经纬度转为米数坐标系(以第一个点为参考)
            ref_lng, ref_lat = coords[0]
            meter_coords = []
            for lng, lat in coords:
                mx, my = self._lnglat_to_meters(lng, lat, ref_lng, ref_lat)
                meter_coords.append((mx, my))
            poly = Polygon(meter_coords)
            if not poly.is_valid:
                poly = poly.buffer(0)
            # 添加安全半径缓冲区
            buffered = poly.buffer(self.safety_radius)
            polygons.append(buffered)
        return polygons, ref_lng, ref_lat

    def plan_straight(self, start, end):
        """直线飞越路径规划"""
        path = [start, end]
        total_dist = self._calc_distance(start, end)
        return {
            "name": "直线飞越",
            "path": path,
            "distance": total_dist,
            "waypoints": [start, end],
            "clears_obstacles": self._check_clearance(path),
        }

    def plan_avoid_left(self, start, end):
        """向左绕飞路径规划"""
        return self._plan_avoid(start, end, direction="left")

    def plan_avoid_right(self, start, end):
        """向右绕飞路径规划"""
        return self._plan_avoid(start, end, direction="right")

    def plan_optimal(self, start, end):
        """最优路径(弧线)规划"""
        if not self.obstacles:
            return self.plan_straight(start, end)

        polygons, ref_lng, ref_lat = self._build_obstacle_polygons()
        if not polygons:
            return self.plan_straight(start, end)

        merged = unary_union(polygons)

        # 将起终点转为米坐标
        sx, sy = self._lnglat_to_meters(start[0], start[1], ref_lng, ref_lat)
        ex, ey = self._lnglat_to_meters(end[0], end[1], ref_lng, ref_lat)

        start_pt = Point(sx, sy)
        end_pt = Point(ex, ey)

        # 检查直线是否通过障碍物
        direct = LineString([(sx, sy), (ex, ey)])
        if not direct.intersects(merged):
            return self.plan_straight(start, end)

        # 可见性图法 + Dijkstra寻找最短路径
        waypoints_m = self._visibility_graph_path(start_pt, end_pt, merged)
        waypoints = []
        for wx, wy in waypoints_m:
            lng, lat = self._meters_to_lnglat(wx, wy, ref_lng, ref_lat)
            waypoints.append((lng, lat))

        total_dist = sum(
            self._calc_distance(waypoints[i], waypoints[i + 1])
            for i in range(len(waypoints) - 1)
        )

        return {
            "name": "最优路径(最短弧线)",
            "path": waypoints,
            "distance": total_dist,
            "waypoints": waypoints,
            "clears_obstacles": self._check_clearance(waypoints),
        }

    def _plan_avoid(self, start, end, direction="left"):
        """绕飞路径规划核心"""
        if not self.obstacles:
            return self.plan_straight(start, end)

        polygons, ref_lng, ref_lat = self._build_obstacle_polygons()
        if not polygons:
            return self.plan_straight(start, end)

        merged = unary_union(polygons)

        sx, sy = self._lnglat_to_meters(start[0], start[1], ref_lng, ref_lat)
        ex, ey = self._lnglat_to_meters(end[0], end[1], ref_lng, ref_lat)

        direct = LineString([(sx, sy), (ex, ey)])
        if not direct.intersects(merged):
            return self.plan_straight(start, end)

        # 确定绕行方向: 左绕=逆时针偏移, 右绕=顺时针偏移
        sign = 1 if direction == "left" else -1
        avoidance_dist = self.safety_radius + 15  # 额外15米绕飞间距

        waypoints_m = [(sx, sy)]

        # 对每个相交的障碍物生成绕行点
        path_line = LineString([(sx, sy), (ex, ey)])
        intersecting = []
        for poly in polygons:
            if path_line.intersects(poly):
                intersecting.append(poly)

        if not intersecting:
            return self.plan_straight(start, end)

        sorted_polys = sorted(
            intersecting,
            key=lambda p: p.centroid.distance(Point(sx, sy))
        )

        current = Point(sx, sy)
        for poly in sorted_polys:
            boundary = poly.exterior
            coords = list(boundary.coords)

            # 计算路径前进方向的法向量
            dx = ex - current.x
            dy = ey - current.y
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            nx = -dy / length * sign  # 法向量
            ny = dx / length * sign

            # 在法向量方向上选择绕行点
            tangent_angle = math.atan2(ny, nx)
            # 添加弧线绕行点
            n_arc = 8
            arc_radius = avoidance_dist

            # 找到多边形边界上最近的点
            nearest_pt = poly.boundary.interpolate(poly.boundary.project(current))
            arc_center_x = nearest_pt.x + nx * arc_radius
            arc_center_y = nearest_pt.y + ny * arc_radius

            for i in range(1, n_arc + 1):
                angle = tangent_angle + (math.pi * i / (n_arc + 1)) * (-sign)
                ax = arc_center_x + avoidance_dist * math.cos(angle)
                ay = arc_center_y + avoidance_dist * math.sin(angle)
                if not merged.contains(Point(ax, ay)):
                    waypoints_m.append((ax, ay))
                else:
                    # 如果点在障碍物内,增大偏移距离
                    ax = arc_center_x + (avoidance_dist + 10) * math.cos(angle)
                    ay = arc_center_y + (avoidance_dist + 10) * math.sin(angle)
                    if not merged.contains(Point(ax, ay)):
                        waypoints_m.append((ax, ay))

        waypoints_m.append((ex, ey))

        waypoints = []
        for wx, wy in waypoints_m:
            lng, lat = self._meters_to_lnglat(wx, wy, ref_lng, ref_lat)
            waypoints.append((lng, lat))

        total_dist = sum(
            self._calc_distance(waypoints[i], waypoints[i + 1])
            for i in range(len(waypoints) - 1)
        )

        label = "左绕飞" if direction == "left" else "右绕飞"
        return {
            "name": label,
            "path": waypoints,
            "distance": total_dist,
            "waypoints": waypoints,
            "clears_obstacles": self._check_clearance(waypoints),
        }

    def _visibility_graph_path(self, start, end, obstacle_union):
        """可见性图 + Dijkstra 最短路径"""
        nodes = [start, end]

        # 从障碍物多边形顶点取样
        if hasattr(obstacle_union, "geoms"):
            for geom in obstacle_union.geoms:
                if geom.geom_type == "Polygon" and geom.exterior:
                    for coord in list(geom.exterior.coords)[::3]:
                        nodes.append(Point(coord))
        elif obstacle_union.exterior:
            for coord in list(obstacle_union.exterior.coords)[::3]:
                nodes.append(Point(coord))

        n = len(nodes)
        dist_matrix = np.full((n, n), np.inf)

        for i in range(n):
            for j in range(i + 1, n):
                line = LineString([(nodes[i].x, nodes[i].y),
                                   (nodes[j].x, nodes[j].y)])
                if not obstacle_union.crosses(line) and not obstacle_union.contains(line):
                    d = nodes[i].distance(nodes[j])
                    dist_matrix[i][j] = d
                    dist_matrix[j][i] = d

        # Dijkstra
        visited = [False] * n
        dists = [np.inf] * n
        prev = [-1] * n
        dists[0] = 0

        for _ in range(n):
            u = min((d for i, d in enumerate(dists) if not visited[i]), default=np.inf)
            if u == np.inf:
                break
            u_idx = dists.index(u)
            visited[u_idx] = True
            for v in range(n):
                if not visited[v] and dist_matrix[u_idx][v] < np.inf:
                    alt = dists[u_idx] + dist_matrix[u_idx][v]
                    if alt < dists[v]:
                        dists[v] = alt
                        prev[v] = u_idx

        # 回溯路径
        path_nodes = []
        cur = 1  # end node
        while cur != -1:
            path_nodes.append((nodes[cur].x, nodes[cur].y))
            cur = prev[cur]
        path_nodes.reverse()

        return path_nodes if path_nodes else [(start.x, start.y), (end.x, end.y)]

    def _calc_distance(self, p1, p2):
        """Haversine公式计算两点间距离(米)"""
        R = 6371000
        lat1, lng1 = math.radians(p1[1]), math.radians(p1[0])
        lat2, lng2 = math.radians(p2[1]), math.radians(p2[0])
        dlat = lat2 - lat1
        dlng = lng2 - lng1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlng / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        return R * c

    def _check_clearance(self, path):
        """检查路径是否满足安全距离"""
        if not self.obstacles or len(path) < 2:
            return True
        polygons, ref_lng, ref_lat = self._build_obstacle_polygons()
        if not polygons:
            return True
        merged = unary_union(polygons)
        for i in range(len(path) - 1):
            sx, sy = self._lnglat_to_meters(path[i][0], path[i][1], ref_lng, ref_lat)
            ex, ey = self._lnglat_to_meters(path[i + 1][0], path[i + 1][1], ref_lng, ref_lat)
            line = LineString([(sx, sy), (ex, ey)])
            if merged.intersects(line):
                return False
        return True

    def plan_all(self, start, end):
        """生成所有规划方案"""
        results = []
        results.append(self.plan_straight(start, end))
        results.append(self.plan_avoid_left(start, end))
        results.append(self.plan_avoid_right(start, end))
        results.append(self.plan_optimal(start, end))

        # 按距离排序
        results.sort(key=lambda r: r["distance"])
        for i, r in enumerate(results):
            r["rank"] = i + 1

        return results
