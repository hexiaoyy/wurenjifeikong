"""
UAV Flight Planning & Monitoring System
无人机飞行规划与监控系统 - Streamlit可视化界面

功能模块:
  3.1 地图定位模块 - OpenStreetMap显示、WGS84/GCJ02坐标转换
  3.2 障碍物与航线规划模块 - 多边形圈选、航线规划(飞越/绕飞/最优)
  3.3 飞行监控模块 - MAVLink数据仿真、实时状态监控
  3.4 通信链路展示模块 - GCS-OBC-FCU拓扑图、MAVLink报文显示

运行方式: streamlit run app.py
"""

import sys
from pathlib import Path

# 确保utils包能被找到(兼容Streamlit Cloud等部署环境)
sys.path.insert(0, str(Path(__file__).parent.resolve()))

import streamlit as st
import json
import time
import folium

from utils.coord_transform import wgs84_to_gcj02, gcj02_to_wgs84, NJVT_CENTER_WGS84, NJVT_CENTER_GCJ02
from utils.obstacle_manager import ObstacleManager, Obstacle
from utils.flight_planner import FlightPlanner
from utils.map_utils import MapUtils
from utils.comm_topology import CommTopology
from utils.mavlink_sim import MAVLinkSimulator

# ============================================================
# 页面配置(必须在第一个st命令之前)
# ============================================================
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
        "point_b": [118.8020, 32.1070],
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

        result = MapUtils.render_map(m, key="main_map", height=500)
        if result and result.get("last_clicked"):
            c = result["last_clicked"]
            if c.get("lat") and c.get("lng"):
                st.info(f"点击位置: 纬度={c['lat']:.6f}, 经度={c['lng']:.6f}")

    with col2:
        st.subheader("坐标转换工具")
        st.markdown("**WGS-84 -> GCJ-02**")
        c1 = st.columns(2)
        wgs_lng = c1[0].number_input("经度", value=118.7993, key="wgs_lng", format="%.6f")
        wgs_lat = c1[1].number_input("纬度", value=32.1080, key="wgs_lat", format="%.6f")
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
        a_lng = c3[0].number_input("A点经度", value=118.7970, key="a_lng", format="%.6f")
        a_lat = c3[0].number_input("A点纬度", value=32.1090, key="a_lat", format="%.6f")
        b_lng = c3[1].number_input("B点经度", value=118.8020, key="b_lng", format="%.6f")
        b_lat = c3[1].number_input("B点纬度", value=32.1070, key="b_lat", format="%.6f")
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
            MapUtils.render_map(m, key="obstacle_draw_map", height=500)

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
                    MapUtils.render_map(mp, key="plan_disp", height=500)
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
    MapUtils.render_map(mm, key="mon_map", height=400)

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
        st.subheader("MAVLink 数据流与报文显示")
        h1, h2 = st.columns(2)
        with h1:
            if st.button("发送心跳包", use_container_width=True, type="primary"):
                comm.add_heartbeat()
                st.rerun()
            if st.button("重置计数", use_container_width=True):
                comm.heartbeat_count = 0
                comm.message_log.clear()
                st.rerun()
        with h2:
            st.metric("心跳计数", comm.heartbeat_count)
            if comm.last_heartbeat > 0:
                st.metric("上次心跳距今", f"{time.time() - comm.last_heartbeat:.1f}s")

        if comm.message_log:
            st.markdown("#### 最近心跳包")
            st.json(comm.message_log[-1].data)

        st.markdown("---")
        st.subheader("报文历史")
        recent = comm.get_recent_messages(20)
        if recent:
            rows = []
            for msg in reversed(recent):
                rows.append({
                    "时间": time.strftime("%H:%M:%S", time.localtime(msg.timestamp)),
                    "来源": msg.source, "消息": msg.msg_name,
                    "数据": json.dumps(msg.data, ensure_ascii=False)[:60],
                })
            st.dataframe(rows, use_container_width=True, hide_index=True)
        else:
            st.info("暂无报文记录")


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
