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

import streamlit as st
import json
import time
import folium
from datetime import datetime

from utils.coord_transform import wgs84_to_gcj02, gcj02_to_wgs84, NJVT_CENTER_WGS84, NJVT_CENTER_GCJ02
from utils.obstacle_manager import ObstacleManager
from utils.flight_planner import FlightPlanner
from utils.map_utils import MapUtils
from utils.comm_topology import CommTopology
from utils.mavlink_sim import MAVLinkSimulator

# ============================================================
# 页面配置
# ============================================================
st.set_page_config(
    page_title="无人机飞行规划与监控系统",
    page_icon="🛩️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# 自定义CSS
st.markdown("""
<style>
    .stApp { max-width: 100%; }
    .block-container { padding-top: 1rem; }
    section[data-testid="stSidebar"] .stMarkdown { font-size: 0.9rem; }
    .metric-container {
        background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
        border-radius: 10px;
        padding: 15px;
        margin: 5px 0;
    }
</style>
""", unsafe_allow_html=True)


def init_session_state():
    """初始化session_state"""
    defaults = {
        "page": "map",
        "obstacle_mgr": None,
        "flight_planner": FlightPlanner(),
        "comm_topology": CommTopology(),
        "mavlink_sim": MAVLinkSimulator(),
        "heartbeat_count": 0,
        "monitor_running": False,
        "selected_plan": None,
        "point_a": list(NJVT_CENTER_WGS84),
        "point_b": None,
        "flight_height": 50.0,
        "safety_radius": 10.0,
    }
    for key, val in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = val

    # 障碍物管理器初始化(含文件加载)
    if st.session_state.obstacle_mgr is None:
        mgr = ObstacleManager()
        mgr.load_from_file()
        st.session_state.obstacle_mgr = mgr


def sidebar_navigation():
    """侧边栏导航"""
    st.sidebar.title("导航菜单")
    st.sidebar.markdown("---")

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

    st.sidebar.markdown("---")
    st.sidebar.caption(
        "南京科技职业学院\n"
        "无人机飞行规划与监控系统 v1.0"
    )


# ============================================================
# 3.1 地图定位模块
# ============================================================
def page_map():
    """地图定位模块"""
    st.header("3.1 地图定位模块")
    st.markdown("基于 OpenStreetMap 的校园地图显示，支持 WGS-84/GCJ-02 坐标系转换")

    col1, col2 = st.columns([2, 1])

    with col1:
        # 坐标系选择
        use_gcj02 = st.checkbox("使用 GCJ-02 坐标系（火星坐标系）", value=False)
        center = NJVT_CENTER_GCJ02 if use_gcj02 else NJVT_CENTER_WGS84

        # 创建并显示地图
        m = MapUtils.create_base_map(center=center, use_gcj02=use_gcj02)

        # 在地图上标记校园中心
        center_lat, center_lng = center[1], center[0]
        folium.Marker(
            location=[center_lat, center_lng],
            popup=f"{'GCJ-02' if use_gcj02 else 'WGS-84'} 校园中心\n({center[0]:.6f}, {center[1]:.6f})",
            icon=folium.Icon(color="blue", icon="info-sign"),
        ).add_to(m)

        # 添加当前障碍物
        obs_mgr = st.session_state.obstacle_mgr
        if obs_mgr.obstacles:
            MapUtils.add_obstacle_polygons(m, obs_mgr.obstacles)

        # 添加路径点
        if st.session_state.point_b:
            p_a = st.session_state.point_a
            p_b = st.session_state.point_b
            folium.Marker(
                location=[p_a[1], p_a[0]],
                popup=f"起点A: ({p_a[0]:.6f}, {p_a[1]:.6f})",
                icon=folium.Icon(color="green", icon="play"),
            ).add_to(m)
            folium.Marker(
                location=[p_b[1], p_b[0]],
                popup=f"终点B: ({p_b[0]:.6f}, {p_b[1]:.6f})",
                icon=folium.Icon(color="red", icon="flag-checkered"),
            ).add_to(m)

        result = MapUtils.render_map(m, key="main_map", height=500)

        # 处理地图点击
        if result and result.get("last_clicked"):
            clicked = result["last_clicked"]
            if clicked.get("lat") and clicked.get("lng"):
                st.info(
                    f"📍 点击位置: 纬度={clicked['lat']:.6f}, 经度={clicked['lng']:.6f} "
                    f"({'GCJ-02' if use_gcj02 else 'WGS-84'})"
                )

    with col2:
        st.subheader("坐标转换工具")

        # WGS84转GCJ02
        st.markdown("**WGS-84 → GCJ-02**")
        c1 = st.columns(2)
        wgs_lng = c1[0].number_input("经度", value=118.7993, key="wgs_lng", format="%.6f")
        wgs_lat = c1[1].number_input("纬度", value=32.1080, key="wgs_lat", format="%.6f")

        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        st.success(f"GCJ-02: ({gcj_lng:.6f}, {gcj_lat:.6f})")

        st.markdown("---")
        st.markdown("**GCJ-02 → WGS-84**")
        c2 = st.columns(2)
        gcj_in_lng = c2[0].number_input("经度", value=round(gcj_lng, 6), key="gcj_lng_in", format="%.6f")
        gcj_in_lat = c2[1].number_input("纬度", value=round(gcj_lat, 6), key="gcj_lat_in", format="%.6f")

        rec_lng, rec_lat = gcj02_to_wgs84(gcj_in_lng, gcj_in_lat)
        st.success(f"WGS-84: ({rec_lng:.6f}, {rec_lat:.6f})")

        st.markdown("---")
        st.subheader("坐标参考点")

        st.markdown(f"""
        **南京科技职业学院** (校园中心)
        - WGS-84: `({NJVT_CENTER_WGS84[0]}, {NJVT_CENTER_WGS84[1]})`
        - GCJ-02: `({NJVT_CENTER_GCJ02[0]:.6f}, {NJVT_CENTER_GCJ02[1]:.6f})`
        """)

        # 设置航点A和B
        st.markdown("---")
        st.subheader("设置航点坐标")
        st.markdown("在校园范围内设置 A/B 两点")

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
    """障碍物与航线规划模块"""
    st.header("3.2 障碍物与航线规划模块")
    obs_mgr = st.session_state.obstacle_mgr

    tab1, tab2, tab3 = st.tabs(["障碍物管理", "航线规划", "JSON数据"])

    # ---- 障碍物管理 ----
    with tab1:
        col_left, col_right = st.columns([2, 1])

        with col_left:
            st.subheader("3.2.1 多边形圈选障碍物")

            # 显示带绘制工具的地图
            m = MapUtils.create_base_map()
            if obs_mgr.obstacles:
                MapUtils.add_obstacle_polygons(m, obs_mgr.obstacles)

            draw_result = MapUtils.render_map(m, key="obstacle_draw_map", height=500)

            # 处理绘制结果
            if draw_result and draw_result.get("all_drawings"):
                drawings = draw_result["all_drawings"]
                if drawings:
                    st.info(f"检测到 {len(drawings)} 个绘制图形，请添加为障碍物")

            st.markdown("---")
            st.subheader("手动添加障碍物")

            with st.form("add_obstacle_form"):
                obs_name = st.text_input("障碍物名称", value=f"障碍物{len(obs_mgr.obstacles) + 1}")
                obs_height = st.number_input("高度(米)", min_value=0, max_value=500, value=30)

                st.markdown("**输入多边形顶点坐标(经度, 纬度)**")
                coords_text = st.text_area(
                    "坐标列表(每行一个: 经度,纬度)",
                    value="118.7980,32.1085\n118.7985,32.1083\n118.7988,32.1086\n118.7983,32.1088",
                    height=120,
                )

                submitted = st.form_submit_button("添加障碍物", use_container_width=True)
                if submitted:
                    try:
                        coords = []
                        for line in coords_text.strip().split("\n"):
                            if line.strip():
                                parts = line.strip().split(",")
                                coords.append((float(parts[0]), float(parts[1])))
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
                    with st.expander(f"{obs.name} ({obs.height}m)", expanded=True):
                        st.write(f"顶点数: {len(obs.coordinates)}")
                        st.write(f"高度: {obs.height}m")
                        new_height = st.number_input(
                            "调整高度", min_value=0, max_value=500,
                            value=int(obs.height),
                            key=f"height_{i}",
                        )
                        if st.button("更新高度", key=f"update_{i}"):
                            obs_mgr.update_obstacle_height(i, new_height)
                            obs_mgr.save_to_file()
                            st.success("高度已更新")

                        c_del = st.columns(2)
                        if c_del[0].button("删除", key=f"del_{i}"):
                            obs_mgr.remove_obstacle(i)
                            st.rerun()

                st.markdown("---")
                if st.button("保存所有障碍物", use_container_width=True, type="primary"):
                    obs_mgr.save_to_file()
                    st.success("障碍物数据已保存到文件!")
                if st.button("清空所有障碍物", use_container_width=True):
                    obs_mgr.obstacles.clear()
                    obs_mgr.save_to_file()
                    st.rerun()
            else:
                st.info("暂无障碍物，请在地图上圈选或手动添加")

    # ---- 航线规划 ----
    with tab2:
        st.subheader("3.2.3 飞行参数设置")

        c_param = st.columns(3)
        st.session_state.flight_height = c_param[0].number_input(
            "飞行高度(米)", min_value=1, max_value=500, value=50, key="fh"
        )
        st.session_state.safety_radius = c_param[1].number_input(
            "安全半径(米)", min_value=1, max_value=100, value=10, key="sr"
        )
        c_param[2].markdown(f"""
        **当前航点**<br>
        A: `({st.session_state.point_a[0]:.4f}, {st.session_state.point_a[1]:.4f})`<br>
        B: `({st.session_state.point_b[0] if st.session_state.point_b else '-'}, {st.session_state.point_b[1] if st.session_state.point_b else '-'})`
        """)

        if not st.session_state.point_b:
            st.warning("请先在地图定位模块中设置A、B航点")
            return

        planner = st.session_state.flight_planner
        planner.set_parameters(st.session_state.flight_height, st.session_state.safety_radius)
        planner.set_obstacles(obs_mgr.obstacles)

        start = tuple(st.session_state.point_a)
        end = tuple(st.session_state.point_b)

        if st.button("🚀 生成所有航线方案", use_container_width=True, type="primary"):
            with st.spinner("正在规划航线..."):
                results = planner.plan_all(start, end)
                st.session_state.selected_plan = results

        if st.session_state.selected_plan:
            results = st.session_state.selected_plan

            # 方案对比表
            st.subheader("3.2.4 航线规划结果对比")
            cols = st.columns(len(results))
            plan_colors = ["#4CAF50", "#2196F3", "#FF9800", "#9C27B0"]

            for idx, (result, color) in enumerate(zip(results, plan_colors)):
                with cols[idx]:
                    status_icon = "✅" if result["clears_obstacles"] else "⚠️"
                    st.markdown(f"**#{result['rank']} {result['name']}** {status_icon}")
                    st.metric("航程距离", f"{result['distance']:.1f} m")
                    st.metric("航点数量", len(result["waypoints"]))
                    if st.button(f"选择此方案", key=f"plan_{idx}", use_container_width=True):
                        st.session_state.current_display_plan = result
                        st.rerun()

            # 地图显示选中方案
            if "current_display_plan" in st.session_state:
                st.markdown("---")
                selected = st.session_state.current_display_plan

                c_map, c_info = st.columns([2, 1])
                with c_map:
                    m_plan = MapUtils.create_base_map()
                    MapUtils.add_obstacle_polygons(m_plan, obs_mgr.obstacles)
                    MapUtils.add_safety_buffer(
                        m_plan, obs_mgr.obstacles, st.session_state.safety_radius
                    )
                    MapUtils.add_flight_path(m_plan, selected, color=plan_colors[
                        results.index(selected) % len(plan_colors)
                    ])
                    MapUtils.render_map(m_plan, key="plan_display", height=500)

                with c_info:
                    st.subheader(f"方案详情: {selected['name']}")
                    st.write(f"总距离: **{selected['distance']:.1f} 米**")
                    st.write(f"航点数: **{len(selected['waypoints'])}**")
                    st.write(f"安全距离满足: {'✅ 是' if selected['clears_obstacles'] else '❌ 否'}")
                    st.write(f"飞行高度: **{st.session_state.flight_height} m**")
                    st.write(f"安全半径: **{st.session_state.safety_radius} m**")

                    st.markdown("**航点列表:**")
                    for i, wp in enumerate(selected["waypoints"]):
                        st.code(f"WP{i}: ({wp[0]:.6f}, {wp[1]:.6f})")

    # ---- JSON数据 ----
    with tab3:
        st.subheader("3.2.2 障碍物 JSON 数据")
        json_str = obs_mgr.to_json_string()
        st.code(json_str, language="json")
        st.download_button(
            "下载 JSON 文件",
            json_str.encode("utf-8"),
            file_name="obstacles_data.json",
            mime="application/json",
            use_container_width=True,
        )

        st.markdown("---")
        st.subheader("上传障碍物数据")
        uploaded = st.file_uploader("选择 JSON 文件", type=["json"])
        if uploaded:
            try:
                data = json.load(uploaded)
                obs_mgr.obstacles.clear()
                for d in data:
                    from utils.obstacle_manager import Obstacle
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
    """飞行监控模块"""
    st.header("3.3 飞行监控模块")
    sim = st.session_state.mavlink_sim

    # 控制栏
    ctrl_col1, ctrl_col2, ctrl_col3 = st.columns(3)

    with ctrl_col1:
        if not st.session_state.monitor_running:
            if st.button("▶️ 开始监控", use_container_width=True, type="primary"):
                sim.arm()
                sim.running = True
                st.session_state.monitor_running = True
                st.rerun()
        else:
            if st.button("⏹️ 停止监控", use_container_width=True):
                sim.disarm()
                sim.running = False
                st.session_state.monitor_running = False
                st.rerun()

    with ctrl_col2:
        mode = st.selectbox(
            "飞行模式",
            ["STABILIZED", "AUTO", "GUIDED", "LOITER", "RTL", "LAND"],
            key="flight_mode_select",
        )
        if st.button("切换模式", use_container_width=True):
            sim.set_mode(mode)

    with ctrl_col3:
        st.markdown(f"""
        **无人机状态**<br>
        心跳计数: `{sim.heartbeat_seq}`<br>
        系统状态: `{'🟢 运行中' if sim.running else '🔴 已停止'}`<br>
        电机状态: `{'🟢 已解锁' if sim.armed else '🔴 已锁定'}`
        """)

    if st.session_state.monitor_running:
        # 生成仿真数据
        sim.generate_all()

    # 仪表盘
    st.markdown("---")

    metric_cols = st.columns(6)
    state = sim.state

    metrics = [
        ("纬度", f"{state['lat']:.6f}°"),
        ("经度", f"{state['lng']:.6f}°"),
        ("高度", f"{state['alt']:.1f} m"),
        ("地速", f"{state['ground_speed']:.1f} m/s"),
        ("航向", f"{state['heading']:.0f}°"),
        ("电池", f"{state['battery_remaining']:.0f}%"),
    ]
    for col, (label, value) in zip(metric_cols, metrics):
        col.metric(label, value)

    # 详细状态面板
    detail_col1, detail_col2, detail_col3 = st.columns(3)

    with detail_col1:
        st.subheader("姿态数据")
        st.markdown(f"""
        - 俯仰角(Pitch): **{state['pitch']:.2f}°**
        - 横滚角(Roll): **{state['roll']:.2f}°**
        - 偏航角(Yaw): **{state['yaw']:.2f}°**
        """)

    with detail_col2:
        st.subheader("导航数据")
        st.markdown(f"""
        - GPS定位: **{'3D Fix' if state['gps_fix'] == 3 else 'No Fix'}**
        - 卫星数量: **{state['gps_satellites']}**
        - 垂直速度: **{state['climb_rate']:.2f} m/s**
        - 空速: **{state['air_speed']:.1f} m/s**
        """)

    with detail_col3:
        st.subheader("电源状态")
        bat_color = "🟢" if state["battery_remaining"] > 30 else ("🟡" if state["battery_remaining"] > 15 else "🔴")
        st.markdown(f"""
        - 电压: **{state['battery_voltage']:.2f} V** {bat_color}
        - 剩余: **{state['battery_remaining']:.0f}%**
        - 油门: **{state['throttle']}%**
        - 飞行模式: **{state['flight_mode']}**
        """)

    # 监控地图
    st.markdown("---")
    st.subheader("实时位置地图")

    m_monitor = MapUtils.create_base_map(
        center=(state["lng"], state["lat"]),
    )

    # 无人机位置标记
    drone_icon = folium.DivIcon(
        html='<div style="font-size:24px;">✈️</div>',
        icon_size=(30, 30),
    )
    folium.Marker(
        location=[state["lat"], state["lng"]],
        popup=f"无人机位置\n高度: {state['alt']:.1f}m\n速度: {state['ground_speed']:.1f}m/s",
        icon=drone_icon,
    ).add_to(m_monitor)

    # 障碍物显示
    obs_mgr = st.session_state.obstacle_mgr
    if obs_mgr.obstacles:
        MapUtils.add_obstacle_polygons(m_monitor, obs_mgr.obstacles)

    # 飞行路径显示
    if st.session_state.get("current_display_plan"):
        MapUtils.add_flight_path(m_monitor, st.session_state.current_display_plan, color="#2196F3")

    MapUtils.render_map(m_monitor, key="monitor_map", height=400)

    # MAVLink 报文日志
    st.markdown("---")
    st.subheader("MAVLink 报文日志")

    log_data = sim.get_log_table_data(30)
    if log_data:
        st.dataframe(log_data, use_container_width=True, hide_index=True)
    else:
        st.info("暂无报文数据")

    # 自动刷新
    if st.session_state.monitor_running:
        time.sleep(0.5)
        st.rerun()


# ============================================================
# 3.4 通信链路展示模块
# ============================================================
def page_comm():
    """通信链路展示模块"""
    st.header("3.4 通信链路展示模块")

    comm = st.session_state.comm_topology

    tab1, tab2 = st.tabs(["3.4.1 通信拓扑", "3.4.2 MAVLink数据流"])

    with tab1:
        st.subheader("GCS - OBC - FCU 通信拓扑结构")

        # 显示拓扑图
        svg_html = comm.generate_topology_html()
        st.components.v1.html(svg_html, height=530)

        # 节点状态
        st.markdown("---")
        st.subheader("节点状态详情")

        status_summary = comm.get_node_status_summary()
        s1, s2, s3 = st.columns(3)
        s1.metric("总节点数", status_summary["total"])
        s2.metric("在线", status_summary["online"], delta="正常")
        s3.metric("离线", status_summary["offline"])

        # 节点列表
        for node in comm.nodes:
            icon = "🟢" if node.status == "online" else "🔴"
            with st.expander(f"{icon} {node.name} [{node.node_type}]"):
                c_info = st.columns(3)
                c_info[0].write(f"IP: `{node.ip}`")
                c_info[1].write(f"端口: `{node.port}`")
                c_info[2].write(f"状态: `{node.status}`")

        # 链路信息
        st.markdown("---")
        st.subheader("链路信息")
        link_cols = st.columns(min(len(comm.links), 4))
        for i, link in enumerate(comm.links):
            with link_cols[i % len(link_cols)]:
                latency = link.latency_ms
                latency_color = "🟢" if latency < 10 else ("🟡" if latency < 20 else "🔴")
                st.markdown(f"""
                **{link.source[:4]} → {link.target[:4]}**
                - 协议: `{link.protocol}`
                - 波特率: `{link.baud_rate}`
                - 延迟: {latency_color} `{latency:.1f}ms`
                """)

    with tab2:
        st.subheader("MAVLink 数据流与报文显示")

        # 心跳包控制
        hb_col1, hb_col2 = st.columns(2)
        with hb_col1:
            if st.button("💓 发送心跳包", use_container_width=True, type="primary"):
                comm.add_heartbeat()
                st.rerun()

            if st.button("🔄 重置计数", use_container_width=True):
                comm.heartbeat_count = 0
                comm.message_log.clear()
                st.rerun()

        with hb_col2:
            st.metric("心跳包计数", comm.heartbeat_count)
            if comm.last_heartbeat > 0:
                elapsed = time.time() - comm.last_heartbeat
                st.metric("上次心跳距今", f"{elapsed:.1f}s")

        # 心跳包详情
        if comm.message_log:
            latest = comm.message_log[-1]
            st.markdown("#### 最近心跳包")
            st.json(latest.data)

        # MAVLink 报文表格
        st.markdown("---")
        st.subheader("报文历史记录")

        recent = comm.get_recent_messages(20)
        if recent:
            msg_rows = []
            for msg in reversed(recent):
                msg_rows.append({
                    "时间": time.strftime("%H:%M:%S", time.localtime(msg.timestamp)),
                    "来源": msg.source,
                    "目标": msg.target,
                    "消息": msg.msg_name,
                    "数据": json.dumps(msg.data, ensure_ascii=False)[:60],
                })
            st.dataframe(msg_rows, use_container_width=True, hide_index=True)
        else:
            st.info("暂无报文记录，点击发送心跳包开始")


# ============================================================
# 主程序入口
# ============================================================
def main():
    init_session_state()
    sidebar_navigation()

    # 根据选中页面渲染
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
