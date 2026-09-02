from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_diablo_motion_control_uses_native_message_and_topic():
    bridge = read_text("diablo_web_interface/motion_cmd_bridge.py")
    mux = read_text("diablo_web_interface/motion_cmd_mux.py")
    web = read_text("diablo_web_interface/ros_node.py")

    assert "from motion_msgs.msg import MotionCtrl" in bridge
    assert "value.forward" in bridge
    assert "value.left" in bridge
    assert "/diablo/MotionCmd/manual" in mux
    assert "/diablo/MotionCmd/nav" in mux
    assert "/diablo/MotionCmd" in mux
    assert "MotionCtrl" in web


def test_nav2_launch_contains_action_stack_and_safety_bridges():
    launch = read_text("launch/navigation.launch.py")
    params = read_text("config/nav2_params.yaml")

    for package in (
        "nav2_map_server",
        "nav2_amcl",
        "nav2_controller",
        "nav2_planner",
        "nav2_bt_navigator",
        "nav2_lifecycle_manager",
    ):
        assert f'package="{package}"' in launch
    assert "motion_cmd_bridge" in launch
    assert "motion_cmd_mux" in launch
    assert "DWBLocalPlanner" in params
    assert "local_costmap:" in params
    assert "global_costmap:" in params


def test_web_ui_has_teleop_navigation_and_topic_echo_panels():
    app = read_text("src/App.tsx")
    drive = read_text("src/components/DriveView.tsx")
    navigation = read_text("src/components/NavigationView.tsx")
    panel = read_text("src/components/Panel.tsx")
    sidebar = read_text("src/components/Sidebar.tsx")
    connection = read_text("src/hooks/useDiabloConnection.ts")
    package_json = read_text("package.json")
    html = read_text("diablo_web_interface/static/index.html")
    javascript = read_text("diablo_web_interface/static/app.js")
    ros_node = read_text("diablo_web_interface/ros_node.py")
    web_node = read_text("diablo_web_interface/web_node.py")
    hardware_manager = read_text("diablo_web_interface/hardware_manager.py")
    web_launch = read_text("launch/web_interface.launch.py")
    nav2_launch = read_text("launch/nav2_web.launch.py")

    assert '"vite"' in package_json
    assert '"react"' in package_json
    assert 'from "./components/DriveView"' in app
    assert 'from "./components/NavigationView"' in app
    assert 'from "./components/TopicsView"' in app
    assert 'from "./components/SettingsView"' in app
    assert 'new WebSocket' in connection
    assert "/ws" in connection
    assert '"/api/teleop"' in connection
    assert 'wheel_pose' in drive
    assert 'Trajectory Map' in drive
    assert 'Keybind Legend' in drive
    assert 'RESET ENCODER' in drive
    assert 'START LIDAR' in drive
    assert 'key: "z"' in drive
    assert 'key: "x"' in drive
    assert "hardwareReady" in drive
    assert "Front Obstacle Laser" not in drive
    assert "Magnetic Navigation Sensor" not in drive
    assert "Navigation Map" in navigation
    assert "Pose & Stations" in navigation
    assert "Navigation Controls" in navigation
    assert "global_costmap" in navigation
    assert "local_costmap" in navigation
    assert "INFLATION LAYER" in navigation
    assert "start_hardware" in navigation
    assert "defaultCollapsed" in panel
    assert "onToggleCollapse" in sidebar
    assert '"/diablo/reset_encoder"' in ros_node
    assert '"/start_motor"' in ros_node
    assert 'DeclareLaunchArgument("lidar_start_service", default_value="/start_motor")' in web_launch
    assert 'DeclareLaunchArgument("lidar_start_service", default_value="/start_motor")' in nav2_launch
    assert "HardwareManager" in ros_node
    assert "def start_hardware" in ros_node
    assert "def list_maps" in ros_node
    assert 'reset_encoder' in web_node
    assert 'start_lidar' in web_node
    assert '"/api/hardware/start"' in web_node
    assert '"/api/navigation/start"' in web_node
    assert '"/api/mapping/start"' in web_node
    assert "class HardwareManager" in hardware_manager
    assert 'data-tab="teleop"' in html
    assert 'data-tab="navigation"' in html
    assert 'data-tab="topics"' in html
    assert 'data-tab="settings"' in html
    assert 'id="nav-canvas"' in html
    assert 'id="topic-cards"' in html
    assert 'type: "goal_pose"' in javascript
    assert 'type: "subscribe"' in javascript
    assert "drawNavigation" in javascript
    assert "preview-sidebar-toggle" in javascript
    assert "start-hardware" in javascript
