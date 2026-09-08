from pathlib import Path

from diablo_web_interface.hardware_manager import HardwareManager


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


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
    wheel_odom = read_text("diablo_web_interface/wheel_odom.py")
    assert 'declare_parameter("max_wheel_delta", 1.5)' in wheel_odom
    assert "Ignored discontinuous wheel sample" in wheel_odom


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


def test_web_ui_has_mapping_teleop_and_topic_echo_panels():
    app = read_text("src/App.tsx")
    drive = read_text("src/components/DriveView.tsx")
    mapping = read_text("src/components/MappingView.tsx")
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
    assert 'from "./components/MappingView"' in app
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
    assert "Live Occupancy Grid" in mapping
    assert "START MAPPING" in mapping
    assert "SAVE MAP" in mapping
    assert 'type: "save_map"' in mapping
    assert 'type: "stop_mapping"' in mapping
    assert 'LaunchToggleButton' in mapping
    assert 'LaunchControls' in read_text("src/components/LaunchControls.tsx")
    assert 'type: "stop_hardware"' in read_text("src/components/LaunchControls.tsx")
    assert 'type: "joint_position"' in drive
    assert 'Dynamixel Joint Control' in drive
    assert "mapping.active" in mapping
    assert "defaultCollapsed" in panel
    assert "onToggleCollapse" in sidebar
    assert '"/diablo/reset_encoder"' in ros_node
    assert '"/start_motor"' in ros_node
    assert 'DeclareLaunchArgument("lidar_start_service", default_value="/start_motor")' in web_launch
    assert 'DeclareLaunchArgument("lidar_start_service", default_value="/start_motor")' in nav2_launch
    assert 'DeclareLaunchArgument("lidar_start_service_type", default_value="empty")' in web_launch
    assert 'DeclareLaunchArgument("lidar_start_service_type", default_value="empty")' in nav2_launch
    assert "not self.lidar_start_command" in ros_node
    assert "HardwareManager" in ros_node
    assert "def start_hardware" in ros_node
    assert "def list_maps" in ros_node
    assert 'reset_encoder' in web_node
    assert 'start_lidar' in web_node
    assert '"/api/hardware/start"' in web_node
    assert '"/api/navigation/start"' in web_node
    assert '"/api/mapping/start"' in web_node
    assert '"/api/mapping/save"' in web_node
    assert '"/api/mapping/stop"' in web_node
    assert '"/api/hardware/stop"' in web_node
    assert '"/api/navigation/stop-localization"' in web_node
    assert '"/api/navigation/stop"' in web_node
    assert '"/api/joints/{motor_id}/position"' in web_node
    assert "class HardwareManager" in hardware_manager
    assert "def stop_hardware" in hardware_manager
    assert "JOINT_DEFINITIONS" in ros_node
    assert "Only Dynamixel IDs 1 through 10" in ros_node
    assert "trajectory_msgs" in read_text("package.xml")
    assert 'data-tab="mapping"' in html
    assert 'data-tab="topics"' in html
    assert 'data-tab="settings"' in html
    assert 'id="map-canvas"' in html
    assert 'id="topic-cards"' in html
    assert 'id="launch-localization"' in html
    assert 'id="launch-navigation"' in html
    assert 'id="launch-mapping"' in html
    assert 'id="confirm-modal"' in html
    assert 'id="joint-sliders"' in html
    assert 'type: "save_map"' in javascript
    assert 'type: "start_mapping"' in javascript
    assert 'stop_hardware' in javascript
    assert 'confirm-modal' in javascript
    assert 'joint_position' in javascript
    assert 'type: "subscribe"' in javascript
    assert "start-hardware" in javascript
    assert "relative_asset.is_absolute()" in web_node
    assert '".." in relative_asset.parts' in web_node
    assert "/dev/diablo_controller" in web_launch
    assert "/dev/rplidar" in web_launch
    assert "/dev/u2d2_arm" in web_launch
    assert "/dev/u2d2_hand" in web_launch
    assert "W A S D" in html


def test_mapping_gate_does_not_require_optional_dynamixel_feedback():
    manager = HardwareManager(
        _Logger(),
        diablo_command="diablo",
        lidar_command="lidar",
        dynamixel_command="dynamixel",
    )
    manager.mark_message("diablo")
    manager.mark_message("lidar")
    manager.update([])

    status = manager.snapshot()
    assert status["ready"] is True
    assert status["mapping_ready"] is True
    assert status["all_ready"] is False


def test_intentional_hardware_stop_is_not_reported_as_process_error(monkeypatch):
    class _Process:
        pid = 12345

        def __init__(self):
            self.return_code = None

        def poll(self):
            return self.return_code

        def wait(self, timeout):
            assert timeout == 2.0
            self.return_code = -15

    manager = HardwareManager(_Logger(), diablo_command="diablo")
    manager._processes["diablo"] = _Process()
    manager._started_at["diablo"] = 0.0
    monkeypatch.setattr("os.killpg", lambda _pid, _signal: None)

    assert manager.stop_hardware()["requested"] is True
    manager.update([])
    component = manager.snapshot()["components"][0]
    assert component["state"] == "offline"
