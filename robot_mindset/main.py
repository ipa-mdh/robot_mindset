import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nicegui import Client, app, ui, ui_run

from robot_mindset.gui import create_pages

class NiceGuiNode(Node):
    def __init__(self):
        super().__init__('nicegui_node')
        
        package_name = 'robot_mindset'
        share_dir = Path(get_package_share_directory(package_name))

        with Client.auto_index_client:
            create_pages.create(share_dir)

# # Start the NiceGUI application
# ui.run()

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass

def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')