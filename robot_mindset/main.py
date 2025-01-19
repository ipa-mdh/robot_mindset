import os
import threading
from pathlib import Path
from datetime import datetime

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Bool
from spark_msgs.srv import SparkRegistration, SparkUpdate

from nicegui import Client, app, ui, ui_run

from robot_mindset.gui import create_pages
from robot_mindset.spark.spark_detector import get_spark_packages
from robot_mindset.utils.config_data import ConfigData


class NiceGuiNode(Node):
    def __init__(self):
        package_name = 'robot_mindset'
        super().__init__(package_name)
        
        share_dir = Path(get_package_share_directory(package_name))

        # self.spark_package_names = get_spark_packages()

        # Create a ConfigData instance instead of a plain dictionary.
        self.config = ConfigData(
            share_dir=share_dir,
            spark_list={}, # Initialize an empty dictionary to hold spark registrations
            spark_list_callback=None,  # Will be set by the UI page later
            # get_spark_parameters_fnc=None,
            set_spark_parameters_fnc=self.update_parameter_value,  # Provide your setter function if needed
            use_heardbeat=True
        )
        
        self.init_service()
        self.init_topic()
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        with Client.auto_index_client:
            # Build the UI pages using the ConfigData object.
            create_pages.create(self.config)
        
    def init_service(self):
        self.spark_registration_srv = self.create_service(
            SparkRegistration,
            'spark_registration',
            self.spark_registration_callback
        )
    
    def init_topic(self):
        self.publisher_spark_heardbeat = self.create_publisher(Bool, '/spark_heardbeat', 10)
    
    def update_parameter_value(self, node_name: str, param_name: str, new_value: str):
        print(f'update_parameter_value: {node_name}, {param_name}')
        # pass
    
        set_cli = self.create_client(SparkUpdate, f'/{node_name}/update_parameters')
        while not set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_parameters service...')
        
        req = SparkUpdate.Request()
        req.node_name = node_name
        req.parameters = json.dumps(new_value)
        
        # self.get_logger().info(f'Sending set_parameters request for: {param_name} with new value: {new_value}')
        # self.get_logger().info(f'Sending set_parameters request for: {param_name} with new value: ...')
        future = set_cli.call_async(req)
    
    def timer_callback(self) -> None:
        msg = Bool()
        msg.data = self.config.use_heardbeat
        
        if callable(self.config.spark_list_callback):
            self.config.spark_list_callback()
        
        self.publisher_spark_heardbeat.publish(msg)

    def spark_registration_callback(self, request, response) -> any:
        # Create a buffer for the incoming spark registration data.
        buffer = {
            'node_name': request.node_name,
            'parameters': request.parameters,
            'date': datetime.now().strftime('%c')
        }
        # Update the spark_list in the ConfigData instance.
        self.config.spark_list.update({str(request.node_name): buffer})
        
        response.registrated = True

        return response

    def get_spark_parameters(self, spark_name: str) -> None:
        # Implement your spark parameters request if needed.
        pass

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py,
    # but it's empty to enable NiceGUI auto-reloading.
    pass

def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


# Run the ROS node in a separate thread once NiceGUI starts up.
app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here

# Start the NiceGUI app.
ui.run(title="Robot Mindset",
       uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
       favicon='🤖')
