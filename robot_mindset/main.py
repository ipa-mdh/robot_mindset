# import threading
# from pathlib import Path
# from datetime import datetime

# import rclpy
# from rclpy.executors import ExternalShutdownException
# from rclpy.node import Node
# from rcl_interfaces.srv import GetParameters, SetParameters
# from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
# from ament_index_python.packages import get_package_share_directory


# from std_msgs.msg import String, Bool
# from spark_msgs.srv import SparkRegistration

# from nicegui import Client, app, ui, ui_run

# from robot_mindset.gui import create_pages
# from robot_mindset.spark.spark_detector import get_spark_packages
# from robot_mindset.utils.config_data import ConfigData

# class NiceGuiNode(Node):
#     def __init__(self):
#         package_name = 'robot_mindset'
#         super().__init__(package_name)
        
#         share_dir = Path(get_package_share_directory(package_name))

#         self.spark_package_names = get_spark_packages()
        
#         self.spark_list = {}
#         self.data = {"spark_list": self.spark_list,
#                      "share_dir": share_dir,
#                      "spark_list_callback": None,
#                      "get_spark_parameters_fnc": None,
#                      "set_spark_parameters_fnc": None}
        
#         with Client.auto_index_client:
#             # self.test_label = ui.label('Hello, world!')
#             create_pages.create(self.data)
        
#         self.spark_registration_srv = self.create_service(SparkRegistration, 'spark_registration', self.spark_registration_callback)
        
#         self.publisher_spark_registration_refresh_ = self.create_publisher(Bool, 'spark_registration_refresh', 10)
#         timer_period = 1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def init_parameter_service(self, node_name):
#         #  Create clients for the target node's parameter services
#         self.get_cli = self.create_client(GetParameters, f'{node_name}/get_parameters')
#         self.set_cli = self.create_client(SetParameters, f'{node_name}/set_parameters')
        
#         # Wait for the services to be available
#         while not self.get_cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for get_parameters service...')
#         while not self.set_cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for set_parameters service...')
        
#         # First, get the parameter from the parameter_node
#         self.get_parameter_value('my_parameter')
    
#     def get_spark_parameters_fnc(self, node_name):
#         # self.init_parameter_service(node_name)
#         # return self.get_parameter_value
#         return "test"
    
#     def test(self):
#         my_param = self.get_parameter('/spark_my_namespace/spark_my_node/parameters_json')
#         print(f'test: {my_param}')

#     def timer_callback(self):
#         msg = Bool()
#         msg.data = True
        
#         if callable(self.data['spark_list_callback']):
#             self.data['spark_list_callback']()
        
#         self.publisher_spark_registration_refresh_.publish(msg)
#         # self.get_logger().info('Publishing: "%s"' % msg.data)
        
#         # self.test()

#     def spark_registration_callback(self, request, response):
#         buffer = {'node_name': request.node_name,
#                   'parameters': request.parameters,
#                   'date': datetime.now().strftime('%c')}
#         self.spark_list.update({str(request.node_name): buffer})
        
#         response.registrated = True
#         # self.get_logger().info(f'Incoming request\n  node_name: {request.node_name}\n  parameters: {request.parameters}')
#         # self.test_label.text = f'node_name: {request.node_name}'
        
#         # if 'spark_list_callback' in self.data:
#         #     self.data['spark_list_callback']()

#         return response

#     def get_spark_parameters(self, spark_name):
        
#         pass

# def main() -> None:
#     # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
#     pass

# def ros_main() -> None:
#     rclpy.init()
#     node = NiceGuiNode()
#     try:
#         rclpy.spin(node)
#     except ExternalShutdownException:
#         pass


# app.on_startup(lambda: threading.Thread(target=ros_main).start())
# ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
# ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')

import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Bool
from spark_msgs.srv import SparkRegistration

from nicegui import Client, app, ui, ui_run

from robot_mindset.gui import create_pages
from robot_mindset.spark.spark_detector import get_spark_packages
from robot_mindset.utils.config_data import ConfigData


class NiceGuiNode(Node):
    def __init__(self):
        package_name = 'robot_mindset'
        super().__init__(package_name)
        
        share_dir = Path(get_package_share_directory(package_name))

        self.spark_package_names = get_spark_packages()
        # Initialize an empty dictionary to hold spark registrations
        self.spark_list = {}

        # Create a ConfigData instance instead of a plain dictionary.
        self.config = ConfigData(
            share_dir=share_dir,
            spark_list=self.spark_list,
            spark_list_callback=None,  # Will be set by the UI page later
            get_spark_parameters_fnc=None,
            set_spark_parameters_fnc=None  # Provide your setter function if needed
        )
        
        with Client.auto_index_client:
            # Build the UI pages using the ConfigData object.
            create_pages.create(self.config)
        
        self.spark_registration_srv = self.create_service(
            SparkRegistration,
            'spark_registration',
            self.spark_registration_callback
        )
        
        self.publisher_spark_registration_refresh_ = self.create_publisher(Bool, 'spark_registration_refresh', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_parameter_service(self, node_name: str) -> None:
        # Create clients for the target node's parameter services
        self.get_cli = self.create_client(GetParameters, f'{node_name}/get_parameters')
        self.set_cli = self.create_client(SetParameters, f'{node_name}/set_parameters')
        
        # Wait for the services to be available
        while not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_parameters service...')
        while not self.set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_parameters service...')
        
        # First, get the parameter from the parameter_node
        self.get_parameter_value('my_parameter')
    
    def get_spark_parameters_fnc(self, node_name: str) -> str:
        # You can uncomment and complete this if you plan to implement a full parameter service interaction.
        # self.init_parameter_service(node_name)
        # return self.get_parameter_value
        return "test"
    
    def test(self) -> None:
        my_param = self.get_parameter('/spark_my_namespace/spark_my_node/parameters_json')
        print(f'test: {my_param}')

    def timer_callback(self) -> None:
        msg = Bool()
        msg.data = True
        
        if callable(self.config.spark_list_callback):
            self.config.spark_list_callback()
        
        self.publisher_spark_registration_refresh_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.test()

    def spark_registration_callback(self, request, response) -> any:
        # Create a buffer for the incoming spark registration data.
        buffer = {
            'node_name': request.node_name,
            'parameters': request.parameters,
            'date': datetime.now().strftime('%c')
        }
        # Update the spark_list in the ConfigData instance.
        self.spark_list.update({str(request.node_name): buffer})
        
        response.registrated = True

        # Uncomment if you need to trigger the UI update here.
        # if self.config.spark_list_callback:
        #     self.config.spark_list_callback()

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
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
