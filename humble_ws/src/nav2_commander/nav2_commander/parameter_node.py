__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

from rclpy.node import Node

class ParameterNode(Node):
    """
    ROS parameter manager node."""
    def __init__(self, node_name:str)->None:
        super().__init__(node_name)
        self._set_param("use_localizer", False)
        self._set_param("use_slam", True)

    def _set_param(self, param_name:str, param_value=None)->None:
        """
        Set parameter to the parameter server."""
        self.declare_parameter(param_name, param_value)

    def _get_param(self, param_name:str):
        """
        Get parameter from the parameter server."""
        return self.get_parameter(param_name).value