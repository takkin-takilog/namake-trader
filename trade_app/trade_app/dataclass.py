from typing import Any
from dataclasses import dataclass
import datetime as dt
from rclpy.parameter import Parameter


@dataclass
class RosParam:
    """
    ROS parameter elements.
    """

    name: str
    type: Parameter.Type
    value: Any = None


@dataclass
class RosParamTime(RosParam):
    """
    ROS parameter elements for time.
    """

    time: dt.time = dt.time()
