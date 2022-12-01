from typing import Optional, Any
from dataclasses import dataclass
import datetime as dt


@dataclass
class RosParam:
    """
    ROS parameter elements.
    """

    name: str
    type: int
    value: Optional[Any] = None


@dataclass
class RosParamTime(RosParam):
    """
    ROS parameter elements for time.
    """

    time: dt.time = dt.time()
