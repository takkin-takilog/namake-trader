from dataclasses import dataclass
import datetime as dt

RosParamType = (
    bool | int | float | str | list[bool] | list[int] | list[float] | list[str] | None
)


@dataclass
class RosParam:
    """
    ROS parameter elements.
    """

    name: str
    type: int
    value: RosParamType = None


@dataclass
class RosParamTime(RosParam):
    """
    ROS parameter elements for time.
    """

    time: dt.time = dt.time()
