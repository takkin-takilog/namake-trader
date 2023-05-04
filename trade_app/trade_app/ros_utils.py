import datetime as dt
from rclpy.node import Node
from .constant import FMT_TIME_HMS
from .exception import RosParameterErrorException
from .dataclass import RosParam, RosParamTime


def set_parameters(node: Node, ros_param: RosParam) -> None:
    """
    Set parameters for the RosParam.

    :param node: Node in the ROS graph.
    :param ros_param: RosParam.
    """
    node.declare_parameter(ros_param.name, ros_param.type)

    para = node.get_parameter(ros_param.name)
    ros_param.value = para.value

    if isinstance(ros_param, RosParamTime):
        datetime_ = dt.datetime.strptime(para.value, FMT_TIME_HMS)
        ros_param.time = datetime_.time()
        node.get_logger().debug(
            "[Param]{}:[{}] {}".format(
                ros_param.name, ros_param.time, type(ros_param.time)
            )
        )
    else:
        node.get_logger().debug(
            "[Param]{}:[{}] {}".format(
                ros_param.name, ros_param.value, type(ros_param.value)
            )
        )


def validate_ros_param_length(node: Node, ros_param_list: list[RosParam]) -> None:
    """
    Validate ROS parameter length.

    :param node: The Node in the ROS graph.
    :param ros_param_list: "RosParam" list.
    """
    base_len = len(ros_param_list[0].value)
    validation_ok = True
    for ros_param in ros_param_list:
        if base_len != len(ros_param.value):
            validation_ok = False
            break

    if not validation_ok:
        node.get_logger().error(
            "{:!^50}".format(" ROS Parameter Error: Length not match ")
        )
        for ros_param in ros_param_list:
            node.get_logger().error(
                "  - {}:[{}]".format(ros_param.name, len(ros_param.value))
            )
        raise RosParameterErrorException("ROS Parameter Error: Length not match.")
