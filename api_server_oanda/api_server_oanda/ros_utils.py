import datetime as dt
from rclpy.node import Node
from .constant import FMT_TIME_HMS
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
        node.get_logger().debug("[Param]{}:[{}] {}".format(ros_param.name, ros_param.time, type(ros_param.time)))
    else:
        node.get_logger().debug("[Param]{}:[{}] {}".format(ros_param.name, ros_param.value, type(ros_param.value)))
