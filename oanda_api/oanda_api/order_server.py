import rclpy
from rclpy.node import Node


class OrderServer(Node):

    def __init__(self):
        super().__init__("order_server")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)


def main(args=None):
    rclpy.init(args=args)
    order = OrderServer()
    rclpy.spin(order)
    rclpy.shutdown()
