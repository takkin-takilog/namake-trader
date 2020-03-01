import rclpy
from rclpy.node import Node


class OrderService(Node):

    def __init__(self):
        super().__init__("order_service")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    rclpy.spin(order)
    rclpy.shutdown()
