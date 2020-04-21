import rclpy
from rclpy.node import Node


class CandlestickManager(Node):

    def __init__(self):
        super().__init__("candlestick_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)


def main(args=None):
    rclpy.init(args=args)
    cm = CandlestickManager()

    try:
        rclpy.spin(cm)
    except KeyboardInterrupt:
        pass

    cm.destroy_node()
    rclpy.shutdown()
