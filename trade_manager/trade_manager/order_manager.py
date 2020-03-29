import rclpy
from rclpy.node import Node


class OrderManager(Node):

    def __init__(self):
        super().__init__("order_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        TPCNM_ORDER_REQUEST = "order_request"

    def __on_recv_order_request(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()
    try:
        rclpy.spin(order_manager)
    except KeyboardInterrupt:
        order_manager.destroy_node()
        rclpy.shutdown()
