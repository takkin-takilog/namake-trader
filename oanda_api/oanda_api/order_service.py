import rclpy
from rclpy.node import Node
from api_msgs.srv import OrderCreate


class OrderService(Node):

    def __init__(self):
        super().__init__("order_service")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Create service "order_create"
        srv_type = OrderCreate
        srv_name = "order_create"
        callback = self.__cb_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

    def __cb_order_create(self, request, response):
        pass


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    rclpy.spin(order)
    rclpy.shutdown()
