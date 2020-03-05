import rclpy
from rclpy.node import Node
from api_msgs.srv import OrderCreate
from oandapyV20 import API
from oandapyV20.exceptions import V20Error, StreamTerminated


class OrderService(Node):

    def __init__(self):
        super().__init__("order_service")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self.__logger.debug("[OANDA]Account Number:%s" % account_number)
        self.__logger.debug("[OANDA]Access Token:%s" % access_token)

        # Create service "order_create"
        srv_type = OrderCreate
        srv_name = "order_create"
        callback = self.__cb_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)


        self.__api = API(access_token=access_token)


    def __cb_order_create(self, request, response):

        return response

def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    rclpy.spin(order)
    rclpy.shutdown()
