import rclpy
from rclpy.node import Node
from api_msgs.srv import OrderCreateSrv
from api_msgs.srv._order_create_srv import Metaclass_OrderCreateSrv_Request as OrderCreateReq
from oandapyV20.endpoints.orders import OrderCreate
from oandapyV20 import API


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

        self.__account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self.__logger.debug("[OANDA]Account Number:%s" % self.__account_number)
        self.__logger.debug("[OANDA]Access Token:%s" % access_token)

        # Create service "order_create"
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        callback = self.__cb_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__api = API(access_token=access_token)

    def __cb_order_create(self, request, response):

        data = {
            "order": {
                "type": request.type,
            }
        }

        data_order = data["order"]

        if (request.type == OrderCreateReq.TYP_LIMIT) or (request.type == OrderCreateReq.TYP_STOP):
            tmp = {
                "price": request.units,
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": request.instrument,
            "units": request.units,
            "positionFill": "DEFAULT",
            "takeProfitOnFill": {
                "timeInForce": "GTC",
                "price": request.take_profit_price
            },
            "stopLossOnFill": {
                "timeInForce": "GTC",
                "price": request.stop_loss_price
            },
        }
        data_order.update(tmp)

        ep = OrderCreate(accountID=self.__account_number, data=data)
        rsp = self.__api.request(ep)

        import json
        print(json.dumps(rsp, indent=2))

        return response


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    rclpy.spin(order)
    rclpy.shutdown()
