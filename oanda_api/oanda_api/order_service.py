from requests.exceptions import ConnectionError
import rclpy
from rclpy.node import Node
from api_msgs.srv import OrderCreateSrv
from api_msgs.srv._order_create_srv import OrderCreateSrv_Request as OrderCreateReq
from api_msgs.srv._order_create_srv import OrderCreateSrv_Response as OrderCreateRsp
from oandapyV20.endpoints.orders import OrderCreate
from oandapyV20 import API

ORDER_TYP_DICT = {
    OrderCreateReq.TYP_MARKET: "MARKET",
    OrderCreateReq.TYP_LIMIT: "LIMIT",
    OrderCreateReq.TYP_STOP: "STOP",
}

ORDER_INST_DICT = {
    OrderCreateReq.INST_USD_JPY: "USD_JPY",
    OrderCreateReq.INST_EUR_JPY: "EUR_JPY",
    OrderCreateReq.INST_EUR_USD: "EUR_USD",
}


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
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        callback = self.__cb_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__api = API(access_token=access_token)

        self.__account_number = account_number

    def __cb_order_create(self, request, response):

        data = {
            "order": {
                "type": ORDER_TYP_DICT[request.type],
            }
        }

        data_order = data["order"]

        if ((request.type == OrderCreateReq.TYP_LIMIT)
            or (request.type == OrderCreateReq.TYP_STOP)):
            tmp = {
                "price": request.units,
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": ORDER_INST_DICT[request.instrument],
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

        try:
            rsp = self.__api.request(ep)
        except ConnectionError as ce:
            self.__logger.error("%s" % ce)
            response.fail_reason_code = OrderCreateRsp.REASON_CONNECTION_ERROR
        else:
            if "orderFillTransaction" in rsp.keys():
                response.fail_reason_code = OrderCreateRsp.REASON_UNSET
            elif "orderCancelTransaction" in rsp.keys():
                reason = rsp["orderCancelTransaction"]["reason"]
                if reason == "MARKET_HALTED":
                    response.fail_reason_code = OrderCreateRsp.REASON_MARKET_HALTED
                else:
                    response.fail_reason_code = OrderCreateRsp.REASON_OTHERS
            else:
                response.fail_reason_code = OrderCreateRsp.REASON_OTHERS

            import json
            print(json.dumps(rsp, indent=2))

        return response


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        order.destroy_node()
        rclpy.shutdown()
