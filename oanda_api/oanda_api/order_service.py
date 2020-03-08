from requests.exceptions import ConnectionError
import rclpy
from rclpy.node import Node
from api_msgs.srv import OrderCreateSrv
from api_msgs.msg import OrderType, Instrument
from api_msgs.msg import FailReasonCode as frc
from oandapyV20.endpoints.orders import OrderCreate
from oandapyV20 import API

ORDER_TYP_DICT = {
    OrderType.TYP_MARKET: "MARKET",
    OrderType.TYP_LIMIT: "LIMIT",
    OrderType.TYP_STOP: "STOP",
}

ORDER_INST_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
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

    def __cb_order_create(self, req, rsp):

        data = {
            "order": {
                "type": ORDER_TYP_DICT[req.ordertype_msg.type],
            }
        }

        data_order = data["order"]

        if ((req.ordertype_msg.type == OrderType.TYP_LIMIT)
                or (req.ordertype_msg.type == OrderType.TYP_STOP)):
            tmp = {
                "price": req.units,
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": ORDER_INST_DICT[req.instrument],
            "units": req.units,
            "positionFill": "DEFAULT",
            "takeProfitOnFill": {
                "timeInForce": "GTC",
                "price": req.take_profit_price
            },
            "stopLossOnFill": {
                "timeInForce": "GTC",
                "price": req.stop_loss_price
            },
        }
        data_order.update(tmp)

        ep = OrderCreate(accountID=self.__account_number, data=data)

        try:
            apirsp = self.__api.request(ep)
        except ConnectionError as ce:
            self.__logger.error("%s" % ce)
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        else:
            if "orderFillTransaction" in apirsp.keys():
                rsp.frc_msg.reason_code = frc.REASON_UNSET
            elif "orderCancelTransaction" in apirsp.keys():
                reason = apirsp["orderCancelTransaction"]["reason"]
                if reason == "MARKET_HALTED":
                    rsp.frc_msg.reason_code = frc.REASON_MARKET_HALTED
                else:
                    rsp.frc_msg.reason_code = frc.REASON_OTHERS
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

            import json
            print(json.dumps(apirsp, indent=2))

        return rsp


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        order.destroy_node()
        rclpy.shutdown()
