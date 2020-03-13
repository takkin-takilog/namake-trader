from requests.exceptions import ConnectionError
import rclpy
from rclpy.node import Node
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType, Instrument
from api_msgs.msg import FailReasonCode as frc
from oandapyV20.endpoints.orders import OrderCreate, OrderDetails, OrderCancel
from oandapyV20.endpoints.trades import TradeDetails, TradeCRCDO, TradeClose
from oandapyV20 import API
from oandapyV20.exceptions import V20Error

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

        # Create service "OrderCreate"
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        callback = self.__on_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)
        # Create service "TradeDetails"
        srv_type = TradeDetailsSrv
        srv_name = "trade_details"
        callback = self.__on_trade_details
        self.trade_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service "TradeCRCDO"
        srv_type = TradeCRCDOSrv
        srv_name = "trade_crcdo"
        callback = self.__on_trade_crcdo
        self.trade_crcdo_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service "TradeClose"
        srv_type = TradeCloseSrv
        srv_name = "trade_close"
        callback = self.__on_trade_close
        self.trade_close_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service "OrderDetailsSrv"
        srv_type = OrderDetailsSrv
        srv_name = "order_details"
        callback = self.__on_order_details
        self.order_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service "OrderCancel"
        srv_type = OrderCancelSrv
        srv_name = "order_cancel"
        callback = self.__on_order_cancel
        self.order_cancel_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__api = API(access_token=access_token)

        self.__account_number = account_number

    def __request_api(self, endpoint, rsp):

        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self.__api.request(endpoint)
        except ConnectionError as err:
            self.__logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except V20Error as err:
            self.__logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR

        return rsp, apirsp

    def __on_order_create(self, req, rsp):

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
            "instrument": ORDER_INST_DICT[req.inst_msg.instrument_id],
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

        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            import json
            print(json.dumps(apirsp, indent=2))

            rsp = self.__update_order_create_response(apirsp, rsp)

            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __on_trade_details(self, req, rsp):

        ep = TradeDetails(accountID=self.__account_number,
                          tradeID=req.trade_id)
        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            import json
            print(json.dumps(apirsp, indent=2))
            #rsp = self.__update_order_create_response(apirsp, rsp)
            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __on_trade_crcdo(self, req, rsp):

        ep = 0
        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            import json
            print(json.dumps(apirsp, indent=2))
            #rsp = self.__update_order_create_response(apirsp, rsp)
            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __on_trade_close(self, req, rsp):

        ep = 0
        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            import json
            print(json.dumps(apirsp, indent=2))
            #rsp = self.__update_order_create_response(apirsp, rsp)
            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __on_order_details(self, req, rsp):

        ep = 0
        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            import json
            print(json.dumps(apirsp, indent=2))
            #rsp = self.__update_order_create_response(apirsp, rsp)
            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __on_order_cancel(self, req, rsp):

        ep = 0
        rsp, apirsp = self.__request_api(ep, rsp)

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            import json
            print(json.dumps(apirsp, indent=2))
            #rsp = self.__update_order_create_response(apirsp, rsp)
            rsp.result = True
        else:
            rsp.result = False

        return rsp

    def __update_order_create_response(self, apirsp, rsp):

        if "orderFillTransaction" in apirsp.keys():
            data_oft = apirsp["orderFillTransaction"]
            data_to = data_oft["tradeOpened"]
            rsp.trade_id = int(data_to["tradeID"])
            rsp.contract_price = float(data_to["price"])
            rsp.units = int(data_to["units"])
            rsp.half_spread_cost = float(data_to["halfSpreadCost"])
            rsp.time = data_oft["time"]
            rsp.frc_msg.reason_code = frc.REASON_UNSET
        elif "orderCancelTransaction" in apirsp.keys():
            reason = apirsp["orderCancelTransaction"]["reason"]
            if reason == "MARKET_HALTED":
                rsp.frc_msg.reason_code = frc.REASON_MARKET_HALTED
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_details_response(self, apirsp, rsp):

        return rsp


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        order.destroy_node()
        rclpy.shutdown()
