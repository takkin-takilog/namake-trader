from requests.exceptions import ConnectionError
import rclpy
from rclpy.node import Node
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType, OrderState, Instrument
from api_msgs.msg import FailReasonCode as frc
from oandapyV20.endpoints.orders import OrderCreate, OrderDetails, OrderCancel
from oandapyV20.endpoints.trades import TradeDetails, TradeCRCDO, TradeClose
from oandapyV20 import API
from oandapyV20.exceptions import V20Error


def inverse_dict(d):
    return {v: k for k, v in d.items()}


ORDER_TYP_DICT = {
    OrderType.TYP_MARKET: "MARKET",
    OrderType.TYP_LIMIT: "LIMIT",
    OrderType.TYP_STOP: "STOP",
}

ORDER_TYP_NAME_DICT = inverse_dict(ORDER_TYP_DICT)

ORDER_INST_ID_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
}

ORDER_INST_NAME_DICT = inverse_dict(ORDER_INST_ID_DICT)

ORDER_STS_DICT = {
    "PENDING": OrderState.STS_PENDING,
    "FILLED": OrderState.STS_FILLED,
    "TRIGGERED": OrderState.STS_TRIGGERED,
    "CANCELLED": OrderState.STS_CANCELLED,
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
        # Create service "OrderDetails"
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

        return apirsp, rsp

    def __on_order_create(self, req, rsp):

        data = self.__make_data_for_order_create(req)
        ep = OrderCreate(accountID=self.__account_number, data=data)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_order_create_response(apirsp, rsp)

        return rsp

    def __on_trade_details(self, req, rsp):

        ep = TradeDetails(accountID=self.__account_number,
                          tradeID=req.trade_id)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_trade_details_response(apirsp, rsp)

        return rsp

    def __on_trade_crcdo(self, req, rsp):

        data = self.__make_data_for_trade_crcdo(req)
        ep = TradeCRCDO(accountID=self.__account_number,
                        tradeID=req.trade_id, data=data)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_trade_crcdo_response(apirsp, rsp)

        return rsp

    def __on_trade_close(self, req, rsp):

        ep = TradeClose(accountID=self.__account_number, tradeID=req.trade_id)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_trade_close_response(apirsp, rsp)

        return rsp

    def __on_order_details(self, req, rsp):

        ep = OrderDetails(accountID=self.__account_number,
                          orderID=req.order_id)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_order_details_response(apirsp, rsp)

        return rsp

    def __on_order_cancel(self, req, rsp):

        ep = OrderCancel(accountID=self.__account_number,
                         orderID=req.order_id)
        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_order_cancel_response(apirsp, rsp)

        return rsp

    def __make_data_for_order_create(self, req):

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
            "instrument": ORDER_INST_ID_DICT[req.inst_msg.instrument_id],
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

        return data

    def __make_data_for_trade_crcdo(self, req):

        data = {
            "takeProfit": {
                "price": req.take_profit_price,
                "timeInForce": "GTC",
            },
            "stopLoss": {
                "price": req.stop_loss_price,
                "timeInForce": "GTC",
            },
        }

        return data

    def __update_order_create_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                data_to = data_oft["tradeOpened"]
                rsp.trade_id = int(data_to["tradeID"])
                rsp.contract_price = float(data_to["price"])
                rsp.units = int(data_to["units"])
                rsp.half_spread_cost = float(data_to["halfSpreadCost"])
                rsp.time = data_oft["time"]
                rsp.result = True
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
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "trade" in apirsp.keys():
                data_trd = apirsp["trade"]
                rsp.current_units = data_trd["currentUnits"]
                rsp.realized_pl = data_trd["realizedPL"]
                rsp.unrealized_pl = data_trd["unrealizedPL"]
                rsp.open_time = data_trd["openTime"]
                data_tpo = data_trd["takeProfitOrder"]
                rsp.profit_order_msg.price = data_tpo["price"]
                rsp.profit_order_msg.order_state_msg.state = ORDER_STS_DICT[data_tpo["state"]]
                data_slo = data_trd["stopLossOrder"]
                rsp.loss_order_msg.price = data_slo["price"]
                rsp.loss_order_msg.order_state_msg.state = ORDER_STS_DICT[data_slo["state"]]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_crcdo_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if (("takeProfitOrderTransaction" in apirsp.keys())
                    and ("stopLossOrderTransaction" in apirsp.keys())):
                data_tpot = apirsp["takeProfitOrderTransaction"]
                rsp.take_profit_price = data_tpot["price"]
                data_slot = apirsp["stopLossOrderTransaction"]
                rsp.stop_loss_price = data_slot["price"]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_close_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                data_oft.inst_msg.instrument_id = ORDER_INST_NAME_DICT(
                    data_oft["instrument"])
                data_oft.time = data_oft["time"]
                data_tc = data_oft["tradesClosed"]
                data_oft.units = data_tc["units"]
                data_oft.price = data_tc["price"]
                data_oft.realized_pl = data_tc["realizedPL"]
                data_oft.half_spread_cost = data_tc["halfSpreadCost"]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_order_details_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "order" in apirsp.keys():
                data_ord = apirsp["order"]
                data_ord.ordertype_msg.type = ORDER_TYP_NAME_DICT[data_ord["type"]]
                data_ord.inst_msg.instrument_id = ORDER_INST_NAME_DICT[data_ord["instrument"]]
                data_ord.units = data_ord["units"]
                data_ord.price = data_ord["price"]
                data_ord.order_state_msg.state = ORDER_STS_DICT[data_ord["state"]]
                if "takeProfitOnFill" in data_ord.keys():
                    data_tpof = data_ord["takeProfitOnFill"]
                    data_ord.take_profit_pn_fill_price = data_tpof["price"]
                if "stopLossOnFill" in data_ord.keys():
                    data_tpof = data_ord["stopLossOnFill"]
                    data_ord.stop_loss_on_fill_price = data_tpof["price"]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_order_cancel_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "orderCancelTransaction" in apirsp.keys():
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()
    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        order.destroy_node()
        rclpy.shutdown()
