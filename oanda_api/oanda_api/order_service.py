from typing import TypeVar, Dict
import datetime as dt
import json
from decimal import Decimal, ROUND_HALF_UP
import rclpy
from oandapyV20.endpoints.orders import OrderCreate, OrderDetails, OrderCancel
from oandapyV20.endpoints.trades import TradeDetails, TradeCRCDO, TradeClose
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType, OrderState, TradeState
from api_msgs.msg import FailReasonCode as frc
from oanda_api.service_common import ServiceAbs
from oanda_api.service_common import INST_DICT

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
JsonFmt = TypeVar("JsonFmt")
ApiRsp = TypeVar("ApiRsp")


def inverse_dict(d: Dict[int, str]) -> Dict[str, int]:
    return {v: k for k, v in d.items()}


ORDER_TYP_DICT = {
    OrderType.TYP_MARKET: "MARKET",
    OrderType.TYP_LIMIT: "LIMIT",
    OrderType.TYP_STOP: "STOP",
}

ORDER_TYP_NAME_DICT = inverse_dict(ORDER_TYP_DICT)

ORDER_STS_DICT = {
    "PENDING": OrderState.STS_PENDING,
    "FILLED": OrderState.STS_FILLED,
    "TRIGGERED": OrderState.STS_TRIGGERED,
    "CANCELLED": OrderState.STS_CANCELLED,
}

TRADE_STS_DICT = {
    "OPEN": TradeState.STS_OPEN,
    "STS_CLOSED": TradeState.STS_CLOSED,
    "CLOSE_WHEN_TRADEABLE": TradeState.STS_CLOSE_WHEN_TRADEABLE,
}


class OrderService(ServiceAbs):

    def __init__(self) -> None:
        super().__init__("order_service")

        PRMNM_ACCOUNT_NUMBER = "account_number"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        self._logger.debug("[Param]Account Number:[{}]".format(account_number))

        # Create service server "OrderCreate"
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        callback = self.__on_recv_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)
        # Create service server "TradeDetails"
        srv_type = TradeDetailsSrv
        srv_name = "trade_details"
        callback = self.__on_recv_trade_details
        self.trade_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service server "TradeCRCDO"
        srv_type = TradeCRCDOSrv
        srv_name = "trade_crcdo"
        callback = self.__on_recv_trade_crcdo
        self.trade_crcdo_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service server "TradeClose"
        srv_type = TradeCloseSrv
        srv_name = "trade_close"
        callback = self.__on_recv_trade_close
        self.trade_close_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service server "OrderDetails"
        srv_type = OrderDetailsSrv
        srv_name = "order_details"
        callback = self.__on_recv_order_details
        self.order_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service server "OrderCancel"
        srv_type = OrderCancelSrv
        srv_name = "order_cancel"
        callback = self.__on_recv_order_cancel
        self.order_cancel_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__account_number = account_number

    def __on_recv_order_create(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[order_create]:Start "))
        logger.debug("<Request>")
        logger.debug("  - ordertype_msg.type:[{}]".format(req.ordertype_msg.type))
        logger.debug("  - price:[{}]".format(req.price))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - units:[{}]".format(req.units))
        logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self.__make_data_for_order_create(req)
        ep = OrderCreate(accountID=self.__account_number, data=data)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_create_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - id(Trade or Order):[{}]".format(rsp.id))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_create]:End "))

        return rsp

    def __on_recv_trade_details(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[trade_details]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeDetails(accountID=self.__account_number,
                          tradeID=req.trade_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_details_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - contract_price:[{}]".format(rsp.contract_price))
        logger.debug("  - trade_state_msg.state:[{}]".format(rsp.trade_state_msg.state))
        logger.debug("  - current_units:[{}]".format(rsp.current_units))
        logger.debug("  - realized_pl:[{}]".format(rsp.realized_pl))
        logger.debug("  - unrealized_pl:[{}]".format(rsp.unrealized_pl))
        logger.debug("  - open_time:[{}]".format(rsp.open_time))
        logger.debug("  - profit_order_msg.price:[{}]".format(rsp.profit_order_msg.price))
        logger.debug("  - profit_order_msg.order_state_msg.state:[{}]"
                     .format(rsp.profit_order_msg.order_state_msg.state))
        logger.debug("  - loss_order_msg.price:[{}]".format(rsp.loss_order_msg.price))
        logger.debug("  - loss_order_msg.order_state_msg.state:[{}]"
                     .format(rsp.loss_order_msg.order_state_msg.state))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[trade_details]:End "))

        return rsp

    def __on_recv_trade_crcdo(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[trade_crcdo]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self.__make_data_for_trade_crcdo(req)
        ep = TradeCRCDO(accountID=self.__account_number,
                        tradeID=req.trade_id, data=data)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_crcdo_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - take_profit_price:[{}]".format(rsp.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(rsp.stop_loss_price))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[trade_crcdo]:End "))

        return rsp

    def __on_recv_trade_close(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[trade_close]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeClose(accountID=self.__account_number, tradeID=req.trade_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_close_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - inst_msg.inst_id:[{}]".format(rsp.inst_msg.inst_id))
        logger.debug("  - time:[{}]".format(rsp.time))
        logger.debug("  - units:[{}]".format(rsp.units))
        logger.debug("  - price:[{}]".format(rsp.price))
        logger.debug("  - realized_pl:[{}]".format(rsp.realized_pl))
        logger.debug("  - half_spread_cost:[{}]".format(rsp.half_spread_cost))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[trade_close]:End "))

        return rsp

    def __on_recv_order_details(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[order_details]:Start "))
        logger.debug("<Request>")
        logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderDetails(accountID=self.__account_number,
                          orderID=req.order_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_details_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - ordertype_msg.type:[{}]".format(rsp.ordertype_msg.type))
        logger.debug("  - inst_msg.inst_id:[{}]".format(rsp.inst_msg.inst_id))
        logger.debug("  - units:[{}]".format(rsp.units))
        logger.debug("  - price:[{}]".format(rsp.price))
        logger.debug("  - order_state_msg.state:[{}]".format(rsp.order_state_msg.state))
        logger.debug("  - open_trade_id:[{}]".format(rsp.open_trade_id))
        logger.debug("  - take_profit_pn_fill_price:[{}]".format(rsp.take_profit_pn_fill_price))
        logger.debug("  - stop_loss_on_fill_price:[{}]".format(rsp.stop_loss_on_fill_price))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_details]:End "))

        return rsp

    def __on_recv_order_cancel(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[order_cancel]:Start "))
        logger.debug("<Request>")
        logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderCancel(accountID=self.__account_number,
                         orderID=req.order_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_cancel_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_cancel]:End "))

        return rsp

    def __make_data_for_order_create(self,
                                     req: SrvTypeRequest,
                                     ) -> JsonFmt:
        data = {
            "order": {
                "type": ORDER_TYP_DICT[req.ordertype_msg.type],
            }
        }

        data_order = data["order"]

        min_unit = INST_DICT[req.inst_msg.inst_id].min_unit

        if ((req.ordertype_msg.type == OrderType.TYP_LIMIT)
                or (req.ordertype_msg.type == OrderType.TYP_STOP)):
            tmp = {
                "price": self.__fit_unit(req.price, min_unit),
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": INST_DICT[req.inst_msg.inst_id].name,
            "units": req.units,
            "positionFill": "DEFAULT",
            "takeProfitOnFill": {
                "timeInForce": "GTC",
                "price": self.__fit_unit(req.take_profit_price, min_unit)
            },
            "stopLossOnFill": {
                "timeInForce": "GTC",
                "price": self.__fit_unit(req.stop_loss_price, min_unit)
            },
        }
        data_order.update(tmp)

        return data

    def __make_data_for_trade_crcdo(self,
                                    req: SrvTypeRequest,
                                    ) -> JsonFmt:

        min_unit = INST_DICT[req.inst_msg.inst_id].min_unit
        data = {
            "takeProfit": {
                "price": self.__fit_unit(req.take_profit_price, min_unit),
                "timeInForce": "GTC",
            },
            "stopLoss": {
                "price": self.__fit_unit(req.stop_loss_price, min_unit),
                "timeInForce": "GTC",
            },
        }

        return data

    def __update_order_create_response(self,
                                       apirsp: ApiRsp,
                                       rsp: SrvTypeResponse
                                       ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                data_to = data_oft["tradeOpened"]
                rsp.id = int(data_to["tradeID"])
                rsp.result = True
            elif "orderCancelTransaction" in apirsp.keys():
                reason = apirsp["orderCancelTransaction"]["reason"]
                if reason == "MARKET_HALTED":
                    rsp.frc_msg.reason_code = frc.REASON_MARKET_HALTED
                else:
                    rsp.frc_msg.reason_code = frc.REASON_OTHERS
            elif "orderCreateTransaction" in apirsp.keys():
                data_oct = apirsp["orderCreateTransaction"]
                rsp.id = int(data_oct["id"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_details_response(self,
                                        apirsp: ApiRsp,
                                        rsp: SrvTypeResponse
                                        ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if "trade" in apirsp.keys():
                data_trd = apirsp["trade"]
                rsp.contract_price = float(data_trd["price"])
                rsp.trade_state_msg.state = TRADE_STS_DICT[data_trd["state"]]
                rsp.current_units = int(data_trd["currentUnits"])
                rsp.realized_pl = float(data_trd["realizedPL"])
                if "unrealizedPL" in data_trd.keys():
                    rsp.unrealized_pl = float(data_trd["unrealizedPL"])
                rsp.open_time = data_trd["openTime"]
                data_tpo = data_trd["takeProfitOrder"]
                rsp.profit_order_msg.price = float(data_tpo["price"])
                rsp.profit_order_msg.order_state_msg.state = ORDER_STS_DICT[data_tpo["state"]]
                data_slo = data_trd["stopLossOrder"]
                rsp.loss_order_msg.price = float(data_slo["price"])
                rsp.loss_order_msg.order_state_msg.state = ORDER_STS_DICT[data_slo["state"]]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_crcdo_response(self,
                                      apirsp: ApiRsp,
                                      rsp: SrvTypeResponse
                                      ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if (("takeProfitOrderTransaction" in apirsp.keys())
                    and ("stopLossOrderTransaction" in apirsp.keys())):
                data_tpot = apirsp["takeProfitOrderTransaction"]
                rsp.take_profit_price = float(data_tpot["price"])
                data_slot = apirsp["stopLossOrderTransaction"]
                rsp.stop_loss_price = float(data_slot["price"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_trade_close_response(self,
                                      apirsp: ApiRsp,
                                      rsp: SrvTypeResponse
                                      ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                inst_id = self.get_inst_id_from_name(data_oft["instrument"])
                rsp.inst_msg.inst_id = inst_id
                rsp.time = data_oft["time"]
                data_tc = data_oft["tradesClosed"][0]
                rsp.units = int(data_tc["units"])
                rsp.price = float(data_tc["price"])
                rsp.realized_pl = float(data_tc["realizedPL"])
                rsp.half_spread_cost = float(data_tc["halfSpreadCost"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_order_details_response(self,
                                        apirsp: ApiRsp,
                                        rsp: SrvTypeResponse
                                        ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if "order" in apirsp.keys():
                data_ord = apirsp["order"]
                rsp.ordertype_msg.type = ORDER_TYP_NAME_DICT[data_ord["type"]]
                inst_id = self.get_inst_id_from_name(data_ord["instrument"])
                rsp.inst_msg.inst_id = inst_id
                rsp.units = int(data_ord["units"])
                rsp.price = float(data_ord["price"])
                rsp.order_state_msg.state = ORDER_STS_DICT[data_ord["state"]]
                if "takeProfitOnFill" in data_ord.keys():
                    data_tpof = data_ord["takeProfitOnFill"]
                    rsp.take_profit_pn_fill_price = float(data_tpof["price"])
                if "stopLossOnFill" in data_ord.keys():
                    data_tpof = data_ord["stopLossOnFill"]
                    rsp.stop_loss_on_fill_price = float(data_tpof["price"])
                if rsp.order_state_msg.state == OrderState.STS_FILLED:
                    rsp.open_trade_id = data_tpof = data_ord["tradeOpenedID"]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __update_order_cancel_response(self,
                                       apirsp: ApiRsp,
                                       rsp: SrvTypeResponse
                                       ) -> SrvTypeResponse:
        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:

            self._logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            if "orderCancelTransaction" in apirsp.keys():
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __fit_unit(self, value: float, min_unit: str):
        tmp = Decimal(str(value)).quantize(Decimal(min_unit),
                                           rounding=ROUND_HALF_UP)
        return str(tmp)

    def get_inst_id_from_name(self, name):

        inst_id = -1
        for k, v in INST_DICT.items():
            if v.name == name:
                inst_id = k
                break
        return inst_id


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()

    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        pass

    order.destroy_node()
    rclpy.shutdown()
