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
from oanda_api.service_common import MIN_UNIT_DICT

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
        PRMNM_INST_NAME_LIST = "inst_name_list"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_INST_NAME_LIST)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        inst_name_list = self.get_parameter(PRMNM_INST_NAME_LIST).value
        self._logger.debug("[Param]Account Number:[%s]" % account_number)
        self._logger.debug("[Param]Instruments:[%s]" % inst_name_list)

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
        self.__inst_name_list = inst_name_list

    def __on_recv_order_create(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[order_create]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- ordertype_msg.type:[%d]" % (req.ordertype_msg.type))
        logger.debug("- price:[%f]" % (req.price))
        logger.debug("- inst_msg.inst_id:[%d]" % (req.inst_msg.inst_id))
        logger.debug("- units:[%d]" % (req.units))
        logger.debug("- take_profit_price:[%f]" % (req.take_profit_price))
        logger.debug("- stop_loss_price:[%f]" % (req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self.__make_data_for_order_create(req)
        ep = OrderCreate(accountID=self.__account_number, data=data)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_create_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("- id(Trade or Order):[%d]" % (rsp.id))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[order_create]:End ==========")

        return rsp

    def __on_recv_trade_details(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[trade_details]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- trade_id:[%d]" % (req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeDetails(accountID=self.__account_number,
                          tradeID=req.trade_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_details_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("- contract_price:[%f]" % (rsp.contract_price))
        logger.debug("- trade_state_msg.state:[%d]" % (rsp.trade_state_msg.state))
        logger.debug("- current_units:[%d]" % (rsp.current_units))
        logger.debug("- realized_pl:[%f]" % (rsp.realized_pl))
        logger.debug("- unrealized_pl:[%f]" % (rsp.unrealized_pl))
        logger.debug("- open_time:[%s]" % (rsp.open_time))
        logger.debug("- profit_order_msg.price:[%f]" % (rsp.profit_order_msg.price))
        logger.debug("- profit_order_msg.order_state_msg.state:[%d]"
                     % (rsp.profit_order_msg.order_state_msg.state))
        logger.debug("- loss_order_msg.price:[%f]" % (rsp.loss_order_msg.price))
        logger.debug("- loss_order_msg.order_state_msg.state:[%d]"
                     % (rsp.loss_order_msg.order_state_msg.state))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[trade_details]:End ==========")

        return rsp

    def __on_recv_trade_crcdo(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[trade_crcdo]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- trade_id:[%d]" % (req.trade_id))
        logger.debug("- inst_msg.inst_id:[%d]" % (req.inst_msg.inst_id))
        logger.debug("- take_profit_price:[%f]" % (req.take_profit_price))
        logger.debug("- stop_loss_price:[%f]" % (req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self.__make_data_for_trade_crcdo(req)
        ep = TradeCRCDO(accountID=self.__account_number,
                        tradeID=req.trade_id, data=data)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_crcdo_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("- take_profit_price:[%f]" % (rsp.take_profit_price))
        logger.debug("- stop_loss_price:[%f]" % (rsp.stop_loss_price))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[trade_crcdo]:End ==========")

        return rsp

    def __on_recv_trade_close(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[trade_close]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- trade_id:[%d]" % (req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeClose(accountID=self.__account_number, tradeID=req.trade_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_trade_close_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("- inst_msg.inst_id:[%d]" % (rsp.inst_msg.inst_id))
        logger.debug("- time:[%s]" % (rsp.time))
        logger.debug("- units:[%d]" % (rsp.units))
        logger.debug("- price:[%f]" % (rsp.price))
        logger.debug("- realized_pl:[%f]" % (rsp.realized_pl))
        logger.debug("- half_spread_cost:[%f]" % (rsp.half_spread_cost))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[trade_close]:End ==========")

        return rsp

    def __on_recv_order_details(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[order_details]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- order_id:[%d]" % (req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderDetails(accountID=self.__account_number,
                          orderID=req.order_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_details_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("- ordertype_msg.type:[%s]" % (rsp.ordertype_msg.type))
        logger.debug("- inst_msg.inst_id:[%d]" % (rsp.inst_msg.inst_id))
        logger.debug("- units:[%d]" % (rsp.units))
        logger.debug("- price:[%f]" % (rsp.price))
        logger.debug("- order_state_msg.state:[%d]" % (rsp.order_state_msg.state))
        logger.debug("- open_trade_id:[%d]" % (rsp.open_trade_id))
        logger.debug("- take_profit_pn_fill_price:[%f]" % (rsp.take_profit_pn_fill_price))
        logger.debug("- stop_loss_on_fill_price:[%f]" % (rsp.stop_loss_on_fill_price))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[order_details]:End ==========")

        return rsp

    def __on_recv_order_cancel(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[order_cancel]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- order_id:[%d]" % (req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderCancel(accountID=self.__account_number,
                         orderID=req.order_id)
        apirsp, rsp = self._request_api(ep, rsp)
        rsp = self.__update_order_cancel_response(apirsp, rsp)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- result:[%r]" % (rsp.result))
        logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[order_cancel]:End ==========")

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

        min_unit = MIN_UNIT_DICT[req.inst_msg.inst_id]

        if ((req.ordertype_msg.type == OrderType.TYP_LIMIT)
                or (req.ordertype_msg.type == OrderType.TYP_STOP)):
            tmp = {
                "price": self.__fit_unit(req.price, min_unit),
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": self.__inst_name_list[req.inst_msg.inst_id],
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

        min_unit = MIN_UNIT_DICT[req.inst_msg.inst_id]
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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                idx = self.__inst_name_list.index(data_oft["instrument"])
                rsp.inst_msg.inst_id = idx
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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

            if "order" in apirsp.keys():
                data_ord = apirsp["order"]
                rsp.ordertype_msg.type = ORDER_TYP_NAME_DICT[data_ord["type"]]
                idx = self.__inst_name_list.index(data_ord["instrument"])
                rsp.inst_msg.inst_id = idx
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

            self._logger.debug("%s" % (json.dumps(apirsp, indent=2)))

            if "orderCancelTransaction" in apirsp.keys():
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

    def __fit_unit(self, value: float, min_unit: str):
        tmp = Decimal(str(value)).quantize(Decimal(min_unit),
                                           rounding=ROUND_HALF_UP)
        return str(tmp)


def main(args=None):
    rclpy.init(args=args)
    order = OrderService()

    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        pass

    order.destroy_node()
    rclpy.shutdown()
