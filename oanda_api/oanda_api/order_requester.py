from typing import TypeVar
import requests
import datetime as dt
import ast
import json
from decimal import Decimal, ROUND_HALF_UP
from requests.exceptions import ConnectionError, ReadTimeout
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from oandapyV20 import API
from oandapyV20.endpoints.orders import OrderCreate, OrderDetails, OrderCancel
from oandapyV20.endpoints.trades import TradeDetails, TradeCRCDO, TradeClose
from oandapyV20.exceptions import V20Error
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType, OrderState, TradeState
from api_msgs.msg import FailReasonCode as frc
from .constant import ADD_CIPHERS
from .parameter import InstParam
from .dataclass import RosParam
from . import utils as utl
from . import ros_utils as rosutl

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
JsonFmt = TypeVar("JsonFmt")
ApiRsp = TypeVar("ApiRsp")
EndPoint = TypeVar("EndPoint")


_ORDER_TYP_DICT = {
    OrderType.TYP_MARKET: "MARKET",
    OrderType.TYP_LIMIT: "LIMIT",
    OrderType.TYP_STOP: "STOP",
}

_ORDER_TYP_NAME_DICT = utl.inverse_dict(_ORDER_TYP_DICT)

_ORDER_STS_DICT = {
    "PENDING": OrderState.STS_PENDING,
    "FILLED": OrderState.STS_FILLED,
    "TRIGGERED": OrderState.STS_TRIGGERED,
    "CANCELLED": OrderState.STS_CANCELLED,
}

_TRADE_STS_DICT = {
    "OPEN": TradeState.STS_OPEN,
    "CLOSED": TradeState.STS_CLOSED,
    "CLOSE_WHEN_TRADEABLE": TradeState.STS_CLOSE_WHEN_TRADEABLE,
}


class OrderRequester(Node):

    def __init__(self) -> None:
        super().__init__("order_register")

        # --------------- Set logger lebel ---------------
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = logger

        # --------------- Declare ROS parameter ---------------
        self._rosprm_use_env_live = RosParam("use_env_live",
                                             Parameter.Type.BOOL)
        self._rosprm_pra_account_number = RosParam("env_practice.account_number",
                                                   Parameter.Type.STRING)
        self._rosprm_pra_access_token = RosParam("env_practice.access_token",
                                                 Parameter.Type.STRING)
        self._rosprm_liv_account_number = RosParam("env_live.account_number",
                                                   Parameter.Type.STRING)
        self._rosprm_liv_access_token = RosParam("env_live.access_token",
                                                 Parameter.Type.STRING)
        self._rosprm_connection_timeout = RosParam("connection_timeout",
                                                   Parameter.Type.INTEGER)

        rosutl.set_parameters(self, self._rosprm_use_env_live)
        rosutl.set_parameters(self, self._rosprm_pra_account_number)
        rosutl.set_parameters(self, self._rosprm_pra_access_token)
        rosutl.set_parameters(self, self._rosprm_liv_account_number)
        rosutl.set_parameters(self, self._rosprm_liv_access_token)
        rosutl.set_parameters(self, self._rosprm_connection_timeout)

        # --------------- Initialize instance variable ---------------
        if self._rosprm_use_env_live.value:
            environment = "live"
            access_token = self._rosprm_liv_access_token.value
            self._ACCOUNT_NUMBER = self._rosprm_liv_account_number.value
        else:
            environment = "practice"
            access_token = self._rosprm_pra_access_token.value
            self._ACCOUNT_NUMBER = self._rosprm_pra_account_number.value

        if self._rosprm_connection_timeout.value <= 0:
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm_connection_timeout.value}

        self._api = API(access_token=access_token,
                        environment=environment,
                        request_params=request_params)

        # --------------- Create ROS Communication ---------------
        # Create service server "OrderCreate"
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        callback = self._on_recv_order_create
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)
        # Create service server "TradeDetails"
        srv_type = TradeDetailsSrv
        srv_name = "trade_details"
        callback = self._on_recv_trade_details
        self.trade_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service server "TradeCRCDO"
        srv_type = TradeCRCDOSrv
        srv_name = "trade_crcdo"
        callback = self._on_recv_trade_crcdo
        self.trade_crcdo_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service server "TradeClose"
        srv_type = TradeCloseSrv
        srv_name = "trade_close"
        callback = self._on_recv_trade_close
        self.trade_close_srv = self.create_service(srv_type,
                                                   srv_name,
                                                   callback)
        # Create service server "OrderDetails"
        srv_type = OrderDetailsSrv
        srv_name = "order_details"
        callback = self._on_recv_order_details
        self.order_details_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)
        # Create service server "OrderCancel"
        srv_type = OrderCancelSrv
        srv_name = "order_cancel"
        callback = self._on_recv_order_cancel
        self.order_cancel_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

    def _on_recv_order_create(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[order_create]:Start "))
        logger.debug("<Request>")
        logger.debug("  - ordertype_msg.type:[{}]".format(req.ordertype_msg.type))
        logger.debug("  - price:[{}]".format(req.price))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - units:[{}]".format(req.units))
        logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self._generate_order_create_data(req)
        ep = OrderCreate(accountID=self._ACCOUNT_NUMBER, data=data)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" V20Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
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

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("  - id(Trade or Order):[{}]".format(rsp.id))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_create]:End "))

        return rsp

    def _on_recv_trade_details(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[trade_details]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeDetails(accountID=self._ACCOUNT_NUMBER,
                          tradeID=req.trade_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" V20Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "trade" in apirsp.keys():
                data_trd = apirsp["trade"]
                rsp.contract_price = float(data_trd["price"])
                rsp.trade_state_msg.state = _TRADE_STS_DICT[data_trd["state"]]
                rsp.current_units = int(data_trd["currentUnits"])
                rsp.realized_pl = float(data_trd["realizedPL"])
                if "unrealizedPL" in data_trd.keys():
                    rsp.unrealized_pl = float(data_trd["unrealizedPL"])
                rsp.open_time = data_trd["openTime"]
                data_tpo = data_trd["takeProfitOrder"]
                rsp.profit_order_msg.price = float(data_tpo["price"])
                rsp.profit_order_msg.order_state_msg.state = _ORDER_STS_DICT[data_tpo["state"]]
                data_slo = data_trd["stopLossOrder"]
                rsp.loss_order_msg.price = float(data_slo["price"])
                rsp.loss_order_msg.order_state_msg.state = _ORDER_STS_DICT[data_slo["state"]]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

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

    def _on_recv_trade_crcdo(self,
                             req: SrvTypeRequest,
                             rsp: SrvTypeResponse
                             ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[trade_crcdo]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self._generate_trade_crcdo_data(req)
        ep = TradeCRCDO(accountID=self._ACCOUNT_NUMBER,
                        tradeID=req.trade_id, data=data)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" V20Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if (("takeProfitOrderTransaction" in apirsp.keys())
                    and ("stopLossOrderTransaction" in apirsp.keys())):
                data_tpot = apirsp["takeProfitOrderTransaction"]
                rsp.take_profit_price = float(data_tpot["price"])
                data_slot = apirsp["stopLossOrderTransaction"]
                rsp.stop_loss_price = float(data_slot["price"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

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

    def _on_recv_trade_close(self,
                             req: SrvTypeRequest,
                             rsp: SrvTypeResponse
                             ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[trade_close]:Start "))
        logger.debug("<Request>")
        logger.debug("  - trade_id:[{}]".format(req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeClose(accountID=self._ACCOUNT_NUMBER, tradeID=req.trade_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
            try:
                rspdic = ast.literal_eval(err.msg)
            except ValueError:
                pass
            else:
                if isinstance(rspdic, dict):
                    self.logger.debug("{}".format(json.dumps(rspdic, indent=2)))
                    if "orderRejectTransaction" in rspdic.keys():
                        rjc = rspdic["orderRejectTransaction"]
                        if "rejectReason" in rjc.keys():
                            if rjc["rejectReason"] == "TRADE_DOESNT_EXIST":
                                rsp.frc_msg.reason_code = frc.REASON_TRADE_DOESNT_EXIST
            if not rsp.frc_msg.reason_code == frc.REASON_TRADE_DOESNT_EXIST:
                self.logger.error("{:!^50}".format(" V20Error "))
                self.logger.error("{}".format(err))
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "orderFillTransaction" in apirsp.keys():
                data_oft = apirsp["orderFillTransaction"]
                inst_id = InstParam.get_member_by_name(data_oft["instrument"]).msg_id
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

    def _on_recv_order_details(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[order_details]:Start "))
        logger.debug("<Request>")
        logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderDetails(accountID=self._ACCOUNT_NUMBER,
                          orderID=req.order_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" V20Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "order" in apirsp.keys():
                data_ord = apirsp["order"]
                rsp.ordertype_msg.type = _ORDER_TYP_NAME_DICT[data_ord["type"]]
                inst_id = InstParam.get_member_by_name(data_ord["instrument"]).msg_id
                rsp.inst_msg.inst_id = inst_id
                rsp.units = int(data_ord["units"])
                rsp.price = float(data_ord["price"])
                rsp.order_state_msg.state = _ORDER_STS_DICT[data_ord["state"]]
                if "takeProfitOnFill" in data_ord.keys():
                    data_tpof = data_ord["takeProfitOnFill"]
                    rsp.take_profit_on_fill_price = float(data_tpof["price"])
                if "stopLossOnFill" in data_ord.keys():
                    data_tpof = data_ord["stopLossOnFill"]
                    rsp.stop_loss_on_fill_price = float(data_tpof["price"])
                if rsp.order_state_msg.state == OrderState.STS_FILLED:
                    rsp.open_trade_id = int(data_ord["tradeOpenedID"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

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
        logger.debug("  - take_profit_on_fill_price:[{}]".format(rsp.take_profit_on_fill_price))
        logger.debug("  - stop_loss_on_fill_price:[{}]".format(rsp.stop_loss_on_fill_price))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_details]:End "))

        return rsp

    def _on_recv_order_cancel(self,
                              req: SrvTypeRequest,
                              rsp: SrvTypeResponse
                              ) -> SrvTypeResponse:
        logger = self.logger

        logger.debug("{:=^50}".format(" Service[order_cancel]:Start "))
        logger.debug("<Request>")
        logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderCancel(accountID=self._ACCOUNT_NUMBER,
                         orderID=req.order_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
            try:
                rspdic = ast.literal_eval(err.msg)
            except ValueError:
                pass
            else:
                if isinstance(rspdic, dict):
                    self.logger.debug("{}".format(json.dumps(rspdic, indent=2)))
                    if "orderCancelRejectTransaction" in rspdic.keys():
                        rjc = rspdic["orderCancelRejectTransaction"]
                        if "rejectReason" in rjc.keys():
                            if rjc["rejectReason"] == "ORDER_DOESNT_EXIST":
                                rsp.frc_msg.reason_code = frc.REASON_ORDER_DOESNT_EXIST
            if not rsp.frc_msg.reason_code == frc.REASON_ORDER_DOESNT_EXIST:
                self.logger.error("{:!^50}".format(" V20Error "))
                self.logger.error("{}".format(err))
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" Connection Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout  Error"))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" Others Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "orderCancelTransaction" in apirsp.keys():
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - result:[{}]".format(rsp.result))
        logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_cancel]:End "))

        return rsp

    def _generate_order_create_data(self,
                                    req: SrvTypeRequest,
                                    ) -> JsonFmt:
        data = {
            "order": {
                "type": _ORDER_TYP_DICT[req.ordertype_msg.type],
            }
        }

        data_order = data["order"]

        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)
        one_pip_str = inst_param.one_pip_str

        if ((req.ordertype_msg.type == OrderType.TYP_LIMIT)
                or (req.ordertype_msg.type == OrderType.TYP_STOP)):

            tmp = {
                "price": self._fit_unit(req.price, one_pip_str),
                "timeInForce": "GTC",
            }
            data_order.update(tmp)

        tmp = {
            "instrument": inst_param.name,
            "units": req.units,
            "positionFill": "DEFAULT",
            "takeProfitOnFill": {
                "timeInForce": "GTC",
                "price": self._fit_unit(req.take_profit_price, one_pip_str)
            },
            "stopLossOnFill": {
                "timeInForce": "GTC",
                "price": self._fit_unit(req.stop_loss_price, one_pip_str)
            },
        }
        data_order.update(tmp)

        return data

    def _generate_trade_crcdo_data(self,
                                   req: SrvTypeRequest,
                                   ) -> JsonFmt:

        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)
        one_pip_str = inst_param.one_pip_str

        data = {
            "takeProfit": {
                "price": self._fit_unit(req.take_profit_price, one_pip_str),
                "timeInForce": "GTC",
            },
            "stopLoss": {
                "price": self._fit_unit(req.stop_loss_price, one_pip_str),
                "timeInForce": "GTC",
            },
        }

        return data

    def _fit_unit(self, value: float, one_pip_str: str):
        tmp = Decimal(str(value)).quantize(Decimal(one_pip_str),
                                           rounding=ROUND_HALF_UP)
        return str(tmp)


def main(args=None):

    requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ADD_CIPHERS

    rclpy.init(args=args)
    order = OrderRequester()

    try:
        rclpy.spin(order)
    except KeyboardInterrupt:
        pass

    order.destroy_node()
    rclpy.shutdown()
