from typing import TypeVar
import requests
import traceback
import ast
import json
import datetime as dt
from decimal import Decimal, ROUND_HALF_UP
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from oandapyV20 import API
import oandapyV20.endpoints.instruments as instruments
import oandapyV20.endpoints.accounts as accounts
from oandapyV20.endpoints.pricing import PricingInfo
from oandapyV20.endpoints.orders import OrderCreate, OrderDetails, OrderCancel
from oandapyV20.endpoints.trades import TradeDetails, TradeCRCDO, TradeClose
from oandapyV20.exceptions import V20Error
from api_server_msgs.msg import Candle, OrderType, OrderState, TradeState, PriceBucket
from api_server_msgs.msg import FailReasonCode as frc
from api_server_msgs.srv import (
    OrderCreateSrv,
    TradeDetailsSrv,
    TradeCRCDOSrv,
    TradeCloseSrv,
    OrderDetailsSrv,
    OrderCancelSrv,
    CandlesQuerySrv,
    AccountQuerySrv,
    PricingQuerySrv,
)
from .constant import FMT_DTTM_API, FMT_YMDHMS, ADD_CIPHERS
from .parameter import InstParam, GranParam
from .dataclass import RosParam
from . import utils as utl
from . import ros_utils as rosutl

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")


class ApiServer(Node):
    """
    Api server class.
    """

    def __init__(self) -> None:
        super().__init__("api_server")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()

        # --------------- Define Constant value ---------------
        self._MAX_SIZE = 4999
        # self._MAX_SIZE = 5  # For test

        # --------------- Initialize ROS parameter ---------------
        self._rosprm_use_env_live = RosParam("use_env_live", Parameter.Type.BOOL)
        self._rosprm_pra_account_number = RosParam(
            "env_practice.account_number", Parameter.Type.STRING
        )
        self._rosprm_pra_access_token = RosParam(
            "env_practice.access_token", Parameter.Type.STRING
        )
        self._rosprm_liv_account_number = RosParam(
            "env_live.account_number", Parameter.Type.STRING
        )
        self._rosprm_liv_access_token = RosParam(
            "env_live.access_token", Parameter.Type.STRING
        )
        self._rosprm_connection_timeout = RosParam(
            "connection_timeout", Parameter.Type.INTEGER
        )

        rosutl.set_parameters(self, self._rosprm_use_env_live)
        rosutl.set_parameters(self, self._rosprm_pra_account_number)
        rosutl.set_parameters(self, self._rosprm_pra_access_token)
        rosutl.set_parameters(self, self._rosprm_liv_account_number)
        rosutl.set_parameters(self, self._rosprm_liv_access_token)
        rosutl.set_parameters(self, self._rosprm_connection_timeout)

        # --------------- Initialize dictionary ---------------
        self._ordertyp_dict = {
            OrderType.TYP_MARKET: "MARKET",
            OrderType.TYP_LIMIT: "LIMIT",
            OrderType.TYP_STOP: "STOP",
        }

        self._ordertyp_name_dict = utl.inverse_dict(self._ordertyp_dict)

        self._ordersts_dict = {
            "PENDING": OrderState.STS_PENDING,
            "FILLED": OrderState.STS_FILLED,
            "TRIGGERED": OrderState.STS_TRIGGERED,
            "CANCELLED": OrderState.STS_CANCELLED,
        }

        self._tradests_dict = {
            "OPEN": TradeState.STS_OPEN,
            "CLOSED": TradeState.STS_CLOSED,
            "CLOSE_WHEN_TRADEABLE": TradeState.STS_CLOSE_WHEN_TRADEABLE,
        }

        # --------------- Create oandapyV20 api ---------------
        if self._rosprm_use_env_live.value:
            environment = "live"
            access_token = self._rosprm_liv_access_token.value
            account_number = self._rosprm_liv_account_number.value
        else:
            environment = "practice"
            access_token = self._rosprm_pra_access_token.value
            account_number = self._rosprm_pra_account_number.value
        self._ACCOUNT_NUMBER = account_number

        if self._rosprm_connection_timeout.value <= 0:
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm_connection_timeout.value}

        self._api = API(
            access_token=access_token,
            environment=environment,
            request_params=request_params,
        )
        self._accsum = accounts.AccountSummary(account_number)

        # --------------- Initialize ROS callback group ---------------
        self._cb_grp_reent = ReentrantCallbackGroup()

        # --------------- Create ROS Communication ---------------
        # Create service server "OrderCreate"
        self.order_create_srv = self.create_service(
            OrderCreateSrv,
            "order_create",
            self._on_recv_order_create,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "TradeDetails"
        self.trade_details_srv = self.create_service(
            TradeDetailsSrv,
            "trade_details",
            self._on_recv_trade_details,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "TradeCRCDO"
        self.trade_crcdo_srv = self.create_service(
            TradeCRCDOSrv,
            "trade_crcdo",
            self._on_recv_trade_crcdo,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "TradeClose"
        self.trade_close_srv = self.create_service(
            TradeCloseSrv,
            "trade_close",
            self._on_recv_trade_close,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "OrderDetails"
        self.order_details_srv = self.create_service(
            OrderDetailsSrv,
            "order_details",
            self._on_recv_order_details,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "OrderCancel"
        self.order_cancel_srv = self.create_service(
            OrderCancelSrv,
            "order_cancel",
            self._on_recv_order_cancel,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "CandlesQuery"
        self._cdlque_srv = self.create_service(
            CandlesQuerySrv,
            "candles_query",
            self._on_recv_candles_query,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "AccountQuery"
        self._accque_srv = self.create_service(
            AccountQuerySrv,
            "account_query",
            self._on_recv_account_query,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "PricingQuery"
        self._prcque_srv = self.create_service(
            PricingQuerySrv,
            "pricing_query",
            self._on_recv_pricing_query,
            callback_group=self._cb_grp_reent,
        )

    def _on_recv_order_create(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[order_create]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - ordertype_msg.type:[{}]".format(req.ordertype_msg.type))
        self.logger.debug("  - price:[{}]".format(req.price))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - units:[{}]".format(req.units))
        self.logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self._generate_order_create_data(req)
        ep = OrderCreate(accountID=self._ACCOUNT_NUMBER, data=data)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
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
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - id(Trade or Order):[{}]".format(rsp.id))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[order_create]:End "))

        return rsp

    def _on_recv_trade_details(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[trade_details]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - trade_id:[{}]".format(req.trade_id))
        dbg_tm_start = dt.datetime.now()

        ep = TradeDetails(accountID=self._ACCOUNT_NUMBER, tradeID=req.trade_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "trade" in apirsp.keys():
                data_trd = apirsp["trade"]
                rsp.contract_price = float(data_trd["price"])
                rsp.trade_state_msg.state = self._tradests_dict[data_trd["state"]]
                rsp.current_units = int(data_trd["currentUnits"])
                rsp.realized_pl = float(data_trd["realizedPL"])
                if "unrealizedPL" in data_trd.keys():
                    rsp.unrealized_pl = float(data_trd["unrealizedPL"])
                rsp.open_time = data_trd["openTime"]
                data_tpo = data_trd["takeProfitOrder"]
                rsp.profit_order_msg.price = float(data_tpo["price"])
                rsp.profit_order_msg.order_state_msg.state = self._ordersts_dict[
                    data_tpo["state"]
                ]
                data_slo = data_trd["stopLossOrder"]
                rsp.loss_order_msg.price = float(data_slo["price"])
                rsp.loss_order_msg.order_state_msg.state = self._ordersts_dict[
                    data_slo["state"]
                ]
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - contract_price:[{}]".format(rsp.contract_price))
        self.logger.debug(
            "  - trade_state_msg.state:[{}]".format(rsp.trade_state_msg.state)
        )
        self.logger.debug("  - current_units:[{}]".format(rsp.current_units))
        self.logger.debug("  - realized_pl:[{}]".format(rsp.realized_pl))
        self.logger.debug("  - unrealized_pl:[{}]".format(rsp.unrealized_pl))
        self.logger.debug("  - open_time:[{}]".format(rsp.open_time))
        self.logger.debug(
            "  - profit_order_msg.price:[{}]".format(rsp.profit_order_msg.price)
        )
        self.logger.debug(
            "  - profit_order_msg.order_state_msg.state:[{}]".format(
                rsp.profit_order_msg.order_state_msg.state
            )
        )
        self.logger.debug(
            "  - loss_order_msg.price:[{}]".format(rsp.loss_order_msg.price)
        )
        self.logger.debug(
            "  - loss_order_msg.order_state_msg.state:[{}]".format(
                rsp.loss_order_msg.order_state_msg.state
            )
        )
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[trade_details]:End "))

        return rsp

    def _on_recv_trade_crcdo(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[trade_crcdo]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - trade_id:[{}]".format(req.trade_id))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        dbg_tm_start = dt.datetime.now()

        data = self._generate_trade_crcdo_data(req)
        ep = TradeCRCDO(accountID=self._ACCOUNT_NUMBER, tradeID=req.trade_id, data=data)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if ("takeProfitOrderTransaction" in apirsp.keys()) and (
                "stopLossOrderTransaction" in apirsp.keys()
            ):
                data_tpot = apirsp["takeProfitOrderTransaction"]
                rsp.take_profit_price = float(data_tpot["price"])
                data_slot = apirsp["stopLossOrderTransaction"]
                rsp.stop_loss_price = float(data_slot["price"])
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - take_profit_price:[{}]".format(rsp.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(rsp.stop_loss_price))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[trade_crcdo]:End "))

        return rsp

    def _on_recv_trade_close(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[trade_close]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - trade_id:[{}]".format(req.trade_id))
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
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
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
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(rsp.inst_msg.inst_id))
        self.logger.debug("  - time:[{}]".format(rsp.time))
        self.logger.debug("  - units:[{}]".format(rsp.units))
        self.logger.debug("  - price:[{}]".format(rsp.price))
        self.logger.debug("  - realized_pl:[{}]".format(rsp.realized_pl))
        self.logger.debug("  - half_spread_cost:[{}]".format(rsp.half_spread_cost))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[trade_close]:End "))

        return rsp

    def _on_recv_order_details(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[order_details]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderDetails(accountID=self._ACCOUNT_NUMBER, orderID=req.order_id)
        rsp.result = False
        apirsp = None
        try:
            apirsp = self._api.request(ep)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "order" in apirsp.keys():
                data_ord = apirsp["order"]
                rsp.ordertype_msg.type = self._ordertyp_name_dict[data_ord["type"]]
                inst_id = InstParam.get_member_by_name(data_ord["instrument"]).msg_id
                rsp.inst_msg.inst_id = inst_id
                rsp.units = int(data_ord["units"])
                rsp.price = float(data_ord["price"])
                rsp.order_state_msg.state = self._ordersts_dict[data_ord["state"]]
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
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - ordertype_msg.type:[{}]".format(rsp.ordertype_msg.type))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(rsp.inst_msg.inst_id))
        self.logger.debug("  - units:[{}]".format(rsp.units))
        self.logger.debug("  - price:[{}]".format(rsp.price))
        self.logger.debug(
            "  - order_state_msg.state:[{}]".format(rsp.order_state_msg.state)
        )
        self.logger.debug("  - open_trade_id:[{}]".format(rsp.open_trade_id))
        self.logger.debug(
            "  - take_profit_on_fill_price:[{}]".format(rsp.take_profit_on_fill_price)
        )
        self.logger.debug(
            "  - stop_loss_on_fill_price:[{}]".format(rsp.stop_loss_on_fill_price)
        )
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[order_details]:End "))

        return rsp

    def _on_recv_order_cancel(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[order_cancel]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - order_id:[{}]".format(req.order_id))
        dbg_tm_start = dt.datetime.now()

        ep = OrderCancel(accountID=self._ACCOUNT_NUMBER, orderID=req.order_id)
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
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            self.logger.debug("{}".format(json.dumps(apirsp, indent=2)))

            rsp.frc_msg.reason_code = frc.REASON_UNSET
            if "orderCancelTransaction" in apirsp.keys():
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[order_cancel]:End "))

        return rsp

    def _on_recv_candles_query(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[candles_query]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_msg.gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - dt_start:[{}]".format(req.dt_from))
        self.logger.debug("  - dt_end:[{}]".format(req.dt_to))

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET
        rsp.cndl_msg_list = []

        dt_start = dt.datetime.strptime(req.dt_from, FMT_YMDHMS)
        dt_end = dt.datetime.strptime(req.dt_to, FMT_YMDHMS)
        dt_now = dt.datetime.now()

        # ---------- Validate request argument ----------
        if (dt_end < dt_start) or (dt_now < dt_start):
            rsp.frc_msg.reason_code = frc.REASON_ARG_ERR
            dbg_tm_end = dt.datetime.now()
            self.logger.error("{:!^50}".format(" Argument Error "))
            self.logger.error("  - dt_end:[{}]".format(dt_end))
            self.logger.error("  - dt_start:[{}]".format(dt_start))
            self.logger.error("  - dt_now:[{}]".format(dt_now))

            self.logger.debug("<Response>")
            self.logger.debug("  - result:[{}]".format(rsp.result))
            self.logger.debug(
                "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
            )
            self.logger.debug(
                "  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list))
            )
            self.logger.debug("[Performance]")
            self.logger.debug(
                "  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start)
            )
            self.logger.debug("{:=^50}".format(" Service[candles_query]:End "))
            return rsp

        gran_param = GranParam.get_member_by_msgid(req.gran_msg.gran_id)
        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)

        minunit = gran_param.timedelta
        dt_end += minunit

        if dt_now < dt_end:
            dt_end = dt_now
        if dt_now - minunit < dt_start:
            dt_start = dt_now - dt.timedelta(seconds=1)

        gran = gran_param.name
        inst = inst_param.name
        dt_jst_next = dt_start
        dt_jst_from = dt_start
        cndl_msg_all_list: list[Candle] = []

        while dt_jst_next < dt_end:
            dt_jst_next += minunit * self._MAX_SIZE
            dt_jst_to = dt_end if dt_end < dt_jst_next else dt_jst_next

            self.logger.debug("{:-^40}".format(" Service[candles_query]:fetch "))
            self.logger.debug("  - from:[{}]".format(dt_jst_from))
            self.logger.debug("  - to:  [{}]".format(dt_jst_to))

            dt_utc_from = utl.convert_from_jst_to_utc(dt_jst_from)
            dt_utc_to = utl.convert_from_jst_to_utc(dt_jst_to)
            params = {
                "from": dt_utc_from.strftime(FMT_DTTM_API),
                "to": dt_utc_to.strftime(FMT_DTTM_API),
                "granularity": gran,
                "price": "AB",
            }

            ep = instruments.InstrumentsCandles(instrument=inst, params=params)
            try:
                apirsp = self._api.request(ep)
            except V20Error as err:
                self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
                rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
                break
            except requests.exceptions.ConnectionError as err:
                self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
                rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
                break
            except requests.exceptions.Timeout as err:
                self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
                rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
                break
            except requests.exceptions.RequestException as err:
                self.logger.error("{:!^50}".format(" HTTP-Request Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
                rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
                break
            except BaseException as err:
                self.logger.error("{:!^50}".format(" Unexpected Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
                rsp.frc_msg.reason_code = frc.REASON_OTHERS
                break

            cndl_msg_one_list = []
            if "candles" in apirsp.keys() and apirsp["candles"]:
                for raw in apirsp["candles"]:
                    msg = Candle()
                    msg.ask_o = float(raw["ask"]["o"])
                    msg.ask_h = float(raw["ask"]["h"])
                    msg.ask_l = float(raw["ask"]["l"])
                    msg.ask_c = float(raw["ask"]["c"])
                    msg.bid_o = float(raw["bid"]["o"])
                    msg.bid_h = float(raw["bid"]["h"])
                    msg.bid_l = float(raw["bid"]["l"])
                    msg.bid_c = float(raw["bid"]["c"])
                    utc_dt = dt.datetime.strptime(raw["time"], FMT_DTTM_API)
                    jst_dt = utl.convert_from_utc_to_jst(utc_dt)
                    msg.time = jst_dt.strftime(FMT_YMDHMS)
                    msg.is_complete = raw["complete"]
                    cndl_msg_one_list.append(msg)

            if cndl_msg_all_list and cndl_msg_one_list:
                if cndl_msg_all_list[-1].time == cndl_msg_one_list[0].time:
                    del cndl_msg_one_list[0]
            cndl_msg_all_list.extend(cndl_msg_one_list)

            dt_jst_from = dt_jst_to

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            rsp.result = True
            if cndl_msg_all_list:
                rsp.cndl_msg_list = cndl_msg_all_list
            else:
                rsp.frc_msg.reason_code = frc.REASON_DATA_ZERO

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug(
            "  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list))
        )
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_query]:End "))

        return rsp

    def _on_recv_account_query(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse  # pylint: disable=W0613
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[account_query]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - None")

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self._api.request(self._accsum)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            rsp.result = True
            acc = apirsp["account"]
            rsp.margin_rate = float(acc["marginRate"])
            rsp.balance = int(float(acc["balance"]))
            rsp.open_trade_count = int(acc["openTradeCount"])
            rsp.open_position_count = int(acc["openPositionCount"])
            rsp.pending_order_count = int(acc["pendingOrderCount"])
            rsp.pl = int(float(acc["pl"]))
            rsp.resettable_pl = int(float(acc["resettablePL"]))
            rsp.financing = int(float(acc["financing"]))
            rsp.unrealized_pl = int(float(acc["unrealizedPL"]))
            rsp.nav = int(float(acc["NAV"]))
            rsp.margin_used = int(float(acc["marginUsed"]))
            rsp.margin_available = int(float(acc["marginAvailable"]))
            rsp.position_value = int(float(acc["positionValue"]))
            rsp.margin_closeout_percent = float(acc["marginCloseoutPercent"]) * 100

        dbg_tm_end = dt.datetime.now()

        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - margin_rate:[{}]".format(rsp.margin_rate))
        self.logger.debug("  - balance:[{}]".format(rsp.balance))
        self.logger.debug("  - open_trade_count:[{}]".format(rsp.open_trade_count))
        self.logger.debug(
            "  - open_position_count:[{}]".format(rsp.open_position_count)
        )
        self.logger.debug(
            "  - pending_order_count:[{}]".format(rsp.pending_order_count)
        )
        self.logger.debug("  - pl:[{}]".format(rsp.pl))
        self.logger.debug("  - resettable_pl:[{}]".format(rsp.resettable_pl))
        self.logger.debug("  - financing:[{}]".format(rsp.financing))
        self.logger.debug("  - unrealized_pl:[{}]".format(rsp.unrealized_pl))
        self.logger.debug("  - nav:[{}]".format(rsp.nav))
        self.logger.debug("  - margin_used:[{}]".format(rsp.margin_used))
        self.logger.debug("  - margin_available:[{}]".format(rsp.margin_available))
        self.logger.debug("  - position_value:[{}]".format(rsp.position_value))
        self.logger.debug(
            "  - margin_closeout(%):[{}]".format(rsp.margin_closeout_percent)
        )
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[account_query]:End "))

        return rsp

    def _on_recv_pricing_query(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[pricing_query]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))

        dbg_tm_start = dt.datetime.now()

        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)
        params = {"instruments": inst_param.name}
        pricing_info = PricingInfo(self._ACCOUNT_NUMBER, params)

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self._api.request(pricing_info)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except requests.exceptions.ConnectionError as err:
            self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.Timeout as err:
            self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except requests.exceptions.RequestException as err:
            self.logger.error("{:!^50}".format(" HTTP-Request Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()
            rsp.frc_msg.reason_code = frc.REASON_OTHERS
        else:
            rsp.result = True
            price = apirsp["prices"][0]
            rsp.time = utl.convert_datetime_jst(price["time"])
            for bid in price["bids"]:
                pb = PriceBucket()
                pb.price = float(bid["price"])
                pb.liquidity = bid["liquidity"]
                rsp.bids.append(pb)
            for ask in price["asks"]:
                pb = PriceBucket()
                pb.price = float(ask["price"])
                pb.liquidity = ask["liquidity"]
                rsp.asks.append(pb)
            rsp.closeout_bid = float(price["closeoutBid"])
            rsp.closeout_ask = float(price["closeoutAsk"])
            rsp.tradeable = price["tradeable"]

        dbg_tm_end = dt.datetime.now()

        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug(
            "  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code)
        )
        self.logger.debug("  - time:[{}]".format(rsp.time))
        for i, bid in enumerate(rsp.bids):
            self.logger.debug("  - bids[{}]:".format(i))
            self.logger.debug("    - price:[{}]".format(bid.price))
            self.logger.debug("    - liquidity:[{}]".format(bid.liquidity))
        for i, ask in enumerate(rsp.asks):
            self.logger.debug("  - asks[{}]:".format(i))
            self.logger.debug("    - price:[{}]".format(ask.price))
            self.logger.debug("    - liquidity:[{}]".format(ask.liquidity))
        self.logger.debug("  - closeout_bid:[{}]".format(rsp.closeout_bid))
        self.logger.debug("  - closeout_ask:[{}]".format(rsp.closeout_ask))
        self.logger.debug("  - tradeable:[{}]".format(rsp.tradeable))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[pricing_query]:End "))

        return rsp

    def _generate_order_create_data(
        self,
        req: SrvTypeRequest,
    ) -> dict[str, dict[str, str]]:
        data = {
            "order": {
                "type": self._ordertyp_dict[req.ordertype_msg.type],
            }
        }

        data_order = data["order"]

        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)
        one_pip_str = inst_param.one_pip_str

        if (req.ordertype_msg.type == OrderType.TYP_LIMIT) or (
            req.ordertype_msg.type == OrderType.TYP_STOP
        ):

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
                "price": self._fit_unit(req.take_profit_price, one_pip_str),
            },
            "stopLossOnFill": {
                "timeInForce": "GTC",
                "price": self._fit_unit(req.stop_loss_price, one_pip_str),
            },
        }
        data_order.update(tmp)

        return data

    def _generate_trade_crcdo_data(
        self,
        req: SrvTypeRequest,
    ) -> dict[str, dict[str, str]]:

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

    def _fit_unit(self, value: float, one_pip_str: str) -> str:
        return str(
            Decimal(str(value)).quantize(Decimal(one_pip_str), rounding=ROUND_HALF_UP)
        )


def main(args: list[str] | None = None) -> None:

    requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ADD_CIPHERS

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    api_server = ApiServer()

    try:
        rclpy.spin(api_server, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
    finally:
        api_server.destroy_node()
