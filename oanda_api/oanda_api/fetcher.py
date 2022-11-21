from typing import TypeVar
from dataclasses import dataclass
import requests
import datetime as dt
from requests.exceptions import ConnectionError, ReadTimeout
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from oandapyV20 import API
import oandapyV20.endpoints.instruments as instruments
import oandapyV20.endpoints.accounts as accounts
from oandapyV20.exceptions import V20Error
from api_msgs.msg import Candle
from api_msgs.msg import FailReasonCode as frc
from api_msgs.srv import CandlesQuerySrv
from api_msgs.srv import AccountQuerySrv
from .constant import FMT_DTTM_API, FMT_YMDHMS
from .constant import ADD_CIPHERS
from .parameter import InstParam, GranParam
from .dataclass import RosParam
from . import utils as utl

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")


@dataclass
class _RosParams():
    """
    ROS Parameter.
    """
    USE_ENV_LIVE = RosParam("use_env_live")
    PRA_ACCOUNT_NUMBER = RosParam("env_practice.account_number")
    PRA_ACCESS_TOKEN = RosParam("env_practice.access_token")
    LIV_ACCOUNT_NUMBER = RosParam("env_live.account_number")
    LIV_ACCESS_TOKEN = RosParam("env_live.access_token")
    CONNECTION_TIMEOUT = RosParam("connection_timeout")


class Fetcher(Node):

    def __init__(self) -> None:
        super().__init__("fetcher")

        # --------------- Set logger lebel ---------------
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = logger

        # --------------- Define Constant value ---------------
        self._MAX_SIZE = 4999
        # self._MAX_SIZE = 10    # For test

        # --------------- Declare ROS parameter ---------------
        self._rosprm = _RosParams()
        self.declare_parameter(self._rosprm.USE_ENV_LIVE.name)
        self.declare_parameter(self._rosprm.PRA_ACCOUNT_NUMBER.name)
        self.declare_parameter(self._rosprm.PRA_ACCESS_TOKEN.name)
        self.declare_parameter(self._rosprm.LIV_ACCOUNT_NUMBER.name)
        self.declare_parameter(self._rosprm.LIV_ACCESS_TOKEN.name)
        self.declare_parameter(self._rosprm.CONNECTION_TIMEOUT.name)

        para = self.get_parameter(self._rosprm.USE_ENV_LIVE.name)
        self._rosprm.USE_ENV_LIVE.value = para.value
        para = self.get_parameter(self._rosprm.PRA_ACCOUNT_NUMBER.name)
        self._rosprm.PRA_ACCOUNT_NUMBER.value = para.value
        para = self.get_parameter(self._rosprm.PRA_ACCESS_TOKEN.name)
        self._rosprm.PRA_ACCESS_TOKEN.value = para.value
        para = self.get_parameter(self._rosprm.LIV_ACCOUNT_NUMBER.name)
        self._rosprm.LIV_ACCOUNT_NUMBER.value = para.value
        para = self.get_parameter(self._rosprm.LIV_ACCESS_TOKEN.name)
        self._rosprm.LIV_ACCESS_TOKEN.value = para.value
        para = self.get_parameter(self._rosprm.CONNECTION_TIMEOUT.name)
        self._rosprm.CONNECTION_TIMEOUT.value = para.value

        self.logger.debug("[Param]Use Env Live:[{}]".
                          format(self._rosprm.USE_ENV_LIVE.value))
        self.logger.debug("[Param]Env Practice")
        self.logger.debug("  - Account_Number:[{}]"
                          .format(self._rosprm.PRA_ACCOUNT_NUMBER.value))
        self.logger.debug("  - Access Token:[{}]"
                          .format(self._rosprm.PRA_ACCESS_TOKEN.value))
        self.logger.debug("[Param]Env Live")
        self.logger.debug("  - Account_Number:[{}]"
                          .format(self._rosprm.LIV_ACCOUNT_NUMBER.value))
        self.logger.debug("  - Access Token:[{}]"
                          .format(self._rosprm.LIV_ACCESS_TOKEN.value))
        self.logger.debug("[Param]Connection Timeout:[{}]"
                          .format(self._rosprm.CONNECTION_TIMEOUT.value))

        if self._rosprm.USE_ENV_LIVE.value:
            environment = "live"
            access_token = self._rosprm.LIV_ACCESS_TOKEN.value
            account_number = self._rosprm.LIV_ACCOUNT_NUMBER.value
        else:
            environment = "practice"
            access_token = self._rosprm.PRA_ACCESS_TOKEN.value
            account_number = self._rosprm.PRA_ACCOUNT_NUMBER.value

        if self._rosprm.CONNECTION_TIMEOUT.value <= 0:
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm.CONNECTION_TIMEOUT.value}

        self._api = API(access_token=access_token,
                        environment=environment,
                        request_params=request_params)
        self._acc = accounts.AccountSummary(account_number)

        # --------------- Create ROS Communication ---------------
        # Create service server "CandlesQuery"
        srv_type = CandlesQuerySrv
        srv_name = "candles_query"
        callback = self._on_recv_candles_query
        self._cq_srv = self.create_service(srv_type,
                                           srv_name,
                                           callback=callback,
                                           callback_group=ReentrantCallbackGroup())

        # Create service server "AccountQuery"
        srv_type = AccountQuerySrv
        srv_name = "account_query"
        callback = self._on_recv_account_query
        self._aq_srv = self.create_service(srv_type,
                                           srv_name,
                                           callback=callback,
                                           callback_group=ReentrantCallbackGroup())

    def _on_recv_candles_query(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[candles_query]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_msg.gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - dt_from:[{}]".format(req.dt_from))
        self.logger.debug("  - dt_to:[{}]".format(req.dt_to))

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET

        rsp = self._validate_candles_query(req, rsp)
        if rsp.frc_msg.reason_code != frc.REASON_UNSET:
            rsp.result = False
            dbg_tm_end = dt.datetime.now()
            self.logger.debug("<Response>")
            self.logger.debug("  - result:[{}]".format(rsp.result))
            self.logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
            self.logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
            self.logger.debug("[Performance]")
            self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
            self.logger.debug("{:=^50}".format(" Service[candles_query]:End "))
            return rsp

        gran_param = GranParam.get_member_by_msgid(req.gran_msg.gran_id)
        inst_param = InstParam.get_member_by_msgid(req.inst_msg.inst_id)

        minunit = gran_param.timedelta
        dt_from = dt.datetime.strptime(req.dt_from, FMT_YMDHMS)
        dt_to = dt.datetime.strptime(req.dt_to, FMT_YMDHMS)
        dt_to = dt_to + minunit

        dtnow = dt.datetime.now()
        if dtnow < dt_to:
            dt_to = dtnow
        if dtnow - minunit < dt_from:
            dt_from = dtnow - dt.timedelta(seconds=1)

        gran = gran_param.name
        inst = inst_param.name
        tmpdt = dt_from
        from_ = dt_from
        tmplist = []
        rsp.cndl_msg_list = []

        while tmpdt < dt_to:
            rsp.cndl_msg_list = []
            tmpdt = tmpdt + (minunit * self._MAX_SIZE)

            if dt_to < tmpdt:
                tmpdt = dt_to
            to_ = tmpdt

            self.logger.debug("{:-^40}".format(" Service[candles_query]:fetch "))
            self.logger.debug("  - from:[{}]".format(from_))
            self.logger.debug("  - to:  [{}]".format(to_))

            utc_from = utl.convert_from_jst_to_utc(from_)
            utc_to = utl.convert_from_jst_to_utc(to_)
            params = {
                "from": utc_from.strftime(FMT_DTTM_API),
                "to": utc_to.strftime(FMT_DTTM_API),
                "granularity": gran,
                "price": "AB"
            }

            ep = instruments.InstrumentsCandles(instrument=inst,
                                                params=params)
            rsp.frc_msg.reason_code = frc.REASON_UNSET
            apirsp = None
            try:
                apirsp = self._api.request(ep)
            except V20Error as err:
                self.logger.error("{:!^50}".format(" V20Error "))
                self.logger.error("{}".format(err))
                rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
            except ConnectionError as err:
                self.logger.error("{:!^50}".format(" ConnectionError "))
                self.logger.error("{}".format(err))
                rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
            except ReadTimeout as err:
                self.logger.error("{:!^50}".format(" ReadTimeout "))
                self.logger.error("{}".format(err))
                rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
            except Exception as err:
                self.logger.error("{:!^50}".format(" OthersError "))
                self.logger.error("{}".format(err))
                rsp.frc_msg.reason_code = frc.REASON_OTHERS
            else:
                rsp.result = True
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
                        dttmp = dt.datetime.strptime(raw["time"], FMT_DTTM_API)
                        jst_dt = utl.convert_from_utc_to_jst(dttmp)
                        msg.time = jst_dt.strftime(FMT_YMDHMS)
                        msg.is_complete = raw["complete"]
                        rsp.cndl_msg_list.append(msg)

                if rsp.cndl_msg_list:
                    tmplist.append(rsp.cndl_msg_list)

                from_ = to_

        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if not tmplist:
                rsp.result = True
                rsp.frc_msg.reason_code = frc.REASON_DATA_ZERO
            else:
                rsp.result = True
                tmplist2 = []
                for tmp in tmplist:
                    if not tmplist2:
                        tmplist2.extend(tmp)
                    else:
                        if tmplist2[-1].time == tmp[0].time:
                            tmplist2.extend(tmp[1:])
                        else:
                            tmplist2.extend(tmp)
                rsp.cndl_msg_list = tmplist2
        else:
            rsp.result = False

        dbg_tm_end = dt.datetime.now()

        self.logger.debug("<Response>")
        self.logger.debug("  - result:[{}]".format(rsp.result))
        self.logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
        self.logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_query]:End "))

        return rsp

    def _on_recv_account_query(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[account_query]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - None")

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self._api.request(self._acc)
        except V20Error as err:
            self.logger.error("{:!^50}".format(" V20Error "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self.logger.error("{:!^50}".format(" ConnectionError "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self.logger.error("{:!^50}".format(" ReadTimeout "))
            self.logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self.logger.error("{:!^50}".format(" OthersError "))
            self.logger.error("{}".format(err))
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
        self.logger.debug("  - margin_rate:[{}]".format(rsp.margin_rate))
        self.logger.debug("  - balance:[{}]".format(rsp.balance))
        self.logger.debug("  - open_trade_count:[{}]".format(rsp.open_trade_count))
        self.logger.debug("  - open_position_count:[{}]".format(rsp.open_position_count))
        self.logger.debug("  - pending_order_count:[{}]".format(rsp.pending_order_count))
        self.logger.debug("  - pl:[{}]".format(rsp.pl))
        self.logger.debug("  - resettable_pl:[{}]".format(rsp.resettable_pl))
        self.logger.debug("  - financing:[{}]".format(rsp.financing))
        self.logger.debug("  - unrealized_pl:[{}]".format(rsp.unrealized_pl))
        self.logger.debug("  - nav:[{}]".format(rsp.nav))
        self.logger.debug("  - margin_used:[{}]".format(rsp.margin_used))
        self.logger.debug("  - margin_available:[{}]".format(rsp.margin_available))
        self.logger.debug("  - position_value:[{}]".format(rsp.position_value))
        self.logger.debug("  - margin_closeout(%):[{}]".format(rsp.margin_closeout_percent))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[account_query]:End "))

        return rsp

    def _validate_candles_query(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:

        dt_from = dt.datetime.strptime(req.dt_from, FMT_YMDHMS)
        dt_to = dt.datetime.strptime(req.dt_to, FMT_YMDHMS)
        dt_now = dt.datetime.now()

        if (dt_to < dt_from) or (dt_now < dt_from):
            rsp.frc_msg.reason_code = frc.REASON_ARG_ERR
            self.logger.error("{:!^50}".format(" Argument Error "))
            self.logger.error("  - dt_to:[{}]".format(dt_to))
            self.logger.error("  - dt_from:[{}]".format(dt_from))
            self.logger.error("  - dt_now:[{}]".format(dt_now))
        return rsp


def main(args=None):

    requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ADD_CIPHERS

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    fetcher = Fetcher()

    try:
        rclpy.spin(fetcher, executor=executor)
    except KeyboardInterrupt:
        pass

    fetcher.destroy_node()
    rclpy.shutdown()
