# mypy: disable-error-code="attr-defined"

from typing import TypeVar
import requests
import traceback
import datetime as dt
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from . import ros_utils as rosutl

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")


class Fetcher(Node):
    """
    Fetcher class.

    """

    def __init__(self) -> None:
        super().__init__("fetcher")

        # --------------- Set logger lebel ---------------
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = logger

        # --------------- Define Constant value ---------------
        self._MAX_SIZE = 4999
        # self._MAX_SIZE = 5  # For test

        # --------------- Declare ROS parameter ---------------
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

        # --------------- Initialize instance variable ---------------
        if self._rosprm_use_env_live.value:
            environment = "live"
            access_token = self._rosprm_liv_access_token.value
            account_number = self._rosprm_liv_account_number.value
        else:
            environment = "practice"
            access_token = self._rosprm_pra_access_token.value
            account_number = self._rosprm_pra_account_number.value

        if self._rosprm_connection_timeout.value <= 0:  # type: ignore[operator]
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm_connection_timeout.value}

        self._api = API(
            access_token=access_token,
            environment=environment,
            request_params=request_params,
        )
        self._acc = accounts.AccountSummary(account_number)

        # --------------- Create ROS Communication ---------------
        # Create service server "CandlesQuery"
        self._cq_srv = self.create_service(
            CandlesQuerySrv,
            "candles_query",
            self._on_recv_candles_query,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create service server "AccountQuery"
        self._aq_srv = self.create_service(
            AccountQuerySrv,
            "account_query",
            self._on_recv_account_query,
            callback_group=ReentrantCallbackGroup(),
        )

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
            except BaseException as err:  # pylint: disable=W0703
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
            apirsp = self._api.request(self._acc)
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
        except BaseException as err:  # pylint: disable=W0703
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


def main(args=None):

    # pylint: disable=E1101
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
