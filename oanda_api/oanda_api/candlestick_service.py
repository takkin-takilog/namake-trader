from typing import TypeVar
from dataclasses import dataclass
import requests
import datetime as dt
from requests.exceptions import ConnectionError, ReadTimeout
import rclpy
from rclpy.node import Node
from oandapyV20 import API
import oandapyV20.endpoints.instruments as instruments
from oandapyV20.exceptions import V20Error
from api_msgs.msg import Candle
from api_msgs.msg import FailReasonCode as frc
from api_msgs.srv import CandlesSrv
from oanda_api.constant import FMT_DTTM_API, FMT_YMDHMS
from oanda_api.constant import ADD_CIPHERS
from oanda_api.constant import InstParam, GranParam
from oanda_api.utility import RosParam
from oanda_api import utility as utl


SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")


@dataclass
class _RosParams():
    """
    ROS Parameter.
    """
    USE_ENV_LIVE = RosParam("use_env_live")
    PRA_ACCESS_TOKEN = RosParam("env_practice.access_token")
    LIV_ACCESS_TOKEN = RosParam("env_live.access_token")
    CONNECTION_TIMEOUT = RosParam("connection_timeout")


class CandlestickService(Node):

    def __init__(self) -> None:
        super().__init__("candlestick_service")

        # Set logger lebel
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = logger

        # Define Constant value.
        self._MAX_SIZE = 4999
        # self._MAX_SIZE = 10    # For test

        # Declare ROS parameter
        self._rosprm = _RosParams()
        self.declare_parameter(self._rosprm.USE_ENV_LIVE.name)
        self.declare_parameter(self._rosprm.PRA_ACCESS_TOKEN.name)
        self.declare_parameter(self._rosprm.LIV_ACCESS_TOKEN.name)
        self.declare_parameter(self._rosprm.CONNECTION_TIMEOUT.name)

        # Set ROS parameter
        para = self.get_parameter(self._rosprm.USE_ENV_LIVE.name)
        self._rosprm.USE_ENV_LIVE.value = para.value
        para = self.get_parameter(self._rosprm.PRA_ACCESS_TOKEN.name)
        self._rosprm.PRA_ACCESS_TOKEN.value = para.value
        para = self.get_parameter(self._rosprm.LIV_ACCESS_TOKEN.name)
        self._rosprm.LIV_ACCESS_TOKEN.value = para.value
        para = self.get_parameter(self._rosprm.CONNECTION_TIMEOUT.name)
        self._rosprm.CONNECTION_TIMEOUT.value = para.value

        self.logger.debug("[Param]Use Env Live:[{}]".
                          format(self._rosprm.USE_ENV_LIVE.value))
        self.logger.debug("[Param]Env Practice")
        self.logger.debug("  - Access Token:[{}]"
                          .format(self._rosprm.PRA_ACCESS_TOKEN.value))
        self.logger.debug("[Param]Env Live")
        self.logger.debug("  - Access Token:[{}]"
                          .format(self._rosprm.LIV_ACCESS_TOKEN.value))
        self.logger.debug("[Param]Connection Timeout:[{}]"
                          .format(self._rosprm.CONNECTION_TIMEOUT.value))

        if self._rosprm.USE_ENV_LIVE.value:
            environment = "live"
            access_token = self._rosprm.LIV_ACCESS_TOKEN.value
        else:
            environment = "practice"
            access_token = self._rosprm.PRA_ACCESS_TOKEN.value

        if self._rosprm.CONNECTION_TIMEOUT.value <= 0:
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm.CONNECTION_TIMEOUT.value}

        self._api = API(access_token=access_token,
                        environment=environment,
                        request_params=request_params)

        # Create service server "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        callback = self._on_recv_candles
        self._candles_srv = self.create_service(srv_type,
                                                srv_name,
                                                callback)

    def _on_recv_candles(self,
                         req: SrvTypeRequest,
                         rsp: SrvTypeResponse
                         ) -> SrvTypeResponse:

        self.logger.debug("{:=^50}".format(" Service[candles]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_msg.gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - dt_from:[{}]".format(req.dt_from))
        self.logger.debug("  - dt_to:[{}]".format(req.dt_to))

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET

        rsp = self._check_consistency(req, rsp)
        if rsp.frc_msg.reason_code != frc.REASON_UNSET:
            rsp.result = False
            dbg_tm_end = dt.datetime.now()
            self.logger.debug("<Response>")
            self.logger.debug("  - result:[{}]".format(rsp.result))
            self.logger.debug("  - frc_msg.reason_code:[{}]".format(rsp.frc_msg.reason_code))
            self.logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
            self.logger.debug("[Performance]")
            self.logger.debug("  - Response Time:[{}]".format(dbg_tm_end - dbg_tm_start))
            self.logger.debug("{:=^50}".format(" Service[candles]:End "))
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

            self.logger.debug("{:-^40}".format(" Service[candles]:fetch "))
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
        self.logger.debug("{:=^50}".format(" Service[candles]:End "))

        return rsp

    def _check_consistency(self,
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
    cs = CandlestickService()

    try:
        rclpy.spin(cs)
    except KeyboardInterrupt:
        pass

    cs.destroy_node()
    rclpy.shutdown()
