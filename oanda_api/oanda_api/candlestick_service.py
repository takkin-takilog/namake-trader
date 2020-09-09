from typing import TypeVar
import datetime as dt
import rclpy
import oandapyV20.endpoints.instruments as instruments
from api_msgs.msg import Candle
from api_msgs.msg import FailReasonCode as frc
from api_msgs.srv import CandlesSrv
from oanda_api.service_common import ServiceAbs
from oanda_api.service_common import INST_DICT, GRAN_DICT

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")


class CandlestickService(ServiceAbs):

    MAX_SIZE = 4999
    # MAX_SIZE = 10    # For test
    TMDLT = dt.timedelta(hours=9)
    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

    def __init__(self) -> None:
        super().__init__("candlestick_service")

        PRMNM_ACCOUNT_NUMBER = "account_number"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        self._logger.debug("[Param]Account Number:[%s]" % (account_number))

        # Create service server "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        callback = self.__on_recv_candles
        self.__candles_srv = self.create_service(srv_type,
                                                 srv_name,
                                                 callback)

        self.__account_number = account_number

    def __on_recv_candles(self,
                          req: SrvTypeRequest,
                          rsp: SrvTypeResponse
                          ) -> SrvTypeResponse:

        self._logger.debug("========== Service[candles]:Start ==========")
        self._logger.debug("<Request>")
        self._logger.debug("- gran_msg.gran_id:[%d]" % (req.gran_msg.gran_id))
        self._logger.debug("- inst_msg.inst_id:[%d]" % (req.inst_msg.inst_id))
        self._logger.debug("- dt_from:[%s]" % (req.dt_from))
        self._logger.debug("- dt_to:[%s]" % (req.dt_to))

        dbg_tm_start = dt.datetime.now()

        rsp.result = False
        rsp.frc_msg.reason_code = frc.REASON_UNSET

        rsp = self.__check_consistency(req, rsp)
        if rsp.frc_msg.reason_code != frc.REASON_UNSET:
            rsp.result = False
            dbg_tm_end = dt.datetime.now()
            self._logger.debug("<Response>")
            self._logger.debug("- result:[%r]" % (rsp.result))
            self._logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
            self._logger.debug("- cndl_msg_list(length):[%d]" % (len(rsp.cndl_msg_list)))
            self._logger.debug("[Performance]")
            self._logger.debug("- Response Time:[%s]" % (dbg_tm_end - dbg_tm_start))
            self._logger.debug("========== Service[candles]:End ==========")
            return rsp

        gran_id = req.gran_msg.gran_id
        minunit = GRAN_DICT[gran_id].timedelta
        dt_from = dt.datetime.strptime(req.dt_from, self.DT_FMT)
        dt_to = dt.datetime.strptime(req.dt_to, self.DT_FMT)
        dt_to = dt_to + minunit

        dtnow = dt.datetime.now()
        if dtnow < dt_to:
            dt_to = dtnow
        if dtnow - minunit < dt_from:
            dt_from = dtnow - dt.timedelta(seconds=1)

        gran = GRAN_DICT[gran_id].name
        inst = INST_DICT[req.inst_msg.inst_id].name
        tmpdt = dt_from
        from_ = dt_from
        tmplist = []
        rsp.cndl_msg_list = []

        while tmpdt < dt_to:
            rsp.cndl_msg_list = []
            tmpdt = tmpdt + (minunit * self.MAX_SIZE)

            if dt_to < tmpdt:
                tmpdt = dt_to
            to_ = tmpdt

            self._logger.debug("----- Service[candles]:fetch -----")
            self._logger.debug("- from:[%s]" % from_)
            self._logger.debug("- to:  [%s]" % to_)

            params = {
                "from": (from_ - self.TMDLT).strftime(self.DT_FMT),
                "to": (to_ - self.TMDLT).strftime(self.DT_FMT),
                "granularity": gran,
                "price": "AB"
            }

            ep = instruments.InstrumentsCandles(instrument=inst,
                                                params=params)
            apirsp, rsp = self._request_api(ep, rsp)

            if rsp.frc_msg.reason_code != frc.REASON_UNSET:
                break

            rsp = self.__update_response(apirsp, rsp)

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

        self._logger.debug("<Response>")
        self._logger.debug("- result:[%r]" % (rsp.result))
        self._logger.debug("- frc_msg.reason_code:[%d]" % (rsp.frc_msg.reason_code))
        self._logger.debug("- cndl_msg_list(length):[%d]" % (len(rsp.cndl_msg_list)))
        self._logger.debug("[Performance]")
        self._logger.debug("- Response Time:[%s]" % (dbg_tm_end - dbg_tm_start))
        self._logger.debug("========== Service[candles]:End ==========")

        return rsp

    def __check_consistency(self,
                            req: SrvTypeRequest,
                            rsp: SrvTypeResponse
                            ) -> SrvTypeResponse:

        dt_from = dt.datetime.strptime(req.dt_from, self.DT_FMT)
        dt_to = dt.datetime.strptime(req.dt_to, self.DT_FMT)
        dt_now = dt.datetime.now()

        if (dt_to < dt_from) or (dt_now < dt_from):
            rsp.frc_msg.reason_code = frc.REASON_ARG_ERR
            self.__logger.error("!!!!!!!!!! Argument Error !!!!!!!!!!")
            self.__logger.error("- dt_to:[%s]" % (dt_to))
            self.__logger.error("- dt_from:[%s]" % (dt_from))
            self.__logger.error("- dt_now:[%s]" % (dt_now))
        return rsp

    def __update_response(self,
                          apirsp: ApiRsp,
                          rsp: SrvTypeResponse
                          ) -> SrvTypeResponse:

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
                dttmp = dt.datetime.strptime(raw["time"], self.DT_FMT)
                msg.time = (dttmp + self.TMDLT).strftime(self.DT_FMT)
                msg.is_complete = raw["complete"]
                rsp.cndl_msg_list.append(msg)

        return rsp


def main(args=None):
    rclpy.init(args=args)
    cs = CandlestickService()

    try:
        rclpy.spin(cs)
    except KeyboardInterrupt:
        pass

    cs.destroy_node()
    rclpy.shutdown()
