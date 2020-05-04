from typing import TypeVar
import datetime as dt
import rclpy
import oandapyV20.endpoints.instruments as instruments
from api_msgs.msg import Granularity, Candle
from api_msgs.msg import FailReasonCode as frc
from api_msgs.srv import CandlesSrv
from oanda_api.service_common import ServiceAbs
from oanda_api.service_common import INST_ID_DICT

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")

GRAN_ID_DICT = {
    Granularity.GRAN_M1: "M1",  # 1 minute
    Granularity.GRAN_M2: "M2",  # 2 minutes
    Granularity.GRAN_M3: "M3",  # 3 minutes
    Granularity.GRAN_M4: "M4",  # 4 minutes
    Granularity.GRAN_M5: "M5",  # 5 minutes
    Granularity.GRAN_M10: "M10",  # 10 minutes
    Granularity.GRAN_M15: "M15",  # 15 minutes
    Granularity.GRAN_M30: "M30",  # 30 minutes
    Granularity.GRAN_H1: "H1",  # 1 hour
    Granularity.GRAN_H2: "H2",  # 2 hours
    Granularity.GRAN_H3: "H3",  # 3 hours
    Granularity.GRAN_H4: "H4",  # 4 hours
    Granularity.GRAN_H6: "H6",  # 6 hours
    Granularity.GRAN_H8: "H8",  # 8 hours
    Granularity.GRAN_H12: "H12",  # 12 hours
    Granularity.GRAN_D: "D",  # 1 Day
    Granularity.GRAN_W: "W",  # 1 Week
}

DT_OFT_DICT = {
    Granularity.GRAN_M1: dt.timedelta(minutes=1),  # 1 minute
    Granularity.GRAN_M2: dt.timedelta(minutes=2),  # 2 minutes
    Granularity.GRAN_M3: dt.timedelta(minutes=3),  # 3 minutes
    Granularity.GRAN_M4: dt.timedelta(minutes=4),  # 4 minutes
    Granularity.GRAN_M5: dt.timedelta(minutes=5),  # 5 minutes
    Granularity.GRAN_M10: dt.timedelta(minutes=10),  # 10 minutes
    Granularity.GRAN_M15: dt.timedelta(minutes=15),  # 15 minutes
    Granularity.GRAN_M30: dt.timedelta(minutes=30),  # 30 minutes
    Granularity.GRAN_H1: dt.timedelta(hours=1),  # 1 hour
    Granularity.GRAN_H2: dt.timedelta(hours=2),  # 2 hours
    Granularity.GRAN_H3: dt.timedelta(hours=3),  # 3 hours
    Granularity.GRAN_H4: dt.timedelta(hours=4),  # 4 hours
    Granularity.GRAN_H6: dt.timedelta(hours=6),  # 6 hours
    Granularity.GRAN_H8: dt.timedelta(hours=8),  # 8 hours
    Granularity.GRAN_H12: dt.timedelta(hours=12),  # 12 hours
    Granularity.GRAN_D: dt.timedelta(days=1),  # 1 Day
    Granularity.GRAN_W: dt.timedelta(weeks=1),  # 1 Week
}


class CandlestickService(ServiceAbs):

    def __init__(self) -> None:
        super().__init__("candlestick_service")

        self.__MAX_SIZE = 5000
        self.__TMDLT = dt.timedelta(hours=9)
        self.__DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

        PRMNM_ACCOUNT_NUMBER = "account_number"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        self._logger.debug("[OANDA]Account Number:%s" % account_number)

        # Create service "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        callback = self.__on_recv_candles
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__account_number = account_number

    def __on_recv_candles(self,
                          req: SrvTypeRequest,
                          rsp: SrvTypeResponse
                          ) -> SrvTypeResponse:

        gran_id = req.gran_msg.granularity_id

        minunit = DT_OFT_DICT[gran_id]

        dt_from = self.__normalize(req.dt_from, gran_id)
        dt_to = self.__normalize(req.dt_to, gran_id) + minunit

        dtnow = dt.datetime.now()
        if dtnow < dt_to:
            dt_to = dtnow

        gran = GRAN_ID_DICT[gran_id]
        inst = INST_ID_DICT[req.inst_msg.instrument_id]
        rsp.cndl_msg_list = []
        tmpdt = dt_from
        from_ = dt_from
        while tmpdt < dt_to:
            tmpdt = tmpdt + (minunit * self.__MAX_SIZE)
            if dt_to < tmpdt:
                tmpdt = dt_to
            to_ = tmpdt

            from_ = self.__adjust(from_, gran_id)

            self._logger.debug("------------ fetch Canclestick ------------")
            self._logger.debug("from:%s" % (from_ - self.__TMDLT))
            self._logger.debug("to:%s" % (to_ - self.__TMDLT))

            params = {
                "from": (from_ - self.__TMDLT).strftime(self.__DT_FMT),
                "to": (to_ - self.__TMDLT).strftime(self.__DT_FMT),
                "granularity": gran,
                "price": "AB"
            }

            ep = instruments.InstrumentsCandles(instrument=inst,
                                                params=params)
            apirsp, rsp = self._request_api(ep, rsp)
            rsp = self.__update_response(apirsp, rsp)

            if rsp.result is False:
                break

            from_ = to_

        rsp.dt_from_act = dt_from.strftime(self.__DT_FMT)
        rsp.dt_to_act = dt_to.strftime(self.__DT_FMT)

        return rsp

    def __normalize(self,
                    datetimestr: str,
                    gran_id: int
                    ) -> dt.datetime:

        dt_ = dt.datetime.strptime(datetimestr, self.__DT_FMT)

        if gran_id == Granularity.GRAN_M1:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute)
        elif gran_id == Granularity.GRAN_M2:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 2))
        elif gran_id == Granularity.GRAN_M3:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 3))
        elif gran_id == Granularity.GRAN_M4:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 4))
        elif gran_id == Granularity.GRAN_M5:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 5))
        elif gran_id == Granularity.GRAN_M10:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 10))
        elif gran_id == Granularity.GRAN_M15:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 15))
        elif gran_id == Granularity.GRAN_M30:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour,
                                dt_.minute - (dt_.minute % 30))
        elif gran_id == Granularity.GRAN_H1:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day, dt_.hour)
        elif gran_id == Granularity.GRAN_H2:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 2))
        elif gran_id == Granularity.GRAN_H3:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 3))
        elif gran_id == Granularity.GRAN_H4:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 4))
        elif gran_id == Granularity.GRAN_H6:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 6))
        elif gran_id == Granularity.GRAN_H8:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 8))
        elif gran_id == Granularity.GRAN_H12:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day,
                                dt_.hour - (dt_.hour % 12))
        elif gran_id == Granularity.GRAN_D:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day)
        elif gran_id == Granularity.GRAN_W:
            dtnor = dt.datetime(dt_.year, dt_.month, dt_.day)
        else:
            pass

        return dtnor

    def __adjust(self,
                 dt_in: dt.datetime,
                 gran_id: int
                 ) -> dt.datetime:

        if ((gran_id == Granularity.GRAN_H2) or
            (gran_id == Granularity.GRAN_H3) or
            (gran_id == Granularity.GRAN_H4) or
            (gran_id == Granularity.GRAN_H6) or
            (gran_id == Granularity.GRAN_H8) or
            (gran_id == Granularity.GRAN_H12) or
            (gran_id == Granularity.GRAN_D) or
                (gran_id == Granularity.GRAN_W)):
            dt_out = dt_in + DT_OFT_DICT[gran_id]
        else:
            dt_out = dt_in

        return dt_out

    def __update_response(self,
                          apirsp: ApiRsp,
                          rsp: SrvTypeResponse
                          ) -> SrvTypeResponse:

        rsp.result = False
        if rsp.frc_msg.reason_code == frc.REASON_UNSET:
            if "candles" in apirsp.keys():
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
                    dttmp = dt.datetime.strptime(raw["time"], self.__DT_FMT)
                    msg.time = (dttmp + self.__TMDLT).strftime(self.__DT_FMT)
                    rsp.cndl_msg_list.append(msg)
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

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
