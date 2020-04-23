import datetime as dt
import rclpy
from rclpy.node import Node
from api_msgs.msg import Granularity, Instrument, Candle
from api_msgs.msg import FailReasonCode as frc
from api_msgs.srv import CandlesSrv
import oandapyV20.endpoints.instruments as instruments
from oandapyV20 import API
from oandapyV20.exceptions import V20Error

GRAN_ID_DICT = {
    Granularity.GRAN_S5: "S5",  # 5 seconds
    Granularity.GRAN_S10: "S10",  # 10 seconds
    Granularity.GRAN_S15: "S15",  # 15 seconds
    Granularity.GRAN_S30: "S30",  # 30 seconds
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

ORDER_INST_ID_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
}

DT_OFT_DICT = {
    Granularity.GRAN_S5: dt.timedelta(seconds=5),  # 5 seconds
    Granularity.GRAN_S10: dt.timedelta(seconds=10),  # 10 seconds
    Granularity.GRAN_S15: dt.timedelta(seconds=15),  # 15 seconds
    Granularity.GRAN_S30: dt.timedelta(seconds=30),  # 30 seconds
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
    Granularity.GRAN_D: dt.timedelta(day=1),  # 1 Day
    Granularity.GRAN_W: dt.timedelta(weeks=1),  # 1 Week
}


class CandlestickService(Node):

    def __init__(self):
        super().__init__("candlestick_service")

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

        # Create service "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        callback = self.__on_recv_candles
        self.order_create_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        self.__api = API(access_token=access_token)
        self.__account_number = account_number

    def __on_recv_candles(self, req, rsp):

        DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"
        ofs = DT_OFT_DICT[req.gran_msg.granularity_id]

        self.__normalize(req)

        gran = GRAN_ID_DICT[req.gran_msg.granularity_id]
        inst = ORDER_INST_ID_DICT[req.inst_msg.instrument_id]
        params = {
            "alignmentTimezone": "Japan",
            "from": req.dt_from.strftime(DT_FMT),
            "to": req.dt_to.strftime(DT_FMT),
            "granularity": gran,
            "price": "AB"
        }

        # APIへ過去データをリクエスト
        ep = instruments.InstrumentsCandles(instrument=inst,
                                            params=params)

        apirsp, rsp = self.__request_api(ep, rsp)
        rsp = self.__update_response(apirsp, rsp)

        return rsp

    def __normalize(self, req):

        DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

        dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)
        dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)
        gran_id = req.gran_msg.granularity_id

        if ((gran_id == Granularity.GRAN_S5) or
            (gran_id == Granularity.GRAN_S10) or
            (gran_id == Granularity.GRAN_S15) or
                (gran_id == Granularity.GRAN_S30)):
            pass
        elif ((gran_id == Granularity.GRAN_M1) or
              (gran_id == Granularity.GRAN_M2) or
              (gran_id == Granularity.GRAN_M3) or
              (gran_id == Granularity.GRAN_M4) or
              (gran_id == Granularity.GRAN_M5) or
              (gran_id == Granularity.GRAN_M10) or
              (gran_id == Granularity.GRAN_M15) or
              (gran_id == Granularity.GRAN_M30)):
            pass
        elif ((gran_id == Granularity.GRAN_H1) or
              (gran_id == Granularity.GRAN_H2) or
              (gran_id == Granularity.GRAN_H3) or
              (gran_id == Granularity.GRAN_H4) or
              (gran_id == Granularity.GRAN_H6) or
              (gran_id == Granularity.GRAN_H8) or
              (gran_id == Granularity.GRAN_H12)):
            pass
        elif gran_id == Granularity.GRAN_D:
            pass
        elif gran_id == Granularity.GRAN_W:
            pass
        else:
            pass

    def __update_response(self, apirsp, rsp):
        import json
        print(json.dumps(apirsp, indent=2))

        rsp.cndl_msg_list = []
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
                    msg.time = raw["time"]
                    rsp.cndl_msg_list.append(msg)
                rsp.result = True
            else:
                rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return rsp

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


def main(args=None):
    rclpy.init(args=args)
    cs = CandlestickService()

    try:
        rclpy.spin(cs)
    except KeyboardInterrupt:
        pass

    cs.destroy_node()
    rclpy.shutdown()
