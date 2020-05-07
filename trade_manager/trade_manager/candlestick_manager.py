import datetime as dt
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument, Granularity
from api_msgs.msg import FailReasonCode as frc


# pd.set_option('display.max_rows', 500)


class CandlestickManager(Node):

    def __init__(self):
        super().__init__("candlestick_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.__DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

        # Create service client "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        self.__cli_cdl = self.__create_service_client(srv_type, srv_name)

        dt_now = dt.datetime.now()

        self.__fetch_day_init(dt_now)

    def __fetch_day_init(self, dt_now: dt.datetime):
        req = CandlesSrv.Request()
        req.gran_msg.granularity_id = Granularity.GRAN_D
        req.inst_msg.instrument_id = Instrument.INST_USD_JPY
        dt_from = dt_now - dt.timedelta(days=365 * 5)
        dt_to = dt_now
        req.dt_from = dt_from.strftime(self.__DT_FMT)
        req.dt_to = dt_to.strftime(self.__DT_FMT)

        future = self.__cli_cdl.call_async(req)
        req_time = dt.datetime.now()
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        dlt_time = dt.datetime.now() - req_time
        self.__logger.debug("<Service:candles> time:%s" % (dlt_time))

        flg = future.done() and future.result() is not None
        assert flg, "initial fetch [Day Candle] failed!"

        rsp = future.result()
        assert rsp.result, "initial fetch [Day Candle] failed!"

        data = []
        for cndl_msg in rsp.cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, self.__DT_FMT)
            data.append([dt_,
                         cndl_msg.ask_o,
                         cndl_msg.ask_h,
                         cndl_msg.ask_l,
                         cndl_msg.ask_c,
                         cndl_msg.bid_o,
                         cndl_msg.bid_h,
                         cndl_msg.bid_l,
                         cndl_msg.bid_c
                         ])

        df = pd.DataFrame(data)
        df.columns = ["time",
                      "open(Ask)",
                      "high(Ask)",
                      "low(Ask)",
                      "close(Ask)",
                      "open(Bid)",
                      "high(Bid)",
                      "low(Bid)",
                      "close(Bid)",
                      ]
        df = df.set_index("time")
        df.index = pd.to_datetime(df.index)
        self.__df_cnd_day = df

        self.__logger.debug("---------- DF[Day] ----------")
        print(df)

    def __create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            self.__logger.info("Waiting for \"" + srv_name + "\" service...")
        return cli


def main(args=None):
    rclpy.init(args=args)
    cm = CandlestickManager()

    try:
        rclpy.spin(cm)
    except KeyboardInterrupt:
        pass

    cm.destroy_node()
    rclpy.shutdown()
