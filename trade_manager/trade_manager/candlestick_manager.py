from typing import TypeVar
import datetime as dt
import pandas as pd
import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.srv import HistoricalCandlesSrv
from trade_manager_msgs.msg import Candle
from trade_manager_msgs.msg import CandleMnt
from trade_manager_msgs.msg import Instrument as InstTm
from trade_manager_msgs.msg import Granularity as GranTm

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class CandlesData():

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

    _DT_LSB_DICT = {
        GranTm.GRAN_M1: dt.timedelta(minutes=1),    # 1 minute
        GranTm.GRAN_M2: dt.timedelta(minutes=2),    # 2 minutes
        GranTm.GRAN_M3: dt.timedelta(minutes=3),    # 3 minutes
        GranTm.GRAN_M4: dt.timedelta(minutes=4),    # 4 minutes
        GranTm.GRAN_M5: dt.timedelta(minutes=5),    # 5 minutes
        GranTm.GRAN_M10: dt.timedelta(minutes=10),  # 10 minutes
        GranTm.GRAN_M15: dt.timedelta(minutes=15),  # 15 minutes
        GranTm.GRAN_M30: dt.timedelta(minutes=30),  # 30 minutes
        GranTm.GRAN_H1: dt.timedelta(hours=1),      # 1 hour
        GranTm.GRAN_H2: dt.timedelta(hours=2),      # 2 hours
        GranTm.GRAN_H3: dt.timedelta(hours=3),      # 3 hours
        GranTm.GRAN_H4: dt.timedelta(hours=4),      # 4 hours
        GranTm.GRAN_H6: dt.timedelta(hours=6),      # 6 hours
        GranTm.GRAN_H8: dt.timedelta(hours=8),      # 8 hours
        GranTm.GRAN_H12: dt.timedelta(hours=12),    # 12 hours
        GranTm.GRAN_D: dt.timedelta(days=1),        # 1 Day
        GranTm.GRAN_W: dt.timedelta(weeks=1),       # 1 Week
    }

    _COL_NAME_TIME = "time"
    _COL_NAME_ASK_OP = "open(Ask)"
    _COL_NAME_ASK_HI = "high(Ask)"
    _COL_NAME_ASK_LO = "low(Ask)"
    _COL_NAME_ASK_CL = "close(Ask)"
    _COL_NAME_BID_OP = "open(Bid)"
    _COL_NAME_BID_HI = "high(Bid)"
    _COL_NAME_BID_LO = "low(Bid)"
    _COL_NAME_BID_CL = "close(Bid)"
    _COL_NAME_COMP = "complete"
    _COL_NAME_MID_OP = "open(Mid)"
    _COL_NAME_MID_HI = "high(Mid)"
    _COL_NAME_MID_LO = "low(Mid)"
    _COL_NAME_MID_CL = "close(Mid)"
    _COL_NAME_HSC_OP = "half_spread_cost(open)"
    _COL_NAME_HSC_HI = "half_spread_cost(high)"
    _COL_NAME_HSC_LO = "half_spread_cost(low)"
    _COL_NAME_HSC_CL = "half_spread_cost(close)"

    _TIMEOUT_SEC = 3.0
    _MARGIN_SEC = dt.timedelta(seconds=2)

    def __init__(self,
                 node: Node,
                 srv_cli: Client,
                 inst_id: InstTm,
                 gran_id: GranTm,
                 data_length: int
                 ) -> None:

        minunit = self._DT_LSB_DICT[gran_id]
        self._logger = node.get_logger()

        self._logger.debug("{:-^40}".format(" Create CandlesData:Start "))
        self._logger.debug("  - inst_id:[{}]".format(inst_id))
        self._logger.debug("  - gran_id:[{}]".format(gran_id))

        dt_now = dt.datetime.now()
        dt_from = dt_now - minunit * data_length
        dt_to = dt_now

        interval = minunit
        dt_1h = dt.timedelta(hours=1)
        if dt_1h < interval:
            interval = dt_1h

        self._srv_cli = srv_cli
        self._inst_id = inst_id
        self._gran_id = gran_id

        cnt = 0
        is_comp = False
        while cnt < 3:
            future = self._request_async(dt_from, dt_to)
            rclpy.spin_until_future_complete(node, future)

            if future.done() and future.result() is not None:
                is_comp = True
                break
            cnt += 1

        if not is_comp:
            self._logger.error("!!!!!!!!!! ROS Service Error !!!!!!!!!!")
            self._logger.error("Service Name:[candles]")
        else:
            df = self._get_df_from_future(future)

            df_comp = df[(df[self._COL_NAME_COMP])]
            df_prov = df[~(df[self._COL_NAME_COMP])]

            self._interval = interval
            self._next_update_time = self._get_next_update_time(gran_id, dt_now)
            self._df_comp = df_comp
            self._df_prov = df_prov
            self._future = None
            self._timeout_end = 0.0

            self._logger.debug("  - last_update_time:[{}]".format(self._next_update_time))
            self._logger.debug("{:-^40}".format(" Create CandlesData:End "))

    def _get_next_update_time(self,
                              gran_id: int,
                              dtin: dt.datetime
                              ) -> dt.datetime:

        if gran_id == GranTm.GRAN_M1:
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, dtin.minute)
        elif gran_id == GranTm.GRAN_M2:
            tmp = dtin.minute - dtin.minute % 2
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M3:
            tmp = dtin.minute - dtin.minute % 3
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M4:
            tmp = dtin.minute - dtin.minute % 4
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M5:
            tmp = dtin.minute - dtin.minute % 5
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M10:
            tmp = dtin.minute - dtin.minute % 10
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M15:
            tmp = dtin.minute - dtin.minute % 15
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M30:
            tmp = dtin.minute - dtin.minute % 30
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif ((gran_id == GranTm.GRAN_H1) or (gran_id == GranTm.GRAN_H2) or
              (gran_id == GranTm.GRAN_H3) or (gran_id == GranTm.GRAN_H4) or
              (gran_id == GranTm.GRAN_H6) or (gran_id == GranTm.GRAN_H8) or
              (gran_id == GranTm.GRAN_H12) or (gran_id == GranTm.GRAN_D) or
                (gran_id == GranTm.GRAN_W)):
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour)
        else:
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour)

        return dtout

    @property
    def dataframe(self) -> pd.DataFrame:
        return self._df_comp

    def update_not_complete_data(self) -> None:

        if self._future is None:

            dt_now = dt.datetime.now()
            """
            self._logger.debug("[{}]update_not_complete_data[inst:{}][gran:{}]"
                                .format(dt_now, self._inst_id, self._gran_id))
            """
            target_time = dt_now - self._next_update_time - self._MARGIN_SEC
            if self._interval < target_time:

                # dt_from = self._next_update_time
                dt_from = self._df_comp.index[-1] + self._interval
                dt_to = dt_now

                self._future = self._request_async(dt_from, dt_to)

                self._timeout_end = time.monotonic() + self._TIMEOUT_SEC

        else:
            if self._future.done() and self._future.result() is not None:

                df = self._get_df_from_future(self._future)
                self._update_df(df)
                self._next_update_time += self._interval
                self._future = None

                self._logger.debug("<Update> inst_id:[{}], gran_id:[{}] last_update_time:[{}]"
                                   .format(self._inst_id, self._gran_id, self._next_update_time))
            else:
                if time.monotonic() >= self._timeout_end:
                    self._future = None

    def _request_async(self,
                       dt_from: dt.datetime,
                       dt_to: dt.datetime
                       ) -> Future:

        req = CandlesSrv.Request()

        req.inst_msg.inst_id = self._inst_id
        req.gran_msg.gran_id = self._gran_id

        req.dt_from = dt_from.strftime(self.DT_FMT)
        req.dt_to = dt_to.strftime(self.DT_FMT)

        future = self._srv_cli.call_async(req)

        return future

    def _get_df_from_future(self, future: Future) -> pd.DataFrame:

        df = pd.DataFrame()
        rsp = future.result()
        if rsp.result is True:

            data = []
            for cndl_msg in rsp.cndl_msg_list:
                dt_ = dt.datetime.strptime(cndl_msg.time, self.DT_FMT)
                hsc_o = (cndl_msg.ask_o - cndl_msg.bid_o) / 2
                hsc_h = (cndl_msg.ask_h - cndl_msg.bid_h) / 2
                hsc_l = (cndl_msg.ask_l - cndl_msg.bid_l) / 2
                hsc_c = (cndl_msg.ask_c - cndl_msg.bid_c) / 2
                data.append([dt_,
                             cndl_msg.ask_o,
                             cndl_msg.ask_h,
                             cndl_msg.ask_l,
                             cndl_msg.ask_c,
                             cndl_msg.bid_o,
                             cndl_msg.bid_h,
                             cndl_msg.bid_l,
                             cndl_msg.bid_c,
                             cndl_msg.bid_o + hsc_o,
                             cndl_msg.bid_h + hsc_h,
                             cndl_msg.bid_l + hsc_l,
                             cndl_msg.bid_c + hsc_c,
                             hsc_o,
                             hsc_h,
                             hsc_l,
                             hsc_c,
                             cndl_msg.is_complete
                             ])

            df = pd.DataFrame(data)

            if not df.empty:
                df.columns = [self._COL_NAME_TIME,
                              self._COL_NAME_ASK_OP,
                              self._COL_NAME_ASK_HI,
                              self._COL_NAME_ASK_LO,
                              self._COL_NAME_ASK_CL,
                              self._COL_NAME_BID_OP,
                              self._COL_NAME_BID_HI,
                              self._COL_NAME_BID_LO,
                              self._COL_NAME_BID_CL,
                              self._COL_NAME_MID_OP,
                              self._COL_NAME_MID_HI,
                              self._COL_NAME_MID_LO,
                              self._COL_NAME_MID_CL,
                              self._COL_NAME_HSC_OP,
                              self._COL_NAME_HSC_HI,
                              self._COL_NAME_HSC_LO,
                              self._COL_NAME_HSC_CL,
                              self._COL_NAME_COMP
                              ]

                TIME = self._COL_NAME_TIME
                if GranTm.GRAN_D <= self._gran_id:
                    df[TIME] = df[TIME].apply(
                        lambda d: dt.datetime(d.year, d.month, d.day))

                df = df.set_index(TIME)

        return df

    def _update_df(self, df: pd.DataFrame):

        if not df.empty:

            df_comp = df[(df[self._COL_NAME_COMP])]
            df_prov = df[~(df[self._COL_NAME_COMP])]

            if not df_comp.empty:
                # "df_comp" deal with FIFO
                self._df_comp = self._df_comp.append(df_comp)
                droplist = self._df_comp.index[range(0, len(df_comp))]
                self._df_comp.drop(droplist, inplace=True)

            if not df_prov.empty:
                self._df_prov = df_prov
            else:
                self._df_prov = self._df_prov[:0]  # All data delete

            self._logger.debug("  - df_comp:[{}]".format(df_comp.index.tolist()))
            self._logger.debug("  - df_prov:[{}]".format(df_prov.index.tolist()))


class CandlestickManager(Node):

    def __init__(self) -> None:
        super().__init__("candlestick_manager")

        # Set logger lebel
        self._logger = super().get_logger()
        self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        ENA_INST = "enable_instrument."
        PRMNM_ENA_INST_USDJPY = ENA_INST + "usdjpy"
        PRMNM_ENA_INST_EURJPY = ENA_INST + "eurjpy"
        PRMNM_ENA_INST_EURUSD = ENA_INST + "eurusd"

        ENA_GRAN = "enable_granularity."
        PRMNM_ENA_GRAN_M1 = ENA_GRAN + "m1"
        PRMNM_ENA_GRAN_M2 = ENA_GRAN + "m2"
        PRMNM_ENA_GRAN_M3 = ENA_GRAN + "m3"
        PRMNM_ENA_GRAN_M4 = ENA_GRAN + "m4"
        PRMNM_ENA_GRAN_M5 = ENA_GRAN + "m5"
        PRMNM_ENA_GRAN_M10 = ENA_GRAN + "m10"
        PRMNM_ENA_GRAN_M15 = ENA_GRAN + "m15"
        PRMNM_ENA_GRAN_M30 = ENA_GRAN + "m30"
        PRMNM_ENA_GRAN_H1 = ENA_GRAN + "h1"
        PRMNM_ENA_GRAN_H2 = ENA_GRAN + "h2"
        PRMNM_ENA_GRAN_H3 = ENA_GRAN + "h3"
        PRMNM_ENA_GRAN_H4 = ENA_GRAN + "h4"
        PRMNM_ENA_GRAN_H6 = ENA_GRAN + "h6"
        PRMNM_ENA_GRAN_H8 = ENA_GRAN + "h8"
        PRMNM_ENA_GRAN_H12 = ENA_GRAN + "h12"
        PRMNM_ENA_GRAN_D = ENA_GRAN + "d"
        PRMNM_ENA_GRAN_W = ENA_GRAN + "w"

        GRAN_HIST = "historical_data_length."
        PRMNM_LENG_M1 = GRAN_HIST + "m1"
        PRMNM_LENG_M2 = GRAN_HIST + "m2"
        PRMNM_LENG_M3 = GRAN_HIST + "m3"
        PRMNM_LENG_M4 = GRAN_HIST + "m4"
        PRMNM_LENG_M5 = GRAN_HIST + "m5"
        PRMNM_LENG_M10 = GRAN_HIST + "m10"
        PRMNM_LENG_M15 = GRAN_HIST + "m15"
        PRMNM_LENG_M30 = GRAN_HIST + "m30"
        PRMNM_LENG_H1 = GRAN_HIST + "h1"
        PRMNM_LENG_H2 = GRAN_HIST + "h2"
        PRMNM_LENG_H3 = GRAN_HIST + "h3"
        PRMNM_LENG_H4 = GRAN_HIST + "h4"
        PRMNM_LENG_H6 = GRAN_HIST + "h6"
        PRMNM_LENG_H8 = GRAN_HIST + "h8"
        PRMNM_LENG_H12 = GRAN_HIST + "h12"
        PRMNM_LENG_D = GRAN_HIST + "d"
        PRMNM_LENG_W = GRAN_HIST + "w"

        # Declare parameter
        self.declare_parameter(PRMNM_ENA_INST_USDJPY)
        self.declare_parameter(PRMNM_ENA_INST_EURJPY)
        self.declare_parameter(PRMNM_ENA_INST_EURUSD)
        self.declare_parameter(PRMNM_ENA_GRAN_M1)
        self.declare_parameter(PRMNM_ENA_GRAN_M2)
        self.declare_parameter(PRMNM_ENA_GRAN_M3)
        self.declare_parameter(PRMNM_ENA_GRAN_M4)
        self.declare_parameter(PRMNM_ENA_GRAN_M5)
        self.declare_parameter(PRMNM_ENA_GRAN_M10)
        self.declare_parameter(PRMNM_ENA_GRAN_M15)
        self.declare_parameter(PRMNM_ENA_GRAN_M30)
        self.declare_parameter(PRMNM_ENA_GRAN_H1)
        self.declare_parameter(PRMNM_ENA_GRAN_H2)
        self.declare_parameter(PRMNM_ENA_GRAN_H3)
        self.declare_parameter(PRMNM_ENA_GRAN_H4)
        self.declare_parameter(PRMNM_ENA_GRAN_H6)
        self.declare_parameter(PRMNM_ENA_GRAN_H8)
        self.declare_parameter(PRMNM_ENA_GRAN_H12)
        self.declare_parameter(PRMNM_ENA_GRAN_D)
        self.declare_parameter(PRMNM_ENA_GRAN_W)
        self.declare_parameter(PRMNM_LENG_M1)
        self.declare_parameter(PRMNM_LENG_M2)
        self.declare_parameter(PRMNM_LENG_M3)
        self.declare_parameter(PRMNM_LENG_M4)
        self.declare_parameter(PRMNM_LENG_M5)
        self.declare_parameter(PRMNM_LENG_M10)
        self.declare_parameter(PRMNM_LENG_M15)
        self.declare_parameter(PRMNM_LENG_M30)
        self.declare_parameter(PRMNM_LENG_H1)
        self.declare_parameter(PRMNM_LENG_H2)
        self.declare_parameter(PRMNM_LENG_H3)
        self.declare_parameter(PRMNM_LENG_H4)
        self.declare_parameter(PRMNM_LENG_H6)
        self.declare_parameter(PRMNM_LENG_H8)
        self.declare_parameter(PRMNM_LENG_H12)
        self.declare_parameter(PRMNM_LENG_D)
        self.declare_parameter(PRMNM_LENG_W)

        ena_inst_usdjpy = self.get_parameter(PRMNM_ENA_INST_USDJPY).value
        ena_inst_eurjpy = self.get_parameter(PRMNM_ENA_INST_EURJPY).value
        ena_inst_eurusd = self.get_parameter(PRMNM_ENA_INST_EURUSD).value
        ena_gran_m1 = self.get_parameter(PRMNM_ENA_GRAN_M1).value
        ena_gran_m2 = self.get_parameter(PRMNM_ENA_GRAN_M2).value
        ena_gran_m3 = self.get_parameter(PRMNM_ENA_GRAN_M3).value
        ena_gran_m4 = self.get_parameter(PRMNM_ENA_GRAN_M4).value
        ena_gran_m5 = self.get_parameter(PRMNM_ENA_GRAN_M5).value
        ena_gran_m10 = self.get_parameter(PRMNM_ENA_GRAN_M10).value
        ena_gran_m15 = self.get_parameter(PRMNM_ENA_GRAN_M15).value
        ena_gran_m30 = self.get_parameter(PRMNM_ENA_GRAN_M30).value
        ena_gran_h1 = self.get_parameter(PRMNM_ENA_GRAN_H1).value
        ena_gran_h2 = self.get_parameter(PRMNM_ENA_GRAN_H2).value
        ena_gran_h3 = self.get_parameter(PRMNM_ENA_GRAN_H3).value
        ena_gran_h4 = self.get_parameter(PRMNM_ENA_GRAN_H4).value
        ena_gran_h6 = self.get_parameter(PRMNM_ENA_GRAN_H6).value
        ena_gran_h8 = self.get_parameter(PRMNM_ENA_GRAN_H8).value
        ena_gran_h12 = self.get_parameter(PRMNM_ENA_GRAN_H12).value
        ena_gran_d = self.get_parameter(PRMNM_ENA_GRAN_D).value
        ena_gran_w = self.get_parameter(PRMNM_ENA_GRAN_W).value
        leng_m1 = self.get_parameter(PRMNM_LENG_M1).value
        leng_m2 = self.get_parameter(PRMNM_LENG_M2).value
        leng_m3 = self.get_parameter(PRMNM_LENG_M3).value
        leng_m4 = self.get_parameter(PRMNM_LENG_M4).value
        leng_m5 = self.get_parameter(PRMNM_LENG_M5).value
        leng_m10 = self.get_parameter(PRMNM_LENG_M10).value
        leng_m15 = self.get_parameter(PRMNM_LENG_M15).value
        leng_m30 = self.get_parameter(PRMNM_LENG_M30).value
        leng_h1 = self.get_parameter(PRMNM_LENG_H1).value
        leng_h2 = self.get_parameter(PRMNM_LENG_H2).value
        leng_h3 = self.get_parameter(PRMNM_LENG_H3).value
        leng_h4 = self.get_parameter(PRMNM_LENG_H4).value
        leng_h6 = self.get_parameter(PRMNM_LENG_H6).value
        leng_h8 = self.get_parameter(PRMNM_LENG_H8).value
        leng_h12 = self.get_parameter(PRMNM_LENG_H12).value
        leng_d = self.get_parameter(PRMNM_LENG_D).value
        leng_w = self.get_parameter(PRMNM_LENG_W).value

        self._logger.debug("[Param]Enable instrument:")
        self._logger.debug("        USD/JPY:[{}]".format(ena_inst_usdjpy))
        self._logger.debug("        EUR/JPY:[{}]".format(ena_inst_eurjpy))
        self._logger.debug("        EUR/USD:[{}]".format(ena_inst_eurusd))
        self._logger.debug("[Param]Enable granularity:")
        self._logger.debug("        M1: [{}]".format(ena_gran_m1))
        self._logger.debug("        M2: [{}]".format(ena_gran_m2))
        self._logger.debug("        M3: [{}]".format(ena_gran_m3))
        self._logger.debug("        M4: [{}]".format(ena_gran_m4))
        self._logger.debug("        M5: [{}]".format(ena_gran_m5))
        self._logger.debug("        M10:[{}]".format(ena_gran_m10))
        self._logger.debug("        M15:[{}]".format(ena_gran_m15))
        self._logger.debug("        M30:[{}]".format(ena_gran_m30))
        self._logger.debug("        H1: [{}]".format(ena_gran_h1))
        self._logger.debug("        H2: [{}]".format(ena_gran_h2))
        self._logger.debug("        H3: [{}]".format(ena_gran_h3))
        self._logger.debug("        H4: [{}]".format(ena_gran_h4))
        self._logger.debug("        H6: [{}]".format(ena_gran_h6))
        self._logger.debug("        H8: [{}]".format(ena_gran_h8))
        self._logger.debug("        H12:[{}]".format(ena_gran_h12))
        self._logger.debug("        D:  [{}]".format(ena_gran_d))
        self._logger.debug("        W:  [{}]".format(ena_gran_w))
        self._logger.debug("[Param]Historical data length:")
        self._logger.debug("        M1: [{}]".format(leng_m1))
        self._logger.debug("        M2: [{}]".format(leng_m2))
        self._logger.debug("        M3: [{}]".format(leng_m3))
        self._logger.debug("        M4: [{}]".format(leng_m4))
        self._logger.debug("        M5: [{}]".format(leng_m5))
        self._logger.debug("        M10:[{}]".format(leng_m10))
        self._logger.debug("        M15:[{}]".format(leng_m15))
        self._logger.debug("        M30:[{}]".format(leng_m30))
        self._logger.debug("        H1: [{}]".format(leng_h1))
        self._logger.debug("        H2: [{}]".format(leng_h2))
        self._logger.debug("        H3: [{}]".format(leng_h3))
        self._logger.debug("        H4: [{}]".format(leng_h4))
        self._logger.debug("        H6: [{}]".format(leng_h6))
        self._logger.debug("        H8: [{}]".format(leng_h8))
        self._logger.debug("        H12:[{}]".format(leng_h12))
        self._logger.debug("        D:  [{}]".format(leng_d))
        self._logger.debug("        W:  [{}]".format(leng_w))

        DATA_LENGTH_DICT = {
            GranApi.GRAN_M1: leng_m1,
            GranApi.GRAN_M2: leng_m2,
            GranApi.GRAN_M3: leng_m3,
            GranApi.GRAN_M4: leng_m4,
            GranApi.GRAN_M5: leng_m5,
            GranApi.GRAN_M10: leng_m10,
            GranApi.GRAN_M15: leng_m15,
            GranApi.GRAN_M30: leng_m30,
            GranApi.GRAN_H1: leng_h1,
            GranApi.GRAN_H2: leng_h2,
            GranApi.GRAN_H3: leng_h3,
            GranApi.GRAN_H4: leng_h4,
            GranApi.GRAN_H6: leng_h6,
            GranApi.GRAN_H8: leng_h8,
            GranApi.GRAN_H12: leng_h12,
            GranApi.GRAN_D: leng_d,
            GranApi.GRAN_W: leng_w,
        }

        INST_ID_LIST = []
        if ena_inst_usdjpy:
            INST_ID_LIST.append(InstApi.INST_USD_JPY)
        if ena_inst_eurjpy:
            INST_ID_LIST.append(InstApi.INST_EUR_JPY)
        if ena_inst_eurusd:
            INST_ID_LIST.append(InstApi.INST_EUR_USD)

        GRAN_ID_LIST = []
        if ena_gran_m1:
            GRAN_ID_LIST.append(GranApi.GRAN_M1)
        if ena_gran_m2:
            GRAN_ID_LIST.append(GranApi.GRAN_M2)
        if ena_gran_m3:
            GRAN_ID_LIST.append(GranApi.GRAN_M3)
        if ena_gran_m4:
            GRAN_ID_LIST.append(GranApi.GRAN_M4)
        if ena_gran_m5:
            GRAN_ID_LIST.append(GranApi.GRAN_M5)
        if ena_gran_m10:
            GRAN_ID_LIST.append(GranApi.GRAN_M10)
        if ena_gran_m15:
            GRAN_ID_LIST.append(GranApi.GRAN_M15)
        if ena_gran_m30:
            GRAN_ID_LIST.append(GranApi.GRAN_M30)
        if ena_gran_h1:
            GRAN_ID_LIST.append(GranApi.GRAN_H1)
        if ena_gran_h2:
            GRAN_ID_LIST.append(GranApi.GRAN_H2)
        if ena_gran_h3:
            GRAN_ID_LIST.append(GranApi.GRAN_H3)
        if ena_gran_h4:
            GRAN_ID_LIST.append(GranApi.GRAN_H4)
        if ena_gran_h6:
            GRAN_ID_LIST.append(GranApi.GRAN_H6)
        if ena_gran_h8:
            GRAN_ID_LIST.append(GranApi.GRAN_H8)
        if ena_gran_h12:
            GRAN_ID_LIST.append(GranApi.GRAN_H12)
        if ena_gran_d:
            GRAN_ID_LIST.append(GranApi.GRAN_D)
        if ena_gran_w:
            GRAN_ID_LIST.append(GranApi.GRAN_W)

        # Create service server "CandlesMonitor"
        srv_type = CandlesMntSrv
        srv_name = "candles_monitor"
        callback = self._on_recv_candlesnt_mnt
        self._candles_mnt_srv = self.create_service(srv_type,
                                                    srv_name,
                                                    callback)

        try:
            # Create service client "Candles"
            srv_type = CandlesSrv
            srv_name = "candles"
            cli_cdl = self._create_service_client(srv_type, srv_name)
        except RuntimeError as err:
            self._logger.error(err)
            self.destroy_node()
            rclpy.shutdown()

        # initialize "data_map"
        obj_map_dict = {}
        for inst_id in INST_ID_LIST:
            gran_dict = {}
            for gran_id in GRAN_ID_LIST:
                data_len = DATA_LENGTH_DICT[gran_id]
                obj = CandlesData(self, cli_cdl, inst_id, gran_id, data_len)
                gran_dict[gran_id] = obj
            obj_map_dict[inst_id] = gran_dict

        # type: Dict[InstrumentMnt][GranularityMnt]
        self._obj_map_dict = obj_map_dict

        # Create service server "HistoricalCandles"
        srv_type = HistoricalCandlesSrv
        srv_name = "historical_candles"
        callback = self._on_recv_historical_candles
        self._hc_srv = self.create_service(srv_type,
                                           srv_name,
                                           callback)

        # Timer(1s)
        self._timer_1s = self.create_timer(timer_period_sec=1.0,
                                           callback=self._on_timeout_1s)

        self._cli_cdl = cli_cdl

        """
        dt_from = dt.datetime(2020, 5, 1)
        dt_to = dt.datetime(2020, 5, 5)

        print("1----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, dt_from, dt_to)
        print(df)
        print("2----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, dt_from)
        print(df)
        print("3----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, None, dt_to)
        print(df)
        print("4----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D)
        print(df)
        """

    def _get_dataframe(self,
                       inst_id: InstApi,
                       gran_id: GranApi,
                       dt_from: dt.datetime=None,
                       dt_to: dt.datetime=None
                       ) -> pd.DataFrame:

        df = self._obj_map_dict[inst_id][gran_id].dataframe

        if dt_from is not None:
            dftmp = df[(dt_from <= df.index)]
        else:
            dftmp = df

        if dt_to is not None:
            dftmp = dftmp[(dftmp.index <= dt_to)]

        return dftmp

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

    def _on_recv_historical_candles(self,
                                    req: SrvTypeRequest,
                                    rsp: SrvTypeResponse
                                    ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[historical_candles]:Start "))
        logger.debug("<Request>")
        logger.debug("  - gran_msg.gran_id:[{}]".format(req.gran_msg.gran_id))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - dt_from:[{}]".format(req.dt_from))
        logger.debug("  - dt_to:[{}]".format(req.dt_to))
        dbg_tm_start = dt.datetime.now()

        DT_FMT = CandlesData.DT_FMT

        if req.dt_from == "":
            dt_from = None
        else:
            dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)

        if req.dt_to == "":
            dt_to = None
        else:
            dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)

        df = self._get_dataframe(req.inst_msg.inst_id,
                                 req.gran_msg.gran_id,
                                 dt_from,
                                 dt_to)

        rsp.cndl_msg_list = []
        if not df.empty:
            for time, sr in df.iterrows():
                msg = Candle()
                msg.ask_o = sr[CandlesData._COL_NAME_ASK_OP]
                msg.ask_h = sr[CandlesData._COL_NAME_ASK_HI]
                msg.ask_l = sr[CandlesData._COL_NAME_ASK_LO]
                msg.ask_c = sr[CandlesData._COL_NAME_ASK_CL]
                msg.mid_o = sr[CandlesData._COL_NAME_MID_OP]
                msg.mid_h = sr[CandlesData._COL_NAME_MID_HI]
                msg.mid_l = sr[CandlesData._COL_NAME_MID_LO]
                msg.mid_c = sr[CandlesData._COL_NAME_MID_CL]
                msg.bid_o = sr[CandlesData._COL_NAME_BID_OP]
                msg.bid_h = sr[CandlesData._COL_NAME_BID_HI]
                msg.bid_l = sr[CandlesData._COL_NAME_BID_LO]
                msg.bid_c = sr[CandlesData._COL_NAME_BID_CL]
                msg.time = time.strftime(DT_FMT)
                msg.is_complete = sr[CandlesData._COL_NAME_COMP]
                rsp.cndl_msg_list.append(msg)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[historical_candles]:End "))

        return rsp

    def _on_recv_candlesnt_mnt(self,
                               req: SrvTypeRequest,
                               rsp: SrvTypeResponse
                               ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("{:=^50}".format(" Service[candles_monitor]:Start "))
        logger.debug("<Request>")
        logger.debug("  - gran_msg.gran_id:[{}]".format(req.gran_msg.gran_id))
        logger.debug("  - inst_msg.inst_id:[{}]".format(req.inst_msg.inst_id))
        logger.debug("  - dt_from:[{}]".format(req.dt_from))
        logger.debug("  - dt_to:[{}]".format(req.dt_to))
        dbg_tm_start = dt.datetime.now()

        DT_FMT = CandlesData.DT_FMT

        if req.dt_from == "":
            dt_from = None
        else:
            dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)

        if req.dt_to == "":
            dt_to = None
        else:
            dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)

        df = self._get_dataframe(req.inst_msg.inst_id,
                                 req.gran_msg.gran_id,
                                 dt_from,
                                 dt_to)

        rsp.cndl_msg_list = []
        if not df.empty:
            for time, sr in df.iterrows():
                msg = CandleMnt()
                msg.ask_o = sr[CandlesData._COL_NAME_ASK_OP]
                msg.ask_h = sr[CandlesData._COL_NAME_ASK_HI]
                msg.ask_l = sr[CandlesData._COL_NAME_ASK_LO]
                msg.ask_c = sr[CandlesData._COL_NAME_ASK_CL]
                msg.mid_o = sr[CandlesData._COL_NAME_MID_OP]
                msg.mid_h = sr[CandlesData._COL_NAME_MID_HI]
                msg.mid_l = sr[CandlesData._COL_NAME_MID_LO]
                msg.mid_c = sr[CandlesData._COL_NAME_MID_CL]
                msg.bid_o = sr[CandlesData._COL_NAME_BID_OP]
                msg.bid_h = sr[CandlesData._COL_NAME_BID_HI]
                msg.bid_l = sr[CandlesData._COL_NAME_BID_LO]
                msg.bid_c = sr[CandlesData._COL_NAME_BID_CL]
                msg.time = time.strftime(DT_FMT)
                msg.is_complete = sr[CandlesData._COL_NAME_COMP]
                rsp.cndl_msg_list.append(msg)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
        logger.debug("[Performance]")
        logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        logger.debug("{:=^50}".format(" Service[order_create]:End "))

        return rsp

    def _on_timeout_1s(self) -> None:

        for gran_dict in self._obj_map_dict.values():
            for obj in gran_dict.values():
                obj.update_not_complete_data()


def main(args=None):
    rclpy.init(args=args)
    cm = CandlestickManager()
    try:
        rclpy.spin(cm)
    except KeyboardInterrupt:
        pass

    cm.destroy_node()
    rclpy.shutdown()
