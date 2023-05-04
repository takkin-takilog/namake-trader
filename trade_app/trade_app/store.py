import sys
import numpy as np
import pandas as pd
import datetime as dt
from enum import Enum, auto
from transitions.extensions.factory import HierarchicalMachine
from rclpy.node import Node
from trade_manager_msgs.srv import CandlesByLengthSrv
from trade_manager_msgs.msg import LatestCandle
from .constant import FMT_YMDHMS
from .constant import ConstantGroup
from .constant import Transitions as Tr
from .exception import InitializerErrorException, RosServiceErrorException
from .parameter import InstParam, GranParam
from .wrapper import RosServiceClient, FutureWrapper


class TimeFrameConverter:
    """
    Time frame converter class.
    """

    def __init__(self, gran_param: GranParam, buf_size: int = 100):

        self._RECORD_SIZE = buf_size

        TF_SEC = 60
        TF_MIN = 1
        TF_HOUR = 1
        TF_DAY = 1

        if gran_param == GranParam.M1:
            TF_MIN = 1
        elif gran_param == GranParam.M2:
            TF_MIN = 2
        elif gran_param == GranParam.M3:
            TF_MIN = 3
        elif gran_param == GranParam.M4:
            TF_MIN = 4
        elif gran_param == GranParam.M5:
            TF_MIN = 5
        elif gran_param == GranParam.M10:
            TF_MIN = 10
        elif gran_param == GranParam.M15:
            TF_MIN = 15
        elif gran_param == GranParam.M30:
            TF_MIN = 30
        elif gran_param == GranParam.H1:
            TF_MIN = 60
            TF_HOUR = 1
        elif gran_param == GranParam.H2:
            TF_MIN = 60
            TF_HOUR = 2
        elif gran_param == GranParam.H3:
            TF_MIN = 60
            TF_HOUR = 3
        elif gran_param == GranParam.H4:
            TF_MIN = 60
            TF_HOUR = 4
        elif gran_param == GranParam.H6:
            TF_MIN = 60
            TF_HOUR = 6
        elif gran_param == GranParam.H8:
            TF_MIN = 60
            TF_HOUR = 8
        elif gran_param == GranParam.H12:
            TF_MIN = 60
            TF_HOUR = 12
        elif gran_param == GranParam.D:
            TF_MIN = 60
            TF_HOUR = 24

        self._seconds_tbl = np.repeat(range(0, 60, TF_SEC), TF_SEC)
        self._minutes_tbl = np.repeat(range(0, 60, TF_MIN), TF_MIN)
        self._hours_tbl = np.repeat(range(0, 24, TF_HOUR), TF_HOUR)
        self._days_tbl = np.repeat(range(0, 32, TF_DAY), TF_DAY)

        self._INDEX = "time"
        self._OPEN_ASK = "open(Ask)"
        self._HIGH_ASK = "high(Ask)"
        self._LOW_ASK = "low(Ask)"
        self._CLOSE_ASK = "close(Ask)"
        self._OPEN_MID = "open(Mid)"
        self._HIGH_MID = "high(Mid)"
        self._LOW_MID = "low(Mid)"
        self._CLOSE_MID = "close(Mid)"
        self._OPEN_BID = "open(Bid)"
        self._HIGH_BID = "high(Bid)"
        self._LOW_BID = "low(Bid)"
        self._CLOSE_BID = "close(Bid)"

        self._COLUMNS_ASK = [
            self._OPEN_ASK,
            self._HIGH_ASK,
            self._LOW_ASK,
            self._CLOSE_ASK,
        ]
        self._COLUMNS_MID = [
            self._OPEN_MID,
            self._HIGH_MID,
            self._LOW_MID,
            self._CLOSE_MID,
        ]
        self._COLUMNS_BID = [
            self._OPEN_BID,
            self._HIGH_BID,
            self._LOW_BID,
            self._CLOSE_BID,
        ]

        columns = (
            [self._INDEX] + self._COLUMNS_ASK + self._COLUMNS_MID + self._COLUMNS_BID
        )
        self._df_ohlc = pd.DataFrame(columns=columns)
        self._df_ohlc.set_index(self._INDEX, inplace=True)

    @property
    def df_ohlc(self) -> pd.DataFrame:
        return self._df_ohlc

    def set_price(
        self,
        time: dt.datetime,
        price_ask: float,
        price_bid: float,
    ) -> None:

        price_mid = (price_ask + price_bid) / 2

        day = self._days_tbl[time.day]
        hour = self._hours_tbl[time.hour]
        minute = self._minutes_tbl[time.minute]
        second = self._seconds_tbl[time.second]
        time_frame = dt.datetime(time.year, time.month, day, hour, minute, second)

        if time_frame in self._df_ohlc.index:
            if self._df_ohlc.loc[time_frame][self._HIGH_ASK] < price_ask:
                self._df_ohlc.loc[time_frame][self._HIGH_ASK] = price_ask
            if self._df_ohlc.loc[time_frame][self._HIGH_MID] < price_mid:
                self._df_ohlc.loc[time_frame][self._HIGH_MID] = price_mid
            if self._df_ohlc.loc[time_frame][self._HIGH_BID] < price_bid:
                self._df_ohlc.loc[time_frame][self._HIGH_BID] = price_bid
            if price_ask < self._df_ohlc.loc[time_frame][self._LOW_ASK]:
                self._df_ohlc.loc[time_frame][self._LOW_ASK] = price_ask
            if price_mid < self._df_ohlc.loc[time_frame][self._LOW_MID]:
                self._df_ohlc.loc[time_frame][self._LOW_MID] = price_mid
            if price_bid < self._df_ohlc.loc[time_frame][self._LOW_BID]:
                self._df_ohlc.loc[time_frame][self._LOW_BID] = price_bid

            self._df_ohlc.loc[time_frame][self._CLOSE_ASK] = price_ask
            self._df_ohlc.loc[time_frame][self._CLOSE_MID] = price_mid
            self._df_ohlc.loc[time_frame][self._CLOSE_BID] = price_bid
        else:
            ohlc_data = [
                price_ask,
                price_ask,
                price_ask,
                price_ask,
                price_mid,
                price_mid,
                price_mid,
                price_mid,
                price_bid,
                price_bid,
                price_bid,
                price_bid,
            ]
            self._df_ohlc.loc[time_frame] = ohlc_data

            if self._RECORD_SIZE < len(self._df_ohlc):
                drop_num = len(self._df_ohlc) - self._RECORD_SIZE
                if 0 < drop_num:
                    self._df_ohlc = self._df_ohlc.iloc[drop_num:]


class ColOhlc(ConstantGroup):
    """
    Pandas OHLC dataframe column name.
    """

    DATETIME = "datetime"
    # ---------- OHLC(Ask) ----------
    ASK_O = "ask_o"
    ASK_H = "ask_h"
    ASK_L = "ask_l"
    ASK_C = "ask_c"
    # ---------- OHLC(Mid) ----------
    MID_O = "mid_o"
    MID_H = "mid_h"
    MID_L = "mid_l"
    MID_C = "mid_c"
    # ---------- OHLC(Bid) ----------
    BID_O = "bid_o"
    BID_H = "bid_h"
    BID_L = "bid_l"
    BID_C = "bid_c"
    # ---------- Complete status ----------
    COMP_STS = "comp_sts"


class CompSts(ConstantGroup):
    """
    OHLC data complete status.
    """

    INCOMP = 1  # Incomplete
    TENT = 2  # Tentative complete
    COMP = 3  # Complete


class OhlcStore:
    """
    OHLC store class.
    """

    class States(Enum):
        idle = auto()
        updating = auto()

    def __init__(
        self, node: Node, inst_param: InstParam, gran_param: GranParam, df_length: int
    ):

        self._DF_LENGTH_MAX = df_length
        self._inst_param = inst_param
        self._gran_param = gran_param

        # --------------- Set logger ---------------
        self.logger = node.logger

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME: self.States.idle,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.updating,
                Tr.ON_ENTER: "_on_enter_updating",
                Tr.ON_EXIT: "_on_exit_updating",
            },
        ]

        transitions = [
            {
                Tr.TRIGGER: "_trans_from_idle_to_updating",
                Tr.SOURCE: self.States.idle,
                Tr.DEST: self.States.updating,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_updating_to_idle",
                Tr.SOURCE: self.States.updating,
                Tr.DEST: self.States.idle,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_self_updating",
                Tr.SOURCE: self.States.updating,
                Tr.DEST: "=",
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
        ]

        self._machine = HierarchicalMachine(
            model=self, states=states, initial=self.States.idle, transitions=transitions
        )

        # --------------- Initialize ROS service ---------------
        # Create service client "CandlesByLength"
        srv_type = CandlesByLengthSrv
        srv_name = "candles_by_length"
        self._srvcli = RosServiceClient(node, srv_type, srv_name)

        # --------------- Initialize OHLC DataFrame ---------------
        req = CandlesByLengthSrv.Request()
        req.gran_msg.gran_id = self._gran_param.msg_id
        req.inst_msg.inst_id = self._inst_param.msg_id
        req.length = self._DF_LENGTH_MAX
        try:
            rsp = self._srvcli.call(req)  # type: ignore[var-annotated]
        except Exception as err:
            self.logger.error("{:!^50}".format(" Initialize Error "))
            self.logger.error("[{}]".format(err))
            raise InitializerErrorException("Call Async ROS Service failed.") from err

        tbl = []
        for msg in rsp.cndl_msg_list:
            record = [
                dt.datetime.strptime(msg.time, FMT_YMDHMS),
                msg.ask_o,
                msg.ask_h,
                msg.ask_l,
                msg.ask_c,
                msg.mid_o,
                msg.mid_h,
                msg.mid_l,
                msg.mid_c,
                msg.bid_o,
                msg.bid_h,
                msg.bid_l,
                msg.bid_c,
                CompSts.COMP,
            ]
            tbl.append(record)
        df = pd.DataFrame(tbl, columns=ColOhlc.to_list())

        # --------------- Initialize instance variable ---------------
        self._future: FutureWrapper | None = None
        self._df_ohlc = df.set_index(ColOhlc.DATETIME)
        latest_rec = self._df_ohlc.iloc[-1]
        self._latest_tick_price_ask = latest_rec[ColOhlc.ASK_C]
        self._latest_tick_price_bid = latest_rec[ColOhlc.BID_C]
        self._next_update_time = dt.datetime.strptime(rsp.next_update_time, FMT_YMDHMS)
        if self._df_ohlc.empty:
            self._latest_comp_time = dt.datetime.now() - self._gran_param.timedelta
        else:
            self._latest_comp_time = latest_rec.name
        self._is_completed_update = False

        self._tfc = TimeFrameConverter(self._gran_param, 1)

        self.logger.debug("{:-^50}".format(" Initialize Finish! "))
        self.logger.debug("  - OHLC data length:{}".format(len(self._df_ohlc)))
        self.logger.debug("  - next_update_time:[{}]".format(self._next_update_time))
        self.logger.debug("  - latest_comp_time:[{}]".format(self._latest_comp_time))
        self.logger.debug("  < Head >:\n{}".format(self._df_ohlc[:5]))
        self.logger.debug("  < Tail >:\n{}".format(self._df_ohlc[-5:]))

    @property
    def df_ohlc(self) -> pd.DataFrame:
        return self._df_ohlc

    @property
    def latest_record(self) -> pd.Series:
        return self._df_ohlc.iloc[-1]

    @property
    def latest_tick_price_ask(self) -> float:
        return self._latest_tick_price_ask

    @property
    def latest_comp_time(self) -> dt.datetime:
        return self._latest_comp_time

    @property
    def latest_tick_price_bid(self) -> float:
        return self._latest_tick_price_bid

    def is_completed_update(self) -> bool:
        is_completed = self._is_completed_update
        self._is_completed_update = False
        return is_completed

    def is_filled_df(self) -> bool:
        is_filled = False
        if not self._df_ohlc.empty:
            df_flags = self._df_ohlc[ColOhlc.COMP_STS] == CompSts.COMP
            if self._DF_LENGTH_MAX <= len(self._df_ohlc[df_flags]):
                is_filled = True
        return is_filled

    def do_cyclic_event(self) -> None:

        if self.state == self.States.idle:
            self._on_do_idle()
        elif self.state == self.States.updating:
            self._on_do_updating()
        else:
            pass

    def add_latest_comp_ohlc(self, msg_buff: list[LatestCandle]) -> None:

        for msg in msg_buff:
            comp_time = dt.datetime.strptime(msg.candle.time, FMT_YMDHMS)
            record = [
                msg.candle.ask_o,
                msg.candle.ask_h,
                msg.candle.ask_l,
                msg.candle.ask_c,
                msg.candle.mid_o,
                msg.candle.mid_h,
                msg.candle.mid_l,
                msg.candle.mid_c,
                msg.candle.bid_o,
                msg.candle.bid_h,
                msg.candle.bid_l,
                msg.candle.bid_c,
                CompSts.COMP,
            ]

            if comp_time not in self._df_ohlc.index:
                self._df_ohlc.loc[comp_time] = 0
            self._df_ohlc.loc[comp_time] = record

        self._df_ohlc.sort_index(inplace=True)
        self._df_ohlc = self._delete_overflowing_record(self._df_ohlc)

        # ---------- Validate incomplete record ----------
        if msg_buff:
            df_ohlc_ex = self._df_ohlc.loc[:comp_time]  # type: ignore[misc]
            df_flags = df_ohlc_ex[ColOhlc.COMP_STS] == CompSts.COMP
            if not df_flags.all():
                self.logger.info("{:-^50}".format(" Detected incompleted recored "))
                self.logger.info("\n{}".format(df_ohlc_ex[~df_flags]))
                self._trans_from_idle_to_updating()

            self._latest_comp_time = comp_time
            self._next_update_time = dt.datetime.strptime(
                msg_buff[-1].next_update_time, FMT_YMDHMS
            )

    def update_incomp_ohlc(
        self, time: dt.datetime, tick_price_ask: float, tick_price_bid: float
    ) -> None:
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        self._tfc.set_price(time, tick_price_ask, tick_price_bid)
        ohlc_series = self._tfc.df_ohlc.iloc[-1]
        record = ohlc_series.to_list() + [CompSts.INCOMP]
        if self._df_ohlc.iloc[-1].name != ohlc_series.name:
            self._df_ohlc.loc[ohlc_series.name] = 0
        self._df_ohlc.loc[ohlc_series.name] = record

        index = self._df_ohlc.iloc[-2].name
        if self._df_ohlc.at[index, ColOhlc.COMP_STS] == CompSts.INCOMP:
            self._df_ohlc.at[index, ColOhlc.COMP_STS] = CompSts.TENT

        self._latest_tick_price_ask = tick_price_ask
        self._latest_tick_price_bid = tick_price_bid

    def _on_do_idle(self) -> None:
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        # ---------- Verify "next_update_time" ----------
        now = dt.datetime.now()
        devi_time = now - self._next_update_time
        if devi_time > self._gran_param.timedelta:
            self.logger.warn(
                "Time deviation between current-time and next-update-time "
                + "has been over the threshold!"
            )
            self.logger.warn("  - now             :[{}]".format(now))
            self.logger.warn("  - next_update_time:[{}]".format(self._next_update_time))
            self.logger.warn("  - Time Deviation:[{}]".format(devi_time))

        # ---------- Update "Compete Status" INCOMP -> TENT ----------
        latest_time = self._df_ohlc.iloc[-1].name
        if self._df_ohlc.at[latest_time, ColOhlc.COMP_STS] == CompSts.INCOMP:
            time_th = latest_time + self._gran_param.timedelta
            if time_th < now:
                self._df_ohlc.at[latest_time, ColOhlc.COMP_STS] = CompSts.TENT
                self.logger.debug("----- update [Compete Status] INCOMP -> TENT -----")
                self.logger.debug("  - target time:[{}])".format(latest_time))

    def _on_enter_updating(self) -> None:
        self.logger.debug("----- Call [{}]".format(sys._getframe().f_code.co_name))
        self._future = None

        req = CandlesByLengthSrv.Request()
        req.gran_msg.gran_id = self._gran_param.msg_id
        req.inst_msg.inst_id = self._inst_param.msg_id
        req.length = self._DF_LENGTH_MAX
        try:
            self._future = self._srvcli.call_async(req)
        except RosServiceErrorException as err:
            self.logger.error("{:!^50}".format(" Call Async ROS Service Error "))
            self.logger.error("[{}]".format(err))

    def _on_do_updating(self) -> None:
        self.logger.debug("----- Call [{}]".format(sys._getframe().f_code.co_name))

        if self._future is None:
            self._trans_self_updating()
            return

        if not self._future.done():
            self.logger.debug("  Updating now...")
            return

        self.logger.debug("{:-^50}".format(" Update OHLC data complete! "))
        rsp = self._future.result()  # type: ignore[var-annotated]
        if rsp is None:
            self.logger.error("{:!^50}".format(" Update DataFrame Error "))
            self.logger.error("  future.result() is None.")
            self._trans_self_updating()
            return

        self.logger.debug(
            "  - Appended record length:[{}]".format(len(rsp.cndl_msg_list))
        )
        for cndl_msg in rsp.cndl_msg_list:
            time = dt.datetime.strptime(cndl_msg.time, FMT_YMDHMS)
            record = [
                cndl_msg.ask_o,
                cndl_msg.ask_h,
                cndl_msg.ask_l,
                cndl_msg.ask_c,
                cndl_msg.mid_o,
                cndl_msg.mid_h,
                cndl_msg.mid_l,
                cndl_msg.mid_c,
                cndl_msg.bid_o,
                cndl_msg.bid_h,
                cndl_msg.bid_l,
                cndl_msg.bid_c,
                CompSts.COMP,
            ]
            if time not in self._df_ohlc.index:
                self._df_ohlc.loc[time] = 0
            self._df_ohlc.loc[time] = record

        self._next_update_time = dt.datetime.strptime(rsp.next_update_time, FMT_YMDHMS)

        # delete records could not be updated
        df_ohlc_ex = self._df_ohlc.loc[: self._latest_comp_time]  # type: ignore[misc]
        df_flags = df_ohlc_ex[ColOhlc.COMP_STS] != CompSts.COMP
        df_drop = df_ohlc_ex[df_flags]
        self._df_ohlc.drop(index=df_drop.index, inplace=True)
        if df_drop.empty:
            self.logger.debug("  - Drop DataFrame index: [None]")
        else:
            self.logger.debug("  - Drop DataFrame index: {}".format(df_drop.index))
        self._df_ohlc.sort_index(inplace=True)
        self._df_ohlc = self._delete_overflowing_record(self._df_ohlc)
        self._is_completed_update = True
        self.logger.debug("  - df_ohlc(length:[{}])".format(len(self._df_ohlc)))

        self._trans_from_updating_to_idle()

    def _on_exit_updating(self) -> None:
        # self.logger.debug("----- Call [{}]".format(sys._getframe().f_code.co_name))
        pass

    def _delete_overflowing_record(self, df_ohlc: pd.DataFrame) -> pd.DataFrame:

        df_flags = df_ohlc[ColOhlc.COMP_STS] == CompSts.COMP
        df_comp = df_ohlc[df_flags]
        drop_num = len(df_comp) - self._DF_LENGTH_MAX
        if 0 < drop_num:
            df_ohlc = df_ohlc.iloc[drop_num:]
        return df_ohlc
