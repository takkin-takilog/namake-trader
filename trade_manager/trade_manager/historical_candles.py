import sys
from typing import List
from typing import TypeVar
from dataclasses import dataclass
from enum import Enum, auto
import datetime as dt
import pandas as pd
from transitions import Machine
# from transitions.extensions.factory import GraphMachine as Machine
from transitions.extensions.factory import GraphMachine
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.client import Client
from rclpy.task import Future
from trade_manager.constant import FMT_YMDHMS, FMT_TIME_HMS
from trade_manager.constant import MIN_TIME, MAX_TIME
from trade_manager.exception import InitializerErrorException
import trade_manager.utility as utl
from trade_manager.utility import RosParam
from trade_manager.data import Transitions as Tr
from trade_manager.data import WeekDay
from trade_manager.data import GranParam, InstParam
from trade_manager.data import CandleColumnNames as ColName
from trade_manager.data import INST_DICT, GRAN_DICT
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from trade_manager_msgs.srv import CandlesByDatetimeSrv
from trade_manager_msgs.srv import CandlesByLengthSrv
from trade_manager_msgs.msg import Candle
from trade_manager_msgs.msg import LatestCandle

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


@dataclass
class _GranData():
    """
    Granularity data.
    """
    gran_id: int
    length: int


@dataclass
class _RosParams():
    """
    ROS Parameter.
    """
    ENA_INST_USDJPY = RosParam("enable_instrument.usdjpy")
    ENA_INST_EURJPY = RosParam("enable_instrument.eurjpy")
    ENA_INST_EURUSD = RosParam("enable_instrument.eurusd")
    ENA_GRAN_M1 = RosParam("enable_granularity.m1")
    ENA_GRAN_M2 = RosParam("enable_granularity.m2")
    ENA_GRAN_M3 = RosParam("enable_granularity.m3")
    ENA_GRAN_M4 = RosParam("enable_granularity.m4")
    ENA_GRAN_M5 = RosParam("enable_granularity.m5")
    ENA_GRAN_M10 = RosParam("enable_granularity.m10")
    ENA_GRAN_M15 = RosParam("enable_granularity.m15")
    ENA_GRAN_M30 = RosParam("enable_granularity.m30")
    ENA_GRAN_H1 = RosParam("enable_granularity.h1")
    ENA_GRAN_H2 = RosParam("enable_granularity.h2")
    ENA_GRAN_H3 = RosParam("enable_granularity.h3")
    ENA_GRAN_H4 = RosParam("enable_granularity.h4")
    ENA_GRAN_H6 = RosParam("enable_granularity.h6")
    ENA_GRAN_H8 = RosParam("enable_granularity.h8")
    ENA_GRAN_H12 = RosParam("enable_granularity.h12")
    ENA_GRAN_D = RosParam("enable_granularity.d")
    ENA_GRAN_W = RosParam("enable_granularity.w")
    LENG_M1 = RosParam("data_length.m1")
    LENG_M2 = RosParam("data_length.m2")
    LENG_M3 = RosParam("data_length.m3")
    LENG_M4 = RosParam("data_length.m4")
    LENG_M5 = RosParam("data_length.m5")
    LENG_M10 = RosParam("data_length.m10")
    LENG_M15 = RosParam("data_length.m15")
    LENG_M30 = RosParam("data_length.m30")
    LENG_H1 = RosParam("data_length.h1")
    LENG_H2 = RosParam("data_length.h2")
    LENG_H3 = RosParam("data_length.h3")
    LENG_H4 = RosParam("data_length.h4")
    LENG_H6 = RosParam("data_length.h6")
    LENG_H8 = RosParam("data_length.h8")
    LENG_H12 = RosParam("data_length.h12")
    LENG_D = RosParam("data_length.d")
    LENG_W = RosParam("data_length.w")

    def enable_inst_list(self):
        inst_list = []
        if self.ENA_INST_USDJPY.value:
            inst_list.append(InstApi.INST_USD_JPY)
        if self.ENA_INST_EURJPY.value:
            inst_list.append(InstApi.INST_EUR_JPY)
        if self.ENA_INST_EURUSD.value:
            inst_list.append(InstApi.INST_EUR_USD)
        return inst_list

    def enable_gran_list(self):
        gran_list = []
        if self.ENA_GRAN_M1.value:
            gran_list.append(_GranData(GranApi.GRAN_M1, self.LENG_M1.value))
        if self.ENA_GRAN_M2.value:
            gran_list.append(_GranData(GranApi.GRAN_M2, self.LENG_M2.value))
        if self.ENA_GRAN_M3.value:
            gran_list.append(_GranData(GranApi.GRAN_M3, self.LENG_M3.value))
        if self.ENA_GRAN_M4.value:
            gran_list.append(_GranData(GranApi.GRAN_M4, self.LENG_M4.value))
        if self.ENA_GRAN_M5.value:
            gran_list.append(_GranData(GranApi.GRAN_M5, self.LENG_M5.value))
        if self.ENA_GRAN_M10.value:
            gran_list.append(_GranData(GranApi.GRAN_M10, self.LENG_M10.value))
        if self.ENA_GRAN_M15.value:
            gran_list.append(_GranData(GranApi.GRAN_M15, self.LENG_M15.value))
        if self.ENA_GRAN_M30.value:
            gran_list.append(_GranData(GranApi.GRAN_M30, self.LENG_M30.value))
        if self.ENA_GRAN_H1.value:
            gran_list.append(_GranData(GranApi.GRAN_H1, self.LENG_H1.value))
        if self.ENA_GRAN_H2.value:
            gran_list.append(_GranData(GranApi.GRAN_H2, self.LENG_H2.value))
        if self.ENA_GRAN_H3.value:
            gran_list.append(_GranData(GranApi.GRAN_H3, self.LENG_H3.value))
        if self.ENA_GRAN_H4.value:
            gran_list.append(_GranData(GranApi.GRAN_H4, self.LENG_H4.value))
        if self.ENA_GRAN_H6.value:
            gran_list.append(_GranData(GranApi.GRAN_H6, self.LENG_H6.value))
        if self.ENA_GRAN_H8.value:
            gran_list.append(_GranData(GranApi.GRAN_H8, self.LENG_H8.value))
        if self.ENA_GRAN_H12.value:
            gran_list.append(_GranData(GranApi.GRAN_H12, self.LENG_H12.value))
        if self.ENA_GRAN_D.value:
            gran_list.append(_GranData(GranApi.GRAN_D, self.LENG_D.value))
        if self.ENA_GRAN_W.value:
            gran_list.append(_GranData(GranApi.GRAN_W, self.LENG_W.value))
        return gran_list


class CandlesData():

    class States(Enum):
        waiting = auto()
        updating = auto()
        retrying = auto()

    cli_cdl = None
    logger = None
    daily_param = None

    def __init__(self,
                 node: 'Node',
                 inst_id: int,
                 gran_data: _GranData
                 ) -> None:

        # Define Constant value.
        gran_param = GranParam.get_member_by_msgid(gran_data.gran_id)
        self._GRAN_INTERVAL = gran_param.timedelta
        self._NEXT_UPDATETIME_OFS_SEC = dt.timedelta(seconds=5)
        self._RETRY_INTERVAL = dt.timedelta(minutes=1)
        self._FAIL_INTERVAL = dt.timedelta(minutes=10)
        self._RETRY_COUNT_MAX = 2
        self._SELF_RETRY_COUNT_MAX = 2

        # ---------- Create State Machine ----------
        states = [
            {
                Tr.NAME.value: self.States.waiting,
                Tr.ON_ENTER.value: "_on_entry_waiting",
                Tr.ON_EXIT.value: "_on_exit_waiting"
            },
            {
                Tr.NAME.value: self.States.updating,
                Tr.ON_ENTER.value: "_on_entry_updating",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.retrying,
                Tr.ON_ENTER.value: "_on_enrty_retrying",
                Tr.ON_EXIT.value: "_on_exit_retrying"
            },
        ]

        transitions = [
            {
                Tr.TRIGGER.value: "_trans_from_wating_to_updating",
                Tr.SOURCE.value: self.States.waiting,
                Tr.DEST.value: self.States.updating,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_updating_to_retrying",
                Tr.SOURCE.value: self.States.updating,
                Tr.DEST.value: self.States.retrying,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_retrying_to_updating",
                Tr.SOURCE.value: self.States.retrying,
                Tr.DEST.value: self.States.updating,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_updating_to_waiting",
                Tr.SOURCE.value: self.States.updating,
                Tr.DEST.value: self.States.waiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_self_updating",
                Tr.SOURCE.value: self.States.updating,
                Tr.DEST.value: "=",
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
        ]

        self._sm = Machine(model=self,
                           states=states,
                           initial=self.States.waiting,
                           transitions=transitions)

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        self._inst_id = inst_id
        self._gran_id = gran_data.gran_id
        self._df_comp = pd.DataFrame()
        self._df_prov = pd.DataFrame()
        self._future = None
        self._is_update_complete = True

        self.logger.debug("{:-^40}".format(" Create CandlesData:Start "))
        self.logger.debug("  - inst_id:[{}]".format(self._inst_id))
        self.logger.debug("  - gran_id:[{}]".format(self._gran_id))

        dt_now = dt.datetime.now()
        dt_from = dt_now - self._GRAN_INTERVAL * gran_data.length
        dt_to = dt_now

        self.logger.debug("  - time_from:[{}]".format(dt_from))
        self.logger.debug("  - time_to  :[{}]".format(dt_to))

        try:
            future = self._request_async_candles(dt_from, dt_to)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Candles) "))
            self.logger.error("{}".format(err))
            raise InitializerErrorException("\"CandlesData\" initialize failed.")

        rclpy.spin_until_future_complete(node, future)
        rsp = future.result()
        if rsp.result is True:
            self._update_dataframe(rsp.cndl_msg_list)
            self.logger.debug("---------- df_comp(length:[{}]) ----------"
                              .format(len(self._df_comp)))
            self.logger.debug("  - Head:\n{}".format(self._df_comp[:5]))
            self.logger.debug("  - Tail:\n{}".format(self._df_comp[-5:]))
            self.logger.debug("---------- df_prov(length:[{}]) ----------"
                              .format(len(self._df_prov)))
            self.logger.debug("\n{}".format(self._df_prov))
        else:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Candles) "))
            self.logger.error("  future result is False")
            raise InitializerErrorException("\"CandlesData\" initialize failed.")

        # Declare publisher and subscriber
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        inst_name = InstParam.get_member_by_msgid(self._inst_id).namespace
        gran_name = GranParam.get_member_by_msgid(self._gran_id).namespace
        TPCNM_LATEST_CANDLE = inst_name + "_" + gran_name + "_latest_candle"
        self._pub = node.create_publisher(LatestCandle,
                                          TPCNM_LATEST_CANDLE,
                                          qos_profile)

        self._on_entry_waiting()

    @property
    def inst_id(self):
        return self._inst_id

    @property
    def gran_id(self):
        return self._gran_id

    @property
    def df_comp(self):
        return self._df_comp

    @property
    def next_updatetime(self):
        return self._next_updatetime

    def _get_latest_datetime_in_dataframe(self) -> dt.datetime:
        return self._df_comp.index[-1].to_pydatetime()

    def _get_next_update_datetime(self,
                                  latest_datetime: dt.datetime
                                  ) -> dt.datetime:

        next_update_dt = latest_datetime + self._GRAN_INTERVAL

        if next_update_dt.weekday() == WeekDay.SAT.value:
            close_time = utl.get_market_close_time(next_update_dt.date())
            if close_time <= next_update_dt.time():
                self.logger.debug(" - Weekly Data Complete!")
                next_monday = next_update_dt.date() + dt.timedelta(days=2)
                open_time = utl.get_market_open_time(next_monday)
                next_update_dt = dt.datetime.combine(next_monday,
                                                     open_time)

        next_update_dt += self._GRAN_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC

        return next_update_dt

    def do_timeout_event(self) -> None:
        # self.logger.debug("state:[{}]".format(self.state))

        if self.state == self.States.waiting:
            self._on_do_waiting()
        elif self.state == self.States.updating:
            self._on_do_updating()
        elif self.state == self.States.retrying:
            self._on_do_retrying()
        else:
            pass

    def _on_entry_waiting(self):
        self.logger.debug("--- <inst_id:[{}], gran_id:[{}]> Call \"{}\""
                          .format(self.inst_id, self.gran_id,
                                  sys._getframe().f_code.co_name))

        dt_now = dt.datetime.now()
        if self._is_update_complete:
            self.logger.debug("========== DF Update OK! ==========")
            latest_dt = self._get_latest_datetime_in_dataframe()
            self._next_updatetime = self._get_next_update_datetime(latest_dt)
            latest_sr = self._df_comp.iloc[-1]
            msg = LatestCandle()
            msg.candle.ask_o = latest_sr[ColName.ASK_OP.value]
            msg.candle.ask_h = latest_sr[ColName.ASK_HI.value]
            msg.candle.ask_l = latest_sr[ColName.ASK_LO.value]
            msg.candle.ask_c = latest_sr[ColName.ASK_CL.value]
            msg.candle.bid_o = latest_sr[ColName.BID_OP.value]
            msg.candle.bid_h = latest_sr[ColName.BID_HI.value]
            msg.candle.bid_l = latest_sr[ColName.BID_LO.value]
            msg.candle.bid_c = latest_sr[ColName.BID_CL.value]
            msg.candle.mid_o = latest_sr[ColName.MID_OP.value]
            msg.candle.mid_h = latest_sr[ColName.MID_HI.value]
            msg.candle.mid_l = latest_sr[ColName.MID_LO.value]
            msg.candle.mid_c = latest_sr[ColName.MID_CL.value]
            msg.candle.time = latest_dt.strftime(FMT_YMDHMS)
            msg.next_update_time = self._next_updatetime.strftime(FMT_YMDHMS)
            self._pub.publish(msg)
        else:
            self.logger.debug("========== DF Update NG! ==========")
            dt_now = dt_now.replace(second=0, microsecond=0)
            next_updatetime = dt_now + self._FAIL_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC
            if self._next_updatetime < next_updatetime:
                self._next_updatetime = next_updatetime

        self.logger.debug(" ---------- Next update time ----------")
        self.logger.debug(" - Now time:        [{}]".format(dt_now))
        self.logger.debug(" - Next update time:[{}]".format(self._next_updatetime))

    def _on_do_waiting(self):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        dt_now = dt.datetime.now()
        if self._next_updatetime < dt_now:
            self._trans_from_wating_to_updating()

    def _on_exit_waiting(self):
        self.logger.debug("--- <inst_id:[{}], gran_id:[{}]> Call \"{}\""
                          .format(self.inst_id, self.gran_id,
                                  sys._getframe().f_code.co_name))
        self._retry_counter = 0
        self._is_update_complete = False
        self._self_retry_counter = 0

    def _on_entry_updating(self):
        self.logger.debug("--- <inst_id:[{}], gran_id:[{}]> Call \"{}\""
                          .format(self.inst_id, self.gran_id,
                                  sys._getframe().f_code.co_name))
        self._future = None
        self._dt_to = dt.datetime.now()

    def _on_do_updating(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self.logger.debug(" - self_retry_counter:[{}]".format(self._self_retry_counter))
        self.logger.debug(" - retry_counter:[{}]".format(self._retry_counter))

        if self._future is None:
            latest_dt = self._get_latest_datetime_in_dataframe()
            dt_from = latest_dt + self._GRAN_INTERVAL
            dt_to = dt.datetime.now()
            self.logger.debug("  - time_from:[{}]".format(dt_from))
            self.logger.debug("  - time_to  :[{}]".format(dt_to))
            self._target_dt = dt_from

            try:
                self._future = self._request_async_candles(dt_from, dt_to)
                self._dt_to = dt_to
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Candles) "))
                self.logger.error("{}".format(err))
                self._trans_updating_common()
        else:
            if self._future.done():
                self.logger.debug("  Request done.")
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        self._update_dataframe(rsp.cndl_msg_list)
                        self.logger.debug("---------- df_comp(length:[{}]) ----------"
                                          .format(len(self._df_comp)))
                        self.logger.debug("  - Head:\n{}".format(self._df_comp[:5]))
                        self.logger.debug("  - Tail:\n{}".format(self._df_comp[-5:]))
                        self.logger.debug("---------- df_prov(length:[{}]) ----------"
                                          .format(len(self._df_prov)))
                        self.logger.debug("\n{}".format(self._df_prov))

                        if rsp.cndl_msg_list:
                            latest_dt = self._get_latest_datetime_in_dataframe()
                            self.logger.debug("  - target_dt <= latest_dt:[{}] <= [{}]"
                                              .format(self._target_dt, latest_dt))
                            if self._target_dt <= latest_dt:
                                self.logger.debug(" - Update complete!")
                                self._is_update_complete = True
                                self._trans_from_updating_to_waiting()
                            else:
                                self.logger.error(" !!!!!!!!!! Unexpected statement !!!!!!!!!!")
                                if self._SELF_RETRY_COUNT_MAX <= self._self_retry_counter:
                                    self._trans_updating_common()
                                else:
                                    self._self_retry_counter += 1
                                    self._trans_self_updating()
                        else:
                            self.logger.warn(" - rsp.cndl_msg_list is empty")
                            latest_dt = self._get_latest_datetime_in_dataframe()
                            close_time = utl.get_market_close_time(latest_dt.date())
                            if latest_dt.weekday() == WeekDay.SAT.value:
                                close_datetime = dt.datetime.combine(latest_dt.date(), close_time)
                                self.logger.debug(" - close datetime:[{}]".format(close_datetime))
                                if close_datetime < self._dt_to:
                                    self._next_updatetime = self._get_next_update_datetime(self._dt_to)
                                    self.logger.debug(" - Next update time:[{}]".format(self._next_updatetime))
                                    self._trans_from_updating_to_waiting()
                                else:
                                    self._trans_updating_common()
                            else:
                                self._trans_updating_common()
                    else:
                        self.logger.error("{:!^50}".format(" Call ROS Service Fail (Updating) "))
                        self._trans_updating_common()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Updating) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_updating_common()
            else:
                self.logger.debug("  Requesting now...")

    def _trans_updating_common(self):
        if self._RETRY_COUNT_MAX <= self._retry_counter:
            self._trans_from_updating_to_waiting()
        else:
            self._trans_from_updating_to_retrying()

    def _on_enrty_retrying(self):
        self.logger.debug("--- <inst_id:[{}], gran_id:[{}]> Call \"{}\""
                          .format(self.inst_id, self.gran_id,
                                  sys._getframe().f_code.co_name))
        dt_now = dt.datetime.now()
        dt_now = dt_now.replace(second=0, microsecond=0)
        self._next_updatetime = dt_now + self._RETRY_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC

        self.logger.debug("---------- Next retry update time ----------")
        self.logger.debug("  - Now time:        [{}]".format(dt_now))
        self.logger.debug("  - Next update time:[{}]".format(self._next_updatetime))

    def _on_do_retrying(self):
        dt_now = dt.datetime.now()
        if self._next_updatetime < dt_now:
            self._trans_from_retrying_to_updating()

    def _on_exit_retrying(self):
        self.logger.debug("--- <inst_id:[{}], gran_id:[{}]> Call \"{}\""
                          .format(self.inst_id, self.gran_id,
                                  sys._getframe().f_code.co_name))
        self._retry_counter += 1
        self._self_retry_counter = 0

    def _update_dataframe(self,
                          cndl_msg_list: List[Candle]
                          ) -> None:

        data = []
        for cndl_msg in cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, FMT_YMDHMS)
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
                         cndl_msg.is_complete
                         ])

        df = pd.DataFrame(data,
                          columns=ColName.to_list())
        df.set_index([ColName.DATETIME.value],
                     inplace=True)

        df_comp = df[(df[ColName.COMP.value])].copy()
        df_prov = df[~(df[ColName.COMP.value])].copy()

        if not df_comp.empty:
            df_comp.drop(ColName.COMP.value, axis=1, inplace=True)
            if self._df_comp.empty:
                self._df_comp = df_comp
            else:
                latest_idx = self._df_comp.index[-1]
                if latest_idx in df_comp.index:
                    row_pos = df_comp.index.get_loc(latest_idx)
                    df_comp = df_comp[row_pos + 1:]
                self._df_comp = self._df_comp.append(df_comp)
                droplist = self._df_comp.index[range(0, len(df_comp))]
                self._df_comp.drop(index=droplist, inplace=True)

        if df_prov.empty:
            self._df_prov = pd.DataFrame()
        else:
            df_prov.drop(ColName.COMP.value, axis=1, inplace=True)
            self._df_prov = df_prov

    def _request_async_candles(self,
                               dt_from: dt.datetime,
                               dt_to: dt.datetime
                               ) -> Future:
        req = CandlesSrv.Request()
        req.inst_msg.inst_id = self._inst_id
        req.gran_msg.gran_id = self._gran_id
        req.dt_from = dt_from.strftime(FMT_YMDHMS)
        req.dt_to = dt_to.strftime(FMT_YMDHMS)

        future = CandlesData.cli_cdl.call_async(req)

        return future


class HistoricalCandles(Node):

    def __init__(self) -> None:
        super().__init__("historical_candles")

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        CandlesData.logger = self.logger

        # Define Constant value.

        # Declare ROS parameter
        self._rosprm = _RosParams()
        self.declare_parameter(self._rosprm.ENA_INST_USDJPY.name)
        self.declare_parameter(self._rosprm.ENA_INST_EURJPY.name)
        self.declare_parameter(self._rosprm.ENA_INST_EURUSD.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M1.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M2.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M3.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M4.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M5.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M10.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M15.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M30.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H1.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H2.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H3.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H4.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H6.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H8.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H12.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_D.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_W.name)
        self.declare_parameter(self._rosprm.LENG_M1.name)
        self.declare_parameter(self._rosprm.LENG_M2.name)
        self.declare_parameter(self._rosprm.LENG_M3.name)
        self.declare_parameter(self._rosprm.LENG_M4.name)
        self.declare_parameter(self._rosprm.LENG_M5.name)
        self.declare_parameter(self._rosprm.LENG_M10.name)
        self.declare_parameter(self._rosprm.LENG_M15.name)
        self.declare_parameter(self._rosprm.LENG_M30.name)
        self.declare_parameter(self._rosprm.LENG_H1.name)
        self.declare_parameter(self._rosprm.LENG_H2.name)
        self.declare_parameter(self._rosprm.LENG_H3.name)
        self.declare_parameter(self._rosprm.LENG_H4.name)
        self.declare_parameter(self._rosprm.LENG_H6.name)
        self.declare_parameter(self._rosprm.LENG_H8.name)
        self.declare_parameter(self._rosprm.LENG_H12.name)
        self.declare_parameter(self._rosprm.LENG_D.name)
        self.declare_parameter(self._rosprm.LENG_W.name)

        # Set ROS parameter
        para = self.get_parameter(self._rosprm.ENA_INST_USDJPY.name)
        self._rosprm.ENA_INST_USDJPY.value = para.value
        para = self.get_parameter(self._rosprm.ENA_INST_EURJPY.name)
        self._rosprm.ENA_INST_EURJPY.value = para.value
        para = self.get_parameter(self._rosprm.ENA_INST_EURUSD.name)
        self._rosprm.ENA_INST_EURUSD.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M1.name)
        self._rosprm.ENA_GRAN_M1.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M2.name)
        self._rosprm.ENA_GRAN_M2.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M3.name)
        self._rosprm.ENA_GRAN_M3.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M4.name)
        self._rosprm.ENA_GRAN_M4.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M5.name)
        self._rosprm.ENA_GRAN_M5.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M10.name)
        self._rosprm.ENA_GRAN_M10.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M15.name)
        self._rosprm.ENA_GRAN_M15.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M30.name)
        self._rosprm.ENA_GRAN_M30.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H1.name)
        self._rosprm.ENA_GRAN_H1.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H2.name)
        self._rosprm.ENA_GRAN_H2.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H3.name)
        self._rosprm.ENA_GRAN_H3.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H4.name)
        self._rosprm.ENA_GRAN_H4.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H6.name)
        self._rosprm.ENA_GRAN_H6.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H8.name)
        self._rosprm.ENA_GRAN_H8.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H12.name)
        self._rosprm.ENA_GRAN_H12.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_D.name)
        self._rosprm.ENA_GRAN_D.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_W.name)
        self._rosprm.ENA_GRAN_W.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M1.name)
        self._rosprm.LENG_M1.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M2.name)
        self._rosprm.LENG_M2.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M3.name)
        self._rosprm.LENG_M3.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M4.name)
        self._rosprm.LENG_M4.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M5.name)
        self._rosprm.LENG_M5.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M10.name)
        self._rosprm.LENG_M10.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M15.name)
        self._rosprm.LENG_M15.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M30.name)
        self._rosprm.LENG_M30.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H1.name)
        self._rosprm.LENG_H1.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H2.name)
        self._rosprm.LENG_H2.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H3.name)
        self._rosprm.LENG_H3.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H4.name)
        self._rosprm.LENG_H4.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H6.name)
        self._rosprm.LENG_H6.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H8.name)
        self._rosprm.LENG_H8.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H12.name)
        self._rosprm.LENG_H12.value = para.value
        para = self.get_parameter(self._rosprm.LENG_D.name)
        self._rosprm.LENG_D.value = para.value
        para = self.get_parameter(self._rosprm.LENG_W.name)
        self._rosprm.LENG_W.value = para.value

        self.logger.debug("[Param]Enable instrument:")
        self.logger.debug("  - USD/JPY:[{}]".format(self._rosprm.ENA_INST_USDJPY.value))
        self.logger.debug("  - EUR/JPY:[{}]".format(self._rosprm.ENA_INST_EURJPY.value))
        self.logger.debug("  - EUR/USD:[{}]".format(self._rosprm.ENA_INST_EURUSD.value))
        self.logger.debug("[Param]Enable granularity:")
        self.logger.debug("  - M1: [{}]".format(self._rosprm.ENA_GRAN_M1.value))
        self.logger.debug("  - M2: [{}]".format(self._rosprm.ENA_GRAN_M2.value))
        self.logger.debug("  - M3: [{}]".format(self._rosprm.ENA_GRAN_M3.value))
        self.logger.debug("  - M4: [{}]".format(self._rosprm.ENA_GRAN_M4.value))
        self.logger.debug("  - M5: [{}]".format(self._rosprm.ENA_GRAN_M5.value))
        self.logger.debug("  - M10:[{}]".format(self._rosprm.ENA_GRAN_M10.value))
        self.logger.debug("  - M15:[{}]".format(self._rosprm.ENA_GRAN_M15.value))
        self.logger.debug("  - M30:[{}]".format(self._rosprm.ENA_GRAN_M30.value))
        self.logger.debug("  - H1: [{}]".format(self._rosprm.ENA_GRAN_H1.value))
        self.logger.debug("  - H2: [{}]".format(self._rosprm.ENA_GRAN_H2.value))
        self.logger.debug("  - H3: [{}]".format(self._rosprm.ENA_GRAN_H3.value))
        self.logger.debug("  - H4: [{}]".format(self._rosprm.ENA_GRAN_H4.value))
        self.logger.debug("  - H6: [{}]".format(self._rosprm.ENA_GRAN_H6.value))
        self.logger.debug("  - H8: [{}]".format(self._rosprm.ENA_GRAN_H8.value))
        self.logger.debug("  - H12:[{}]".format(self._rosprm.ENA_GRAN_H12.value))
        self.logger.debug("  - D:  [{}]".format(self._rosprm.ENA_GRAN_D.value))
        self.logger.debug("  - W:  [{}]".format(self._rosprm.ENA_GRAN_W.value))
        self.logger.debug("[Param]Data length:")
        self.logger.debug("  - M1: [{}]".format(self._rosprm.LENG_M1.value))
        self.logger.debug("  - M2: [{}]".format(self._rosprm.LENG_M2.value))
        self.logger.debug("  - M3: [{}]".format(self._rosprm.LENG_M3.value))
        self.logger.debug("  - M4: [{}]".format(self._rosprm.LENG_M4.value))
        self.logger.debug("  - M5: [{}]".format(self._rosprm.LENG_M5.value))
        self.logger.debug("  - M10:[{}]".format(self._rosprm.LENG_M10.value))
        self.logger.debug("  - M15:[{}]".format(self._rosprm.LENG_M15.value))
        self.logger.debug("  - M30:[{}]".format(self._rosprm.LENG_M30.value))
        self.logger.debug("  - H1: [{}]".format(self._rosprm.LENG_H1.value))
        self.logger.debug("  - H2: [{}]".format(self._rosprm.LENG_H2.value))
        self.logger.debug("  - H3: [{}]".format(self._rosprm.LENG_H3.value))
        self.logger.debug("  - H4: [{}]".format(self._rosprm.LENG_H4.value))
        self.logger.debug("  - H6: [{}]".format(self._rosprm.LENG_H6.value))
        self.logger.debug("  - H8: [{}]".format(self._rosprm.LENG_H8.value))
        self.logger.debug("  - H12:[{}]".format(self._rosprm.LENG_H12.value))
        self.logger.debug("  - D:  [{}]".format(self._rosprm.LENG_D.value))
        self.logger.debug("  - W:  [{}]".format(self._rosprm.LENG_W.value))

        try:
            # Create service client "Candles"
            srv_type = CandlesSrv
            srv_name = "candles"
            CandlesData.cli_cdl = self._create_service_client(srv_type, srv_name)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Exception "))
            self.logger.error(err)
            raise InitializerErrorException("create service client failed.")

        self._candles_data_list = []
        for gran_data in self._rosprm.enable_gran_list():
            for inst_id in self._rosprm.enable_inst_list():
                candles_data = CandlesData(self, inst_id, gran_data)
                self._candles_data_list.append(candles_data)

        # Create service server "CandlesByDatetime"
        srv_type = CandlesByDatetimeSrv
        srv_name = "candles_by_datetime"
        callback = self._on_recv_candles_by_datetime
        self._hc_srv = self.create_service(srv_type,
                                           srv_name,
                                           callback=callback,
                                           callback_group=ReentrantCallbackGroup()
                                           )

        # Create service server "CandlesByLength"
        srv_type = CandlesByLengthSrv
        srv_name = "candles_by_length"
        callback = self._on_recv_candles_by_length
        self._hc_srv = self.create_service(srv_type,
                                           srv_name,
                                           callback=callback,
                                           callback_group=ReentrantCallbackGroup()
                                           )

    def do_timeout_event(self) -> None:
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        for candles_data in self._candles_data_list:
            candles_data.do_timeout_event()
            """
            self.logger.debug("inst_id:{}, gran_id:{}"
                              .format(candles_data._inst_id, candles_data._gran_id))
            """

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self.logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

    async def _on_recv_candles_by_datetime(self,
                                           req: SrvTypeRequest,
                                           rsp: SrvTypeResponse
                                           ) -> SrvTypeResponse:
        self.logger.debug("{:=^50}".format(" Service[candles_by_datetime]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - datetime_start:[{}]".format(req.datetime_start))
        self.logger.debug("  - datetime_end:[{}]".format(req.datetime_end))
        self.logger.debug("  - dayofweeks:[{}]".format(req.dayofweeks))
        self.logger.debug("  - time_from:[{}]".format(req.time_from))
        self.logger.debug("  - time_to:[{}]".format(req.time_to))

        inst_id = INST_DICT[req.inst_msg.inst_id]
        gran_id = GRAN_DICT[req.gran_msg.gran_id]

        dbg_tm_start = dt.datetime.now()

        if ((gran_id == GranApi.GRAN_D) or (req.time_from == "")):
            start_time = None
        else:
            start_time = dt.datetime.strptime(req.time_from, FMT_TIME_HMS).time()

        if ((gran_id == GranApi.GRAN_D) or (req.time_to == "")):
            end_time = None
        else:
            end_time = dt.datetime.strptime(req.time_to, FMT_TIME_HMS).time()

        df_comp = None
        for candles_data in self._candles_data_list:
            if ((inst_id == candles_data.inst_id) and (gran_id == candles_data.gran_id)):
                df_comp = candles_data.df_comp
                next_updatetime = candles_data.next_updatetime
                break

        rsp.cndl_msg_list = []
        if df_comp is not None:
            if not req.datetime_start == "":
                start_dt = dt.datetime.strptime(req.datetime_start, FMT_YMDHMS)
                if gran_id == GranApi.GRAN_D:
                    start_dt = dt.datetime.combine(start_dt.date(), dt.time(6, 0))
                df_comp = df_comp.loc[start_dt:]

            if not req.datetime_end == "":
                end_dt = dt.datetime.strptime(req.datetime_end, FMT_YMDHMS)
                if gran_id == GranApi.GRAN_D:
                    end_dt = dt.datetime.combine(end_dt.date(), dt.time(7, 0))
                df_comp = df_comp.loc[:end_dt]

            if req.dayofweeks:
                cond = [i in req.dayofweeks for i in df_comp.index.dayofweek]
                df_comp = df_comp[cond]

            if ((start_time is not None) and (end_time is not None)):
                df_comp = df_comp.between_time(start_time, end_time)
            elif ((start_time is not None) and (end_time is None)):
                df_comp = df_comp.between_time(start_time, MAX_TIME)
            elif ((start_time is None) and (end_time is not None)):
                df_comp = df_comp.between_time(MIN_TIME, end_time)
            else:
                pass

            if not df_comp.empty:
                """
                for idx, sr in df_comp.iterrows():
                    msg = Candle()
                    msg.ask_o = sr[ColName.ASK_OP.value]
                    msg.ask_h = sr[ColName.ASK_HI.value]
                    msg.ask_l = sr[ColName.ASK_LO.value]
                    msg.ask_c = sr[ColName.ASK_CL.value]
                    msg.mid_o = sr[ColName.MID_OP.value]
                    msg.mid_h = sr[ColName.MID_HI.value]
                    msg.mid_l = sr[ColName.MID_LO.value]
                    msg.mid_c = sr[ColName.MID_CL.value]
                    msg.bid_o = sr[ColName.BID_OP.value]
                    msg.bid_h = sr[ColName.BID_HI.value]
                    msg.bid_l = sr[ColName.BID_LO.value]
                    msg.bid_c = sr[ColName.BID_CL.value]
                    msg.time = idx.strftime(FMT_YMDHMS)
                    rsp.cndl_msg_list.append(msg)
                """
                for t in df_comp.itertuples():
                    msg = Candle()
                    msg.ask_o = t.ask_op
                    msg.ask_h = t.ask_hi
                    msg.ask_l = t.ask_lo
                    msg.ask_c = t.ask_cl
                    msg.mid_o = t.mid_op
                    msg.mid_h = t.mid_hi
                    msg.mid_l = t.mid_lo
                    msg.mid_c = t.mid_cl
                    msg.bid_o = t.bid_op
                    msg.bid_h = t.bid_hi
                    msg.bid_l = t.bid_lo
                    msg.bid_c = t.bid_cl
                    msg.time = t.Index.strftime(FMT_YMDHMS)
                    rsp.cndl_msg_list.append(msg)

        rsp.next_update_time = next_updatetime.strftime(FMT_YMDHMS)

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
        self.logger.debug("  - next_update_time:[{}]".format(rsp.next_update_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_by_datetime]:End "))

        return rsp

    async def _on_recv_candles_by_length(self,
                                         req: SrvTypeRequest,
                                         rsp: SrvTypeResponse
                                         ) -> SrvTypeResponse:
        self.logger.debug("{:=^50}".format(" Service[candles_by_length]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - length:[{}]".format(req.length))

        inst_id = INST_DICT[req.inst_msg.inst_id]
        gran_id = GRAN_DICT[req.gran_msg.gran_id]

        dbg_tm_start = dt.datetime.now()

        df_comp = None
        for candles_data in self._candles_data_list:
            if ((inst_id == candles_data.inst_id) and (gran_id == candles_data.gran_id)):
                df_comp = candles_data._df_comp
                next_updatetime = candles_data.next_updatetime
                break

        rsp.cndl_msg_list = []
        if ((df_comp is not None) and (0 < req.length)):
            df_comp = df_comp.tail(req.length)

            if not df_comp.empty:
                """
                for idx, sr in df_comp.iterrows():
                    msg = Candle()
                    msg.ask_o = sr[ColName.ASK_OP.value]
                    msg.ask_h = sr[ColName.ASK_HI.value]
                    msg.ask_l = sr[ColName.ASK_LO.value]
                    msg.ask_c = sr[ColName.ASK_CL.value]
                    msg.mid_o = sr[ColName.MID_OP.value]
                    msg.mid_h = sr[ColName.MID_HI.value]
                    msg.mid_l = sr[ColName.MID_LO.value]
                    msg.mid_c = sr[ColName.MID_CL.value]
                    msg.bid_o = sr[ColName.BID_OP.value]
                    msg.bid_h = sr[ColName.BID_HI.value]
                    msg.bid_l = sr[ColName.BID_LO.value]
                    msg.bid_c = sr[ColName.BID_CL.value]
                    msg.time = idx.strftime(FMT_YMDHMS)
                    rsp.cndl_msg_list.append(msg)
                """
                for t in df_comp.itertuples():
                    msg = Candle()
                    msg.ask_o = t.ask_op
                    msg.ask_h = t.ask_hi
                    msg.ask_l = t.ask_lo
                    msg.ask_c = t.ask_cl
                    msg.mid_o = t.mid_op
                    msg.mid_h = t.mid_hi
                    msg.mid_l = t.mid_lo
                    msg.mid_c = t.mid_cl
                    msg.bid_o = t.bid_op
                    msg.bid_h = t.bid_hi
                    msg.bid_l = t.bid_lo
                    msg.bid_c = t.bid_cl
                    msg.time = t.Index.strftime(FMT_YMDHMS)
                    rsp.cndl_msg_list.append(msg)

        rsp.next_update_time = next_updatetime.strftime(FMT_YMDHMS)

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list)))
        self.logger.debug("  - next_update_time:[{}]".format(rsp.next_update_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_by_length]:End "))

        return rsp


def main(args=None):

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    try:
        hc = HistoricalCandles()
    except InitializerErrorException:
        pass
    else:
        try:
            while rclpy.ok():
                rclpy.spin_once(hc, executor=executor, timeout_sec=1.0)
                hc.do_timeout_event()
        except KeyboardInterrupt:
            pass

        hc.destroy_node()

    rclpy.shutdown()
