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
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
import trade_manager.utility as utl
from trade_manager.utility import RosParam
from trade_manager.constant import Transitions as Tr
from trade_manager.constant import FMT_YMDHMS
from trade_manager.constant import GranParam
from trade_manager.constant import CandleColumnNames
from trade_manager.exception import InitializerErrorException
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


@dataclass
class DailyParam():
    """
    Daily Parameter.
    """
    today: dt.date = None
    open_time: dt.time = None
    close_time: dt.time = None


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
                Tr.ON_ENTER.value: None,
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
        # self._next_update_time = self._get_next_update_time(gran_id, dt_now)

        self.logger.debug("{:-^40}".format(" Create CandlesData:Start "))
        self.logger.debug("  - inst_id:[{}]".format(self._inst_id))
        self.logger.debug("  - gran_id:[{}]".format(self._gran_id))

        gran_param = GranParam.get_member_by_msgid(self._gran_id)
        self._interval = gran_param.timedelta

        dt_now = dt.datetime.now()
        dt_from = dt_now - self._interval * gran_data.length
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

        CandlesData.daily_param.open_time
        CandlesData.daily_param.close_time

    def do_timeout_event(self) -> None:
        self.logger.debug("state:[{}]".format(self.state))

        if self.state == self.States.waiting:
            self._on_do_waiting()
        elif self.state == self.States.updating:
            self._on_do_updating()
        elif self.state == self.States.retrying:
            self._on_do_retrying()
        else:
            pass

    def _on_entry_waiting(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        # TODO:
        self._next_update_time = None

    def _on_do_waiting(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

    def _on_exit_waiting(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._retry_counter = 0

    def _on_entry_updating(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None

    def _on_do_updating(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if self._future is None:
            dt_to = dt.datetime.now()
            # TODO:
            # dt_from = dt_now - self._interval * gran_data.length

            self.logger.debug("  - time_from:[{}]".format(dt_from))
            self.logger.debug("  - time_to  :[{}]".format(dt_to))

            try:
                self._future = self._request_async_candles(dt_from, dt_to)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Candles) "))
                self.logger.error("{}".format(err))
                self._trans_from_updating_to_retrying()
        else:
            if self._future.done():
                self.logger.debug("  Request done.")
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        self._update_dataframe(rsp.cndl_msg_list)
                        self._trans_from_updating_to_waiting()
                    else:
                        self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Create) "))
                        self._trans_from_updating_to_retrying()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_from_updating_to_retrying()
            else:
                self.logger.debug("  Requesting now...")

    def _on_do_retrying(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

    def _on_exit_retrying(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._retry_counter += 1

    def _update_dataframe(self,
                          cndl_msg_list: List[Candle]
                          ) -> None:

        data = []
        for cndl_msg in cndl_msg_list:
            dt_ = cndl_msg.time.split("T")
            date_ = dt_[0]
            time_ = dt_[1]
            hsc_o = (cndl_msg.ask_o - cndl_msg.bid_o) / 2
            hsc_h = (cndl_msg.ask_h - cndl_msg.bid_h) / 2
            hsc_l = (cndl_msg.ask_l - cndl_msg.bid_l) / 2
            hsc_c = (cndl_msg.ask_c - cndl_msg.bid_c) / 2
            data.append([date_,
                         time_,
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
                          columns=CandleColumnNames.to_list())
        df.set_index([CandleColumnNames.DATE.value,
                      CandleColumnNames.TIME.value],
                     inplace=True)

        df_comp = df[(df[CandleColumnNames.COMP.value])]
        df_prov = df[~(df[CandleColumnNames.COMP.value])]

        if not df_comp.empty:
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

        CandlesData.daily_param = self._create_daily_param()
        self.logger.debug("----- update \"daily param\" -----")
        self.logger.debug("{}".format(CandlesData.daily_param))

        self._candles_data_list = []
        for inst_id in self._rosprm.enable_inst_list():
            for gran_data in self._rosprm.enable_gran_list():
                candles_data = CandlesData(self, inst_id, gran_data)
                self._candles_data_list.append(candles_data)

    def do_timeout_event(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        currday = dt.datetime.now().date()
        if CandlesData.daily_param.today != currday:
            CandlesData.daily_param = self._create_daily_param(currday)
            self.logger.debug("----- update \"daily param\" -----")
            self.logger.debug("{}".format(CandlesData.daily_param))

        for candles_data in self._candles_data_list:
            candles_data.do_timeout_event()
            self.logger.debug("inst_id:{}, gran_id:{}"
                              .format(candles_data._inst_id, candles_data._gran_id))

    def _create_daily_param(self, date: dt.date=None) -> DailyParam:

        if date is None:
            date = dt.date.today()

        if utl.is_summer_time(date):
            open_time = dt.time(6, 0)
            close_time = dt.time(6, 0)
        else:
            open_time = dt.time(7, 0)
            close_time = dt.time(7, 0)

        return DailyParam(date, open_time, close_time)

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli


def main(args=None):

    rclpy.init(args=args)

    try:
        hc = HistoricalCandles()
    except InitializerErrorException:
        pass
    else:
        try:
            while rclpy.ok():
                rclpy.spin_once(hc, timeout_sec=1.0)
                hc.do_timeout_event()
        except KeyboardInterrupt:
            pass

        hc.destroy_node()

    rclpy.shutdown()
