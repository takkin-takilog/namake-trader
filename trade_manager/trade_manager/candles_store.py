import sys
from typing import TypeVar
from enum import Enum, auto
import threading
import datetime as dt
import pandas as pd
from transitions import Machine

# from transitions.extensions.factory import GraphMachine as Machine
from transitions.extensions.factory import GraphMachine
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from api_server_msgs.srv import CandlesQuerySrv
from api_server_msgs.msg import Instrument as InstApi
from api_server_msgs.msg import Granularity as GranApi
from trade_manager_msgs.srv import CandlesByDatetimeSrv
from trade_manager_msgs.srv import CandlesByLengthSrv
from trade_manager_msgs.msg import Candle
from trade_manager_msgs.msg import LatestCandle
from .constant import FMT_YMDHMS, FMT_TIME_HMS
from .constant import MIN_TIME, MAX_TIME
from .constant import Transitions as Tr
from .constant import WeekDay
from .constant import CandleColumnNames as ColNames
from .constant import INST_DICT, GRAN_DICT
from .exception import InitializerErrorException, RosServiceErrorException
from .dataclass import RosParam
from .parameter import GranParam, InstParam
from . import utils as utl
from . import ros_utils as rosutl
from .wrapper import RosServiceClient, FutureWithTimeout

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class _CandlesElement:
    """
    Candles element class.

    """

    class States(Enum):
        waiting = auto()
        updating = auto()
        retrying = auto()

    def __init__(
        self,
        node: Node,
        srvcli: RosServiceClient,
        inst_id: int,
        gran_id: int,
        gran_leng: int,
        next_updatetime_ofs_sec: int,
        retry_interval_min: int,
        fail_interval_min: int,
        retry_count_max: int,
        self_retry_count_max: int,
    ) -> None:
        # --------------- Set logger lebel ---------------
        self.logger = node.get_logger()

        # --------------- Define Constant value ---------------
        gran_param = GranParam.get_member_by_msgid(gran_id)
        self._GRAN_INTERVAL = gran_param.timedelta
        self._NEXT_UPDATETIME_OFS_SEC = dt.timedelta(seconds=next_updatetime_ofs_sec)
        self._RETRY_INTERVAL = dt.timedelta(minutes=retry_interval_min)
        self._FAIL_INTERVAL = dt.timedelta(minutes=fail_interval_min)
        self._RETRY_COUNT_MAX = retry_count_max
        self._SELF_RETRY_COUNT_MAX = self_retry_count_max

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME: self.States.waiting,
                Tr.ON_ENTER: "_on_enter_waiting",
                Tr.ON_EXIT: "_on_exit_waiting",
            },
            {
                Tr.NAME: self.States.updating,
                Tr.ON_ENTER: "_on_enter_updating",
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.retrying,
                Tr.ON_ENTER: "_on_enter_retrying",
                Tr.ON_EXIT: "_on_exit_retrying",
            },
        ]

        transitions = [
            {
                Tr.TRIGGER: "_trans_from_wating_to_updating",
                Tr.SOURCE: self.States.waiting,
                Tr.DEST: self.States.updating,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_updating_to_retrying",
                Tr.SOURCE: self.States.updating,
                Tr.DEST: self.States.retrying,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_retrying_to_updating",
                Tr.SOURCE: self.States.retrying,
                Tr.DEST: self.States.updating,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_updating_to_waiting",
                Tr.SOURCE: self.States.updating,
                Tr.DEST: self.States.waiting,
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

        self._sm = Machine(
            model=self,
            states=states,
            initial=self.States.waiting,
            transitions=transitions,
        )

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        # --------------- Declare instance variable ---------------
        self._weekend_close_time: dt.datetime
        self._request_start_dt: dt.datetime
        self._request_end_dt: dt.datetime

        # --------------- Initialize instance variable ---------------
        self._srvcli = srvcli
        self._inst_id = inst_id
        self._gran_id = gran_id
        self._df_comp: pd.DataFrame = pd.DataFrame()
        self._df_prov: pd.DataFrame = pd.DataFrame()
        self._retry_counter = 0
        self._is_update_complete = True
        self._self_retry_counter = 0
        self._needs_weekend_update = False
        self._fwt: FutureWithTimeout | None = None
        dt_now = dt.datetime.now()
        self._next_updatetime = (
            dt_now + self._FAIL_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC
        )
        self._lock = threading.Lock()

        self.logger.debug("{:-^40}".format(" Create CandlesDataFrame:Start "))
        self.logger.debug("  - inst_id:[{}]".format(self._inst_id))
        self.logger.debug("  - gran_id:[{}]".format(self._gran_id))
        self.logger.debug("  - gran_leng:[{}]".format(gran_leng))

        # --------------- Create Candles(OHLC) DataFrame ---------------
        dt_from = dt_now - self._GRAN_INTERVAL * gran_leng
        dt_to = dt_now

        self.logger.debug("  - time_from:[{}]".format(dt_from))
        self.logger.debug("  - time_to  :[{}]".format(dt_to))

        req = CandlesQuerySrv.Request()
        req.inst_msg.inst_id = self._inst_id
        req.gran_msg.gran_id = self._gran_id
        req.dt_from = dt_from.strftime(FMT_YMDHMS)
        req.dt_to = dt_to.strftime(FMT_YMDHMS)
        try:
            rsp = self._srvcli.call(req)  # type: ignore[var-annotated]
        except RosServiceErrorException as err:
            self.logger.error(
                "{:!^50}".format(" Call ROS Service Error (CandlesQuery) ")
            )
            self.logger.error("{}".format(err))
            raise InitializerErrorException(
                "[CandlesDataFrame] initialize failed."
            ) from err

        if rsp.result is True:
            self._update_dataframe(rsp.cndl_msg_list)
            self.logger.debug(
                "---------- df_comp(length:[{}]) ----------".format(len(self._df_comp))
            )
            self.logger.debug("  - Head:\n{}".format(self._df_comp[:5]))
            self.logger.debug("  - Tail:\n{}".format(self._df_comp[-5:]))
            self.logger.debug(
                "---------- df_prov(length:[{}]) ----------".format(len(self._df_prov))
            )
            self.logger.debug("\n{}".format(self._df_prov))
        else:
            self.logger.error(
                "{:!^50}".format(" Call ROS Service Error (CandlesQuery) ")
            )
            self.logger.error("  future result is False")
            raise InitializerErrorException("[CandlesDataFrame] initialize failed.")

        # --------------- Create ROS Communication ---------------
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )
        inst_name = InstParam.get_member_by_msgid(self._inst_id).namespace
        gran_name = GranParam.get_member_by_msgid(self._gran_id).namespace
        TPCNM_LATEST_CANDLE = inst_name + "_" + gran_name + "_latest_candle"
        self._pub = node.create_publisher(
            LatestCandle, TPCNM_LATEST_CANDLE, qos_profile
        )

        # --------------- Initial process ---------------
        self._on_enter_waiting()

    @property
    def inst_id(self) -> int:
        return self._inst_id

    @property
    def gran_id(self) -> int:
        return self._gran_id

    @property
    def df_comp(self) -> pd.DataFrame:
        with self._lock:
            df_comp = self._df_comp
        return df_comp

    @property
    def next_updatetime(self) -> dt.datetime:
        return self._next_updatetime

    def _get_latest_datetime_in_dataframe(self) -> dt.datetime:
        return self._df_comp.index[-1].to_pydatetime()  # type: ignore[no-any-return]

    def _get_next_update_datetime(self, latest_datetime: dt.datetime) -> dt.datetime:
        next_update_dt: dt.datetime = latest_datetime + self._GRAN_INTERVAL

        if next_update_dt.weekday() == WeekDay.SAT:
            close_time = utl.get_market_close_time(next_update_dt.date())
            if close_time <= next_update_dt.time():
                self.logger.debug(" - Weekly Data Complete!")
                next_monday = next_update_dt.date() + dt.timedelta(days=2)
                open_time = utl.get_market_open_time(next_monday)
                next_update_dt = dt.datetime.combine(next_monday, open_time)

        next_update_dt += self._GRAN_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC

        # Fail safe - optimize next update time
        now_dt = dt.datetime.now()
        if next_update_dt < now_dt:
            self.logger.warn(
                "{:!^50}".format(" Next-update-time under the current time ")
            )
            self.logger.warn(" - Next-update-time:[{}]".format(next_update_dt))
            self.logger.warn(" - Current time:[{}]".format(now_dt))
            while next_update_dt < now_dt:
                next_update_dt += self._GRAN_INTERVAL
            self.logger.warn(
                " - Fail safe: optimize Next-update-time[{}]".format(next_update_dt)
            )

        return next_update_dt

    def do_cyclic_event(self) -> None:
        # self.logger.debug("state:[{}]".format(self.state))

        if self.state == self.States.waiting:
            self._on_do_waiting()
        elif self.state == self.States.updating:
            self._on_do_updating()
        elif self.state == self.States.retrying:
            self._on_do_retrying()
        else:
            pass

    def _on_enter_waiting(self) -> None:
        self.logger.debug(
            "--- <inst_id:[{}], gran_id:[{}]> Call [{}]".format(
                self.inst_id,
                self.gran_id,
                sys._getframe().f_code.co_name,
            )
        )

        dt_now = dt.datetime.now()
        if self._is_update_complete:
            self.logger.debug("========== DF Update OK! ==========")
            msg = LatestCandle()
            if self._needs_weekend_update:
                self.logger.debug("  ----- Needs weekend update -----")
                next_updatetime = self._get_next_update_datetime(
                    self._weekend_close_time
                )
                latest_sr = self._df_comp.iloc[-1]
                latest_close_ask = latest_sr[ColNames.ASK_CL]
                latest_close_bid = latest_sr[ColNames.BID_CL]
                latest_close_mid = latest_sr[ColNames.MID_CL]
                time = self._weekend_close_time - self._GRAN_INTERVAL
                msg.candle.ask_o = latest_close_ask
                msg.candle.ask_h = latest_close_ask
                msg.candle.ask_l = latest_close_ask
                msg.candle.ask_c = latest_close_ask
                msg.candle.bid_o = latest_close_bid
                msg.candle.bid_h = latest_close_bid
                msg.candle.bid_l = latest_close_bid
                msg.candle.bid_c = latest_close_bid
                msg.candle.mid_o = latest_close_mid
                msg.candle.mid_h = latest_close_mid
                msg.candle.mid_l = latest_close_mid
                msg.candle.mid_c = latest_close_mid
                msg.candle.time = time.strftime(FMT_YMDHMS)
                msg.next_update_time = next_updatetime.strftime(FMT_YMDHMS)
                self._needs_weekend_update = False
            else:
                latest_dt = self._get_latest_datetime_in_dataframe()
                next_updatetime = self._get_next_update_datetime(latest_dt)
                latest_sr = self._df_comp.iloc[-1]
                msg.candle.ask_o = latest_sr[ColNames.ASK_OP]
                msg.candle.ask_h = latest_sr[ColNames.ASK_HI]
                msg.candle.ask_l = latest_sr[ColNames.ASK_LO]
                msg.candle.ask_c = latest_sr[ColNames.ASK_CL]
                msg.candle.bid_o = latest_sr[ColNames.BID_OP]
                msg.candle.bid_h = latest_sr[ColNames.BID_HI]
                msg.candle.bid_l = latest_sr[ColNames.BID_LO]
                msg.candle.bid_c = latest_sr[ColNames.BID_CL]
                msg.candle.mid_o = latest_sr[ColNames.MID_OP]
                msg.candle.mid_h = latest_sr[ColNames.MID_HI]
                msg.candle.mid_l = latest_sr[ColNames.MID_LO]
                msg.candle.mid_c = latest_sr[ColNames.MID_CL]
                msg.candle.time = latest_dt.strftime(FMT_YMDHMS)
                msg.next_update_time = next_updatetime.strftime(FMT_YMDHMS)

            self._next_updatetime = next_updatetime
            self._pub.publish(msg)
        else:
            self.logger.debug("========== DF Update NG! ==========")
            dt_now = dt_now.replace(second=0, microsecond=0)
            self._next_updatetime = (
                dt_now + self._FAIL_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC
            )

        self.logger.debug(" ---------- Next update time ----------")
        self.logger.debug(" - Now time:        [{}]".format(dt_now))
        self.logger.debug(" - Next update time:[{}]".format(self._next_updatetime))

    def _on_do_waiting(self) -> None:
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        dt_now = dt.datetime.now()
        if self._next_updatetime < dt_now:
            self._trans_from_wating_to_updating()

    def _on_exit_waiting(self) -> None:
        self.logger.debug(
            "--- <inst_id:[{}], gran_id:[{}]> Call [{}]".format(
                self.inst_id,
                self.gran_id,
                sys._getframe().f_code.co_name,
            )
        )
        self._retry_counter = 0
        self._is_update_complete = False
        self._self_retry_counter = 0

    def _on_enter_updating(self) -> None:
        self.logger.debug(
            "--- <inst_id:[{}], gran_id:[{}]> Call [{}]".format(
                self.inst_id,
                self.gran_id,
                sys._getframe().f_code.co_name,
            )
        )
        dt_from = self._get_latest_datetime_in_dataframe() + self._GRAN_INTERVAL
        dt_to = dt.datetime.now()
        self.logger.debug("  - time_from:[{}]".format(dt_from))
        self.logger.debug("  - time_to  :[{}]".format(dt_to))
        self._fwt = None

        req = CandlesQuerySrv.Request()
        req.inst_msg.inst_id = self._inst_id
        req.gran_msg.gran_id = self._gran_id
        req.dt_from = dt_from.strftime(FMT_YMDHMS)
        req.dt_to = dt_to.strftime(FMT_YMDHMS)
        try:
            self._fwt = self._srvcli.call_async(req)
            self._request_start_dt = dt_from
            self._request_end_dt = dt_to
        except RosServiceErrorException as err:
            self.logger.error(
                "{:!^50}".format(" Call ROS Service Error (CandlesQuery) ")
            )
            self.logger.error("{}".format(err))

    def _on_do_updating(self) -> None:
        self.logger.debug("----- Call [{}]".format(sys._getframe().f_code.co_name))
        self.logger.debug(" - self_retry_counter:[{}]".format(self._self_retry_counter))
        self.logger.debug(" - retry_counter:[{}]".format(self._retry_counter))

        if self._fwt is None:
            self._trans_updating_common()
            return

        if not self._fwt.future.done():
            self.logger.debug("  Requesting now...")
            return

        self.logger.debug("  Request done.")
        rsp = self._fwt.future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Updating) "))
            self.logger.error("  future.result() is None.")
            self._trans_updating_common()
            return

        if not rsp.result:
            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Updating) "))
            self._trans_updating_common()
            return

        self._update_dataframe(rsp.cndl_msg_list)
        self.logger.debug(
            "---------- df_comp(length:[{}]) ----------".format(len(self._df_comp))
        )
        self.logger.debug("  - Head:\n{}".format(self._df_comp[:5]))
        self.logger.debug("  - Tail:\n{}".format(self._df_comp[-5:]))
        self.logger.debug(
            "---------- df_prov(length:[{}]) ----------".format(len(self._df_prov))
        )
        self.logger.debug("\n{}".format(self._df_prov))

        if rsp.cndl_msg_list:
            latest_dt = self._get_latest_datetime_in_dataframe()
            self.logger.debug(
                "  - target_dt <= latest_dt:[{}] <= [{}]".format(
                    self._request_start_dt, latest_dt
                )
            )
            if self._request_start_dt <= latest_dt:
                self.logger.debug(" - Update complete!")
                self._is_update_complete = True
                self._trans_from_updating_to_waiting()
            else:
                self.logger.warn("{:!^50}".format(" Unexpected statement "))
                if self._SELF_RETRY_COUNT_MAX <= self._self_retry_counter:
                    self._trans_updating_common()
                else:
                    self._self_retry_counter += 1
                    self._trans_self_updating()
        else:
            self.logger.warn(" - rsp.cndl_msg_list is empty")
            latest_dt = self._get_latest_datetime_in_dataframe()
            close_time = utl.get_market_close_time(latest_dt.date())
            if latest_dt.weekday() == WeekDay.SAT:
                close_datetime = dt.datetime.combine(latest_dt.date(), close_time)
                self.logger.debug(" - close datetime:[{}]".format(close_datetime))
                if close_datetime < self._request_end_dt:
                    self.logger.debug(" - dt_to:[{}]".format(self._request_end_dt))
                    self._weekend_close_time = close_datetime
                    self._is_update_complete = True
                    self._needs_weekend_update = True
                    self._trans_from_updating_to_waiting()
                else:
                    self._trans_updating_common()
            else:
                self._trans_updating_common()

    def _trans_updating_common(self) -> None:
        if self._RETRY_COUNT_MAX <= self._retry_counter:
            self._trans_from_updating_to_waiting()
        else:
            self._trans_from_updating_to_retrying()

    def _on_enter_retrying(self) -> None:
        self.logger.debug(
            "--- <inst_id:[{}], gran_id:[{}]> Call [{}]".format(
                self.inst_id,
                self.gran_id,
                sys._getframe().f_code.co_name,
            )
        )
        dt_now = dt.datetime.now()
        dt_now = dt_now.replace(second=0, microsecond=0)
        self._next_updatetime = (
            dt_now + self._RETRY_INTERVAL + self._NEXT_UPDATETIME_OFS_SEC
        )

        self.logger.debug("---------- Next retry update time ----------")
        self.logger.debug("  - Now time:        [{}]".format(dt_now))
        self.logger.debug("  - Next update time:[{}]".format(self._next_updatetime))

    def _on_do_retrying(self) -> None:
        dt_now = dt.datetime.now()
        if self._next_updatetime < dt_now:
            self._trans_from_retrying_to_updating()

    def _on_exit_retrying(self) -> None:
        self.logger.debug(
            "--- <inst_id:[{}], gran_id:[{}]> Call [{}]".format(
                self.inst_id,
                self.gran_id,
                sys._getframe().f_code.co_name,
            )
        )
        self._retry_counter += 1
        self._self_retry_counter = 0

    def _update_dataframe(self, cndl_msg_list: list[Candle]) -> None:
        data = []
        for cndl_msg in cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, FMT_YMDHMS)
            hsc_o = (cndl_msg.ask_o - cndl_msg.bid_o) / 2
            hsc_h = (cndl_msg.ask_h - cndl_msg.bid_h) / 2
            hsc_l = (cndl_msg.ask_l - cndl_msg.bid_l) / 2
            hsc_c = (cndl_msg.ask_c - cndl_msg.bid_c) / 2
            data.append(
                [
                    dt_,
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
                    cndl_msg.is_complete,
                ]
            )

        df = pd.DataFrame(data, columns=ColNames.to_list())
        df.set_index([ColNames.DATETIME], inplace=True)

        df_comp = df[(df[ColNames.COMP])].copy()
        df_prov = df[~(df[ColNames.COMP])].copy()

        with self._lock:
            if not df_comp.empty:
                df_comp.drop(columns=ColNames.COMP, inplace=True)
                if self._df_comp.empty:
                    self._df_comp = df_comp
                else:
                    latest_idx = self._df_comp.index[-1]
                    if latest_idx in df_comp.index:
                        start_pos = df_comp.index.get_loc(latest_idx) + 1
                        df_comp = df_comp[start_pos:]
                    self._df_comp = pd.concat([self._df_comp, df_comp])
                    droplist = self._df_comp.index[range(0, len(df_comp))]
                    self._df_comp.drop(index=droplist, inplace=True)

            if df_prov.empty:
                self._df_prov = pd.DataFrame()
            else:
                df_prov.drop(columns=ColNames.COMP, inplace=True)
                self._df_prov = df_prov


class CandlesStore(Node):
    """
    Candles store class.

    """

    def __init__(self) -> None:
        super().__init__("candles_store")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()

        # --------------- Initialize ROS parameter ---------------
        self._rosprm_use_inst_usdjpy = RosParam(
            "use_instrument.usdjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_eurjpy = RosParam(
            "use_instrument.eurjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_eurusd = RosParam(
            "use_instrument.eurusd", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_gbpjpy = RosParam(
            "use_instrument.gbpjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_audjpy = RosParam(
            "use_instrument.audjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_nzdjpy = RosParam(
            "use_instrument.nzdjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_cadjpy = RosParam(
            "use_instrument.cadjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_inst_chfjpy = RosParam(
            "use_instrument.chfjpy", Parameter.Type.BOOL
        )
        self._rosprm_use_gran_m1 = RosParam("use_granularity.m1", Parameter.Type.BOOL)
        self._rosprm_use_gran_m2 = RosParam("use_granularity.m2", Parameter.Type.BOOL)
        self._rosprm_use_gran_m3 = RosParam("use_granularity.m3", Parameter.Type.BOOL)
        self._rosprm_use_gran_m4 = RosParam("use_granularity.m4", Parameter.Type.BOOL)
        self._rosprm_use_gran_m5 = RosParam("use_granularity.m5", Parameter.Type.BOOL)
        self._rosprm_use_gran_m10 = RosParam("use_granularity.m10", Parameter.Type.BOOL)
        self._rosprm_use_gran_m15 = RosParam("use_granularity.m15", Parameter.Type.BOOL)
        self._rosprm_use_gran_m30 = RosParam("use_granularity.m30", Parameter.Type.BOOL)
        self._rosprm_use_gran_h1 = RosParam("use_granularity.h1", Parameter.Type.BOOL)
        self._rosprm_use_gran_h2 = RosParam("use_granularity.h2", Parameter.Type.BOOL)
        self._rosprm_use_gran_h3 = RosParam("use_granularity.h3", Parameter.Type.BOOL)
        self._rosprm_use_gran_h4 = RosParam("use_granularity.h4", Parameter.Type.BOOL)
        self._rosprm_use_gran_h6 = RosParam("use_granularity.h6", Parameter.Type.BOOL)
        self._rosprm_use_gran_h8 = RosParam("use_granularity.h8", Parameter.Type.BOOL)
        self._rosprm_use_gran_h12 = RosParam("use_granularity.h12", Parameter.Type.BOOL)
        self._rosprm_use_gran_d = RosParam("use_granularity.d", Parameter.Type.BOOL)
        self._rosprm_use_gran_w = RosParam("use_granularity.w", Parameter.Type.BOOL)
        self._rosprm_length_m1 = RosParam("data_length.m1", Parameter.Type.INTEGER)
        self._rosprm_length_m2 = RosParam("data_length.m2", Parameter.Type.INTEGER)
        self._rosprm_length_m3 = RosParam("data_length.m3", Parameter.Type.INTEGER)
        self._rosprm_length_m4 = RosParam("data_length.m4", Parameter.Type.INTEGER)
        self._rosprm_length_m5 = RosParam("data_length.m5", Parameter.Type.INTEGER)
        self._rosprm_length_m10 = RosParam("data_length.m10", Parameter.Type.INTEGER)
        self._rosprm_length_m15 = RosParam("data_length.m15", Parameter.Type.INTEGER)
        self._rosprm_length_m30 = RosParam("data_length.m30", Parameter.Type.INTEGER)
        self._rosprm_length_h1 = RosParam("data_length.h1", Parameter.Type.INTEGER)
        self._rosprm_length_h2 = RosParam("data_length.h2", Parameter.Type.INTEGER)
        self._rosprm_length_h3 = RosParam("data_length.h3", Parameter.Type.INTEGER)
        self._rosprm_length_h4 = RosParam("data_length.h4", Parameter.Type.INTEGER)
        self._rosprm_length_h6 = RosParam("data_length.h6", Parameter.Type.INTEGER)
        self._rosprm_length_h8 = RosParam("data_length.h8", Parameter.Type.INTEGER)
        self._rosprm_length_h12 = RosParam("data_length.h12", Parameter.Type.INTEGER)
        self._rosprm_length_d = RosParam("data_length.d", Parameter.Type.INTEGER)
        self._rosprm_length_w = RosParam("data_length.w", Parameter.Type.INTEGER)
        self._rosprm_next_updatetime_ofs_sec = RosParam(
            "next_updatetime_ofs_sec", Parameter.Type.INTEGER
        )
        self._rosprm_retry_interval_min = RosParam(
            "retry_interval_min", Parameter.Type.INTEGER
        )
        self._rosprm_fail_interval_min = RosParam(
            "fail_interval_min", Parameter.Type.INTEGER
        )
        self._rosprm_retry_count_max = RosParam(
            "retry_count_max", Parameter.Type.INTEGER
        )
        self._rosprm_self_retry_count_max = RosParam(
            "self_retry_count_max", Parameter.Type.INTEGER
        )

        rosutl.set_parameters(self, self._rosprm_use_inst_usdjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_eurjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_eurusd)
        rosutl.set_parameters(self, self._rosprm_use_inst_gbpjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_audjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_nzdjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_cadjpy)
        rosutl.set_parameters(self, self._rosprm_use_inst_chfjpy)
        rosutl.set_parameters(self, self._rosprm_use_gran_m1)
        rosutl.set_parameters(self, self._rosprm_use_gran_m2)
        rosutl.set_parameters(self, self._rosprm_use_gran_m3)
        rosutl.set_parameters(self, self._rosprm_use_gran_m4)
        rosutl.set_parameters(self, self._rosprm_use_gran_m5)
        rosutl.set_parameters(self, self._rosprm_use_gran_m10)
        rosutl.set_parameters(self, self._rosprm_use_gran_m15)
        rosutl.set_parameters(self, self._rosprm_use_gran_m30)
        rosutl.set_parameters(self, self._rosprm_use_gran_h1)
        rosutl.set_parameters(self, self._rosprm_use_gran_h2)
        rosutl.set_parameters(self, self._rosprm_use_gran_h3)
        rosutl.set_parameters(self, self._rosprm_use_gran_h4)
        rosutl.set_parameters(self, self._rosprm_use_gran_h6)
        rosutl.set_parameters(self, self._rosprm_use_gran_h8)
        rosutl.set_parameters(self, self._rosprm_use_gran_h12)
        rosutl.set_parameters(self, self._rosprm_use_gran_d)
        rosutl.set_parameters(self, self._rosprm_use_gran_w)
        rosutl.set_parameters(self, self._rosprm_length_m1)
        rosutl.set_parameters(self, self._rosprm_length_m2)
        rosutl.set_parameters(self, self._rosprm_length_m3)
        rosutl.set_parameters(self, self._rosprm_length_m4)
        rosutl.set_parameters(self, self._rosprm_length_m5)
        rosutl.set_parameters(self, self._rosprm_length_m10)
        rosutl.set_parameters(self, self._rosprm_length_m15)
        rosutl.set_parameters(self, self._rosprm_length_m30)
        rosutl.set_parameters(self, self._rosprm_length_h1)
        rosutl.set_parameters(self, self._rosprm_length_h2)
        rosutl.set_parameters(self, self._rosprm_length_h3)
        rosutl.set_parameters(self, self._rosprm_length_h4)
        rosutl.set_parameters(self, self._rosprm_length_h6)
        rosutl.set_parameters(self, self._rosprm_length_h8)
        rosutl.set_parameters(self, self._rosprm_length_h12)
        rosutl.set_parameters(self, self._rosprm_length_d)
        rosutl.set_parameters(self, self._rosprm_length_w)
        rosutl.set_parameters(self, self._rosprm_next_updatetime_ofs_sec)
        rosutl.set_parameters(self, self._rosprm_retry_interval_min)
        rosutl.set_parameters(self, self._rosprm_fail_interval_min)
        rosutl.set_parameters(self, self._rosprm_retry_count_max)
        rosutl.set_parameters(self, self._rosprm_self_retry_count_max)

        # --------------- Initialize ROS callback group ---------------
        self._cb_grp_reent = ReentrantCallbackGroup()
        self._cb_grp_mutua_cndque = MutuallyExclusiveCallbackGroup()
        self._cb_grp_mutua_timer = MutuallyExclusiveCallbackGroup()

        # --------------- Create ROS Communication ---------------
        try:
            # Create service client "CandlesQuery"
            srvcli = RosServiceClient(
                self,
                CandlesQuerySrv,
                "candles_query",
                callback_group=self._cb_grp_mutua_cndque,
            )
        except RosServiceErrorException as err:
            self.logger.error(err)
            raise InitializerErrorException("create service client failed.") from err

        self._candles_elem_list = []
        for gran_id, gran_leng in self._use_gran_list():
            for inst_id in self._use_inst_list():
                candles_elem = _CandlesElement(
                    self,
                    srvcli,
                    inst_id,
                    gran_id,
                    gran_leng,
                    self._rosprm_next_updatetime_ofs_sec.value,
                    self._rosprm_retry_interval_min.value,
                    self._rosprm_fail_interval_min.value,
                    self._rosprm_retry_count_max.value,
                    self._rosprm_self_retry_count_max.value,
                )
                self._candles_elem_list.append(candles_elem)

        # Create service server "CandlesByDatetime"
        self._cbd_srv = self.create_service(
            CandlesByDatetimeSrv,
            "candles_by_datetime",
            self._handle_candles_by_datetime,
            callback_group=self._cb_grp_reent,
        )

        # Create service server "CandlesByLength"
        self._cbl_srv = self.create_service(
            CandlesByLengthSrv,
            "candles_by_length",
            self._handle_candles_by_length,
            callback_group=self._cb_grp_reent,
        )

        # --------------- Create ROS Timer ---------------
        self._timer = self.create_timer(
            1.0, self._do_cyclic_event, callback_group=self._cb_grp_mutua_timer
        )

    def _do_cyclic_event(self) -> None:
        for candles_elem in self._candles_elem_list:
            candles_elem.do_cyclic_event()
            # self.logger.debug("inst_id:{}, gran_id:{}"
            #                   .format(candles_elem._inst_id, candles_data._gran_id))

    def _handle_candles_by_datetime(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
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

        if (gran_id == GranApi.GRAN_D) or (req.time_from == ""):
            start_time = None
        else:
            start_time = dt.datetime.strptime(req.time_from, FMT_TIME_HMS).time()

        if (gran_id == GranApi.GRAN_D) or (req.time_to == ""):
            end_time = None
        else:
            end_time = dt.datetime.strptime(req.time_to, FMT_TIME_HMS).time()

        rsp.cndl_msg_list = []
        rsp.next_update_time = ""
        df_comp = None
        for candles_elem in self._candles_elem_list:
            if (inst_id == candles_elem.inst_id) and (gran_id == candles_elem.gran_id):
                df_comp = candles_elem.df_comp
                rsp.next_update_time = candles_elem.next_updatetime.strftime(FMT_YMDHMS)
                break

        rsp.cndl_msg_list = []
        if df_comp is None:
            self.logger.error("{:!^50}".format(" ROS Service Error "))
            self.logger.error(
                "  Target(inst_id:[{}],gran_id:[{}]) is not exit in "
                "candles_elem.".format(inst_id, gran_id)
            )
        else:
            if not req.datetime_start == "":
                start_dt = dt.datetime.strptime(req.datetime_start, FMT_YMDHMS)
                if gran_id == GranApi.GRAN_D:
                    start_dt = dt.datetime.combine(start_dt.date(), dt.time(6, 0))
                df_comp = df_comp.loc[start_dt:]  # type: ignore[misc]

            if not req.datetime_end == "":
                end_dt = dt.datetime.strptime(req.datetime_end, FMT_YMDHMS)
                if gran_id == GranApi.GRAN_D:
                    end_dt = dt.datetime.combine(end_dt.date(), dt.time(7, 0))
                df_comp = df_comp.loc[:end_dt]  # type: ignore[misc]

            if req.dayofweeks:
                cond = [i in req.dayofweeks for i in df_comp.index.dayofweek]
                df_comp = df_comp[cond]

            if (start_time is not None) and (end_time is not None):
                df_comp = df_comp.between_time(start_time, end_time)
            elif (start_time is not None) and (end_time is None):
                df_comp = df_comp.between_time(start_time, MAX_TIME)
            elif (start_time is None) and (end_time is not None):
                df_comp = df_comp.between_time(MIN_TIME, end_time)
            else:
                pass

            if not df_comp.empty:
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

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug(
            "  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list))
        )
        self.logger.debug("  - next_update_time:[{}]".format(rsp.next_update_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_by_datetime]:End "))

        return rsp

    def _handle_candles_by_length(
        self, req: SrvTypeRequest, rsp: SrvTypeResponse
    ) -> SrvTypeResponse:
        self.logger.debug("{:=^50}".format(" Service[candles_by_length]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - gran_id:[{}]".format(req.gran_msg.gran_id))
        self.logger.debug("  - inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - length:[{}]".format(req.length))

        inst_id = INST_DICT[req.inst_msg.inst_id]
        gran_id = GRAN_DICT[req.gran_msg.gran_id]

        dbg_tm_start = dt.datetime.now()

        rsp.cndl_msg_list = []
        rsp.next_update_time = ""
        df_comp = None
        for candles_elem in self._candles_elem_list:
            if (inst_id == candles_elem.inst_id) and (gran_id == candles_elem.gran_id):
                df_comp = candles_elem.df_comp
                rsp.next_update_time = candles_elem.next_updatetime.strftime(FMT_YMDHMS)
                break

        if df_comp is None:
            self.logger.error("{:!^50}".format(" ROS Service Error "))
            self.logger.error(
                "  Target(inst_id:[{}],gran_id:[{}]) is not exit in "
                "candles_elem.".format(inst_id, gran_id)
            )
        elif req.length < 1:
            self.logger.error("{:!^50}".format(" ROS Service Error "))
            self.logger.error(
                "  Requested length:[{}] is incorrect.".format(req.length)
            )
        else:
            df_comp = df_comp.tail(req.length)

            if not df_comp.empty:
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

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug(
            "  - cndl_msg_list(length):[{}]".format(len(rsp.cndl_msg_list))
        )
        self.logger.debug("  - next_update_time:[{}]".format(rsp.next_update_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[candles_by_length]:End "))

        return rsp

    def _use_inst_list(self) -> list:
        inst_list: list[int] = []
        if self._rosprm_use_inst_usdjpy.value:
            inst_list.append(InstApi.INST_USD_JPY)
        if self._rosprm_use_inst_eurjpy.value:
            inst_list.append(InstApi.INST_EUR_JPY)
        if self._rosprm_use_inst_eurusd.value:
            inst_list.append(InstApi.INST_EUR_USD)
        if self._rosprm_use_inst_gbpjpy.value:
            inst_list.append(InstApi.INST_GBP_JPY)
        if self._rosprm_use_inst_audjpy.value:
            inst_list.append(InstApi.INST_AUD_JPY)
        if self._rosprm_use_inst_nzdjpy.value:
            inst_list.append(InstApi.INST_NZD_JPY)
        if self._rosprm_use_inst_cadjpy.value:
            inst_list.append(InstApi.INST_CAD_JPY)
        if self._rosprm_use_inst_chfjpy.value:
            inst_list.append(InstApi.INST_CHF_JPY)
        return inst_list

    def _use_gran_list(self) -> list:
        gran_list: list[tuple] = []
        if self._rosprm_use_gran_m1.value:
            gran_list.append((GranApi.GRAN_M1, self._rosprm_length_m1.value))
        if self._rosprm_use_gran_m2.value:
            gran_list.append((GranApi.GRAN_M2, self._rosprm_length_m2.value))
        if self._rosprm_use_gran_m3.value:
            gran_list.append((GranApi.GRAN_M3, self._rosprm_length_m3.value))
        if self._rosprm_use_gran_m4.value:
            gran_list.append((GranApi.GRAN_M4, self._rosprm_length_m4.value))
        if self._rosprm_use_gran_m5.value:
            gran_list.append((GranApi.GRAN_M5, self._rosprm_length_m5.value))
        if self._rosprm_use_gran_m10.value:
            gran_list.append((GranApi.GRAN_M10, self._rosprm_length_m10.value))
        if self._rosprm_use_gran_m15.value:
            gran_list.append((GranApi.GRAN_M15, self._rosprm_length_m15.value))
        if self._rosprm_use_gran_m30.value:
            gran_list.append((GranApi.GRAN_M30, self._rosprm_length_m30.value))
        if self._rosprm_use_gran_h1.value:
            gran_list.append((GranApi.GRAN_H1, self._rosprm_length_h1.value))
        if self._rosprm_use_gran_h2.value:
            gran_list.append((GranApi.GRAN_H2, self._rosprm_length_h2.value))
        if self._rosprm_use_gran_h3.value:
            gran_list.append((GranApi.GRAN_H3, self._rosprm_length_h3.value))
        if self._rosprm_use_gran_h4.value:
            gran_list.append((GranApi.GRAN_H4, self._rosprm_length_h4.value))
        if self._rosprm_use_gran_h6.value:
            gran_list.append((GranApi.GRAN_H6, self._rosprm_length_h6.value))
        if self._rosprm_use_gran_h8.value:
            gran_list.append((GranApi.GRAN_H8, self._rosprm_length_h8.value))
        if self._rosprm_use_gran_h12.value:
            gran_list.append((GranApi.GRAN_H12, self._rosprm_length_h12.value))
        if self._rosprm_use_gran_d.value:
            gran_list.append((GranApi.GRAN_D, self._rosprm_length_d.value))
        if self._rosprm_use_gran_w.value:
            gran_list.append((GranApi.GRAN_W, self._rosprm_length_w.value))
        return gran_list


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    candles_store = CandlesStore()

    try:
        rclpy.spin(candles_store, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
    finally:
        candles_store.destroy_node()
