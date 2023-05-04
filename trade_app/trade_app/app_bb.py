import time
from typing import TypeVar
from enum import Enum, auto
import datetime as dt
import pandas as pd
from transitions.extensions.factory import HierarchicalMachine
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from api_server_msgs.msg import Pricing
from trade_manager_msgs.srv import OrderRegisterSrv
from trade_manager_msgs.srv import TradeCloseRequestSrv
from trade_manager_msgs.msg import OrderType, OrderDir
from trade_manager_msgs.msg import LatestCandle
from .constant import FMT_YMDHMSF
from .constant import Transitions as Tr
from .exception import InitializerErrorException, RosServiceErrorException
from .dataclass import RosParam
from .parameter import InstParam, GranParam
from . import utils as utl
from . import ros_utils as rosutl
from .store import OhlcStore, ColOhlc
from .wrapper import RosServiceClient, FutureWrapper

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class TradeItem:
    """
    Trade item class.
    """

    class States(Enum):
        preparing = auto()
        standby = auto()
        long = auto()
        short = auto()

    def __init__(
        self,
        node: Node,
        srvcli_ord: RosServiceClient,
        srvcli_trdcls: RosServiceClient,
        inst_param: InstParam,
        units: int,
        sma_span: int,
        std_coef: float,
        take_profit_pips: int,
        sr_ohlc: pd.Series,
    ) -> None:
        # --------------- Set logger lebel ---------------
        self.logger = node.get_logger()

        # --------------- Define Constant value ---------------
        self._UNITS = units
        self._SMA_SPAN = sma_span
        self._STD_COEF = std_coef
        self._TAKE_PROFIT = inst_param.convert_pips2phy(take_profit_pips)
        self._COL_SMA = "sma"
        self._COL_PSTD = "p_std"
        self._COL_NSTD = "n_std"

        # --------------- Initialize instance variable ---------------
        self._srvcli_ord = srvcli_ord
        self._srvcli_trdcls = srvcli_trdcls
        self._inst_param = inst_param
        self._register_id = 0
        self._is_valid_trade = False
        self._has_done_order_register = True
        self._enable_trade_by_timeframe = True
        self._future: FutureWrapper | None = None

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME: self.States.preparing,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.standby,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.long,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.short,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
        ]

        transitions = [
            {
                Tr.TRIGGER: "_trans_from_preparing_to_standby",
                Tr.SOURCE: self.States.preparing,
                Tr.DEST: self.States.standby,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_preparing_to_long",
                Tr.SOURCE: self.States.preparing,
                Tr.DEST: self.States.long,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_preparing_to_short",
                Tr.SOURCE: self.States.preparing,
                Tr.DEST: self.States.short,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_standby_to_long",
                Tr.SOURCE: self.States.standby,
                Tr.DEST: self.States.long,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: "_on_after_action_from_standby_to_long",
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_long_to_standby",
                Tr.SOURCE: self.States.long,
                Tr.DEST: self.States.standby,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: "_on_after_action_from_long_to_standby",
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_standby_to_short",
                Tr.SOURCE: self.States.standby,
                Tr.DEST: self.States.short,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: "_on_after_action_from_standby_to_short",
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_short_to_standby",
                Tr.SOURCE: self.States.short,
                Tr.DEST: self.States.standby,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: "_on_after_action_from_short_to_standby",
                Tr.CONDITIONS: None,
            },
        ]

        self._sm = HierarchicalMachine(
            model=self,
            states=states,
            initial=self.States.preparing,
            transitions=transitions,
        )

        # --------------- Initialize DataFrame ---------------
        self._df_bb = self._generate_df(sr_ohlc)

    @property
    def df_bb(self) -> pd.DataFrame:
        return self._df_bb

    def refresh_dataframe(self, sr_ohlc: pd.Series) -> None:
        self._df_bb = self._generate_df(sr_ohlc)

    def do_price_subscribe_event(
        self,
        sr_ohlc: pd.Series,
        tick_price_ask: float,
        tick_price_bid: float,
        latest_rec: pd.Series,
    ) -> None:
        # ---------- Refresh DataFrame ----------
        if self._df_bb.index[-1] < sr_ohlc.index[-1]:
            self.refresh_dataframe(sr_ohlc)
            self._enable_trade_by_timeframe = True

        # ---------- Do state event ----------
        bb_rec = self._df_bb.iloc[-1]
        if self.state == self.States.preparing:
            self._on_subscribe_price_in_preparing(
                bb_rec, tick_price_ask, tick_price_bid
            )
        elif self.state == self.States.standby:
            self._on_subscribe_price_in_standby(
                bb_rec, tick_price_ask, tick_price_bid, latest_rec
            )
        elif self.state == self.States.long:
            self._on_subscribe_price_in_long(bb_rec, tick_price_bid)
        elif self.state == self.States.short:
            self._on_subscribe_price_in_short(bb_rec, tick_price_ask)
        else:
            pass

    def do_cyclic_event(self) -> None:

        if self.state == self.States.preparing:
            self._on_do_preparing()
        elif self.state == self.States.standby:
            self._on_do_standby()
        elif self.state == self.States.long:
            self._on_do_long()
        elif self.state == self.States.short:
            self._on_do_short()
        else:
            pass

    def _on_do_preparing(self) -> None:
        pass

    def _on_do_standby(self) -> None:
        pass

    def _on_subscribe_price_in_preparing(
        self, bb_rec: pd.Series, tick_price_ask: float, tick_price_bid: float
    ) -> None:

        if bb_rec[self._COL_PSTD] < tick_price_ask:
            self.logger.debug("----- trans_from_preparing_to_long -----")
            self._trans_from_preparing_to_long()
        elif bb_rec[self._COL_NSTD] > tick_price_bid:
            self.logger.debug("----- trans_from_preparing_to_short -----")
            self._trans_from_preparing_to_short()
        else:
            self.logger.debug("----- trans_from_preparing_to_standby -----")
            self._trans_from_preparing_to_standby()

    def _on_subscribe_price_in_standby(
        self,
        bb_rec: pd.Series,
        tick_price_ask: float,
        tick_price_bid: float,
        latest_rec: pd.Series,
    ) -> None:
        sma = bb_rec[self._COL_SMA]
        pstd = bb_rec[self._COL_PSTD]
        nstd = bb_rec[self._COL_NSTD]

        if pstd < tick_price_ask:
            bid_o = latest_rec[ColOhlc.BID_O]
            if (
                self._enable_trade_by_timeframe
                and (sma < tick_price_bid)
                and (sma < bid_o)
            ):
                self._is_valid_trade = True
                self._enable_trade_by_timeframe = False
                self.logger.debug("********** Trade Execute **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug("  - ExecuteTime:[{}]".format(dt.datetime.now()))
                self.logger.debug("  - P-STD:[{}]".format(pstd))
                self.logger.debug("  - SMA:[{}]".format(sma))
                self.logger.debug("  - tick_price_ask:[{}]".format(tick_price_ask))
                self.logger.debug("  - tick_price_bid:[{}]".format(tick_price_bid))
                self.logger.debug("  - latest_bid_o:[{}]".format(bid_o))
            else:
                self._is_valid_trade = False
                self.logger.debug("********** Trade Not Execute **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug(
                    "  - enable_trade_by_timeframe:[{}]".format(
                        self._enable_trade_by_timeframe
                    )
                )
                self.logger.debug(
                    "  - SMA:[{}] < tick_price_bid:[{}]".format(sma, tick_price_bid)
                )
                self.logger.debug("  - SMA:[{}] < latest_bid_o:[{}]".format(sma, bid_o))
            self._trans_from_standby_to_long()

        elif nstd > tick_price_bid:
            ask_o = latest_rec[ColOhlc.ASK_O]
            if (
                self._enable_trade_by_timeframe
                and (sma > tick_price_ask)
                and (sma > ask_o)
            ):
                self._is_valid_trade = True
                self._enable_trade_by_timeframe = False
                self.logger.debug("********** Trade Execute **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug("  - ExecuteTime:[{}]".format(dt.datetime.now()))
                self.logger.debug("  - N-STD:[{}]".format(nstd))
                self.logger.debug("  - SMA:[{}]".format(sma))
                self.logger.debug("  - tick_price_ask:[{}]".format(tick_price_ask))
                self.logger.debug("  - tick_price_bid:[{}]".format(tick_price_bid))
                self.logger.debug("  - latest_ask_o:[{}]".format(ask_o))
            else:
                self._is_valid_trade = False
                self.logger.debug("********** Trade Not Execute **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug(
                    "  - enable_trade_by_timeframe:[{}]".format(
                        self._enable_trade_by_timeframe
                    )
                )
                self.logger.debug(
                    "  - SMA:[{}] > tick_price_ask:[{}]".format(sma, tick_price_ask)
                )
                self.logger.debug("  - SMA:[{}] > latest_ask_o:[{}]".format(sma, ask_o))
            self._trans_from_standby_to_short()

        else:
            pass

    def _on_do_long(self) -> None:

        if (not self._is_valid_trade) or self._has_done_order_register:
            return

        if self._future is None:
            return
        elif self._future.done():
            if self._future.result() is not None:
                rsp = self._future.result()  # type: ignore[var-annotated]
                self._register_id = rsp.register_id
                self._has_done_order_register = True
                self.logger.debug(
                    "  Order complete.(register_id:[{}])".format(self._register_id)
                )
            else:
                self.logger.error("{:!^50}".format(" Order Request Error "))
                self.logger.error("  future.result() is None.")
                self._is_valid_trade = False
                self.logger.debug("  Order failed.")
        else:
            self.logger.debug("  Ordering now...")

    def _on_after_action_from_standby_to_long(self) -> None:
        self._has_done_order_register = False
        self._future = None

        if self._is_valid_trade:
            try:
                self._future = self._request_order(is_long_order=True)
            except RosServiceErrorException as err:
                self.logger.error("{:!^50}".format(" Call Async ROS Service Error "))
                self.logger.error([{}].format(err))
                self._is_valid_trade = False

    def _on_after_action_from_long_to_standby(self) -> None:

        if self._is_valid_trade:
            if self._has_done_order_register:
                try:
                    self._request_trade_close(self._register_id)
                except RosServiceErrorException as err:
                    self.logger.error(
                        "{:!^50}".format(" Call Async ROS Service Error ")
                    )
                    self.logger.error([{}].format(err))
            else:
                self.logger.warn("{:!^50}".format(" Unexpected statement "))

    def _on_subscribe_price_in_long(
        self, bb_rec: pd.Series, tick_price_bid: float
    ) -> None:

        if tick_price_bid < bb_rec[self._COL_SMA]:
            self._trans_from_long_to_standby()
            # ----- Long exit log -----
            if self._is_valid_trade:
                self._is_valid_trade = False
                self.logger.debug("********** Trade Close (Long) **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug("  - ExecuteTime:[{}]".format(dt.datetime.now()))
                self.logger.debug("  - register_id:[{}]".format(self._register_id))
                self.logger.debug("  - SMA:[{}]".format(bb_rec[self._COL_SMA]))
                self.logger.debug("  - tick_price_bid:[{}]".format(tick_price_bid))

    def _on_do_short(self) -> None:

        if (not self._is_valid_trade) or self._has_done_order_register:
            return

        if self._future is None:
            return
        elif self._future.done():
            if self._future.result() is not None:
                rsp = self._future.result()  # type: ignore[var-annotated]
                self._register_id = rsp.register_id
                self._has_done_order_register = True
                self.logger.debug(
                    "  Order complete.(register_id:[{}])".format(self._register_id)
                )
            else:
                self.logger.error("{:!^50}".format(" Order Request Error "))
                self.logger.error("  future.result() is None.")
                self._is_valid_trade = False
                self.logger.debug("  Order failed.")
        else:
            self.logger.debug("  Ordering now...")

    def _on_after_action_from_standby_to_short(self) -> None:
        self._has_done_order_register = False
        self._future = None

        if self._is_valid_trade:
            try:
                self._future = self._request_order(is_long_order=False)
            except RosServiceErrorException as err:
                self.logger.error("{:!^50}".format(" Call Async ROS Service Error "))
                self.logger.error([{}].format(err))
                self._is_valid_trade = False

    def _on_after_action_from_short_to_standby(self) -> None:

        if self._is_valid_trade:
            if self._has_done_order_register:
                try:
                    self._request_trade_close(self._register_id)
                except RosServiceErrorException:
                    self.logger.error(
                        "{:!^50}".format(" Call Async ROS Service Error ")
                    )
                    self.logger.error(
                        "  Trade close failed.(register_id:[{}])".format(
                            self._register_id
                        )
                    )
            else:
                self.logger.warn("{:!^50}".format(" Unexpected statement "))

    def _on_subscribe_price_in_short(
        self, bb_rec: pd.Series, tick_price_ask: float
    ) -> None:

        if tick_price_ask > bb_rec[self._COL_SMA]:
            self._trans_from_short_to_standby()
            if self._is_valid_trade:
                self._is_valid_trade = False
                self.logger.debug("********** Trade Close (Short) **********")
                self.logger.debug("  [Param]SMA:[{}]".format(self._SMA_SPAN))
                self.logger.debug("  [Param]STD:[{}]".format(self._STD_COEF))
                self.logger.debug("  - ExecuteTime:[{}]".format(dt.datetime.now()))
                self.logger.debug("  - register_id:[{}]".format(self._register_id))
                self.logger.debug("  - SMA:[{}]".format(bb_rec[self._COL_SMA]))
                self.logger.debug("  - tick_price_ask:[{}]".format(tick_price_ask))

    def _generate_df(self, sr_ohlc: pd.Series) -> pd.DataFrame:
        df = pd.DataFrame(sr_ohlc)
        base_roll = sr_ohlc.rolling(window=self._SMA_SPAN)
        sr_sma = base_roll.mean()
        df[self._COL_SMA] = sr_sma
        sr_std = base_roll.std(ddof=0) * self._STD_COEF
        df[self._COL_PSTD] = sr_sma + sr_std
        df[self._COL_NSTD] = sr_sma - sr_std
        return df

    def _request_order(self, is_long_order: bool) -> FutureWrapper:
        latest_rec = self._df_bb.iloc[-1]

        if is_long_order:
            entry_price_ide = utl.roundup(
                latest_rec[self._COL_PSTD], self._inst_param.digit
            )
            take_profit_price = entry_price_ide + self._TAKE_PROFIT
            stop_loss_price = utl.rounddown(
                latest_rec[self._COL_SMA], self._inst_param.digit
            )
            order_dir = OrderDir.LONG
        else:
            entry_price_ide = utl.rounddown(
                latest_rec[self._COL_NSTD], self._inst_param.digit
            )
            take_profit_price = entry_price_ide - self._TAKE_PROFIT
            stop_loss_price = utl.roundup(
                latest_rec[self._COL_SMA], self._inst_param.digit
            )
            order_dir = OrderDir.SHORT

        req = OrderRegisterSrv.Request()
        req.inst_msg.inst_id = self._inst_param.msg_id
        req.ordtyp_msg.order_type = OrderType.MARKET
        req.orddir_msg.order_dir = order_dir
        req.units = self._UNITS
        req.take_profit_price = take_profit_price
        req.stop_loss_price = stop_loss_price
        req.exit_exp_time = ""
        return self._srvcli_ord.call_async(req, timeout_sec=5.0)

    def _request_trade_close(self, register_id: int) -> FutureWrapper:
        req = TradeCloseRequestSrv.Request()
        req.register_id = register_id
        return self._srvcli_trdcls.call_async(req, timeout_sec=5.0)


class AppBollingerBand(Node):
    """
    App BollingerBand class.
    """

    class States(Enum):
        initializing = auto()
        normal = auto()
        abnormal = auto()

    def __init__(self) -> None:
        super().__init__("app_bb")

        # --------------- Set logger lebel ---------------
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = logger

        # --------------- Define constant value ---------------
        self._SUBS_TIMEOUT_SEC = 10.0

        # --------------- Initialize ROS parameter ---------------
        self._rosprm_inst = RosParam("instrument", Parameter.Type.STRING)
        self._rosprm_gran = RosParam("granularity", Parameter.Type.STRING)
        self._rosprm_units = RosParam("units", Parameter.Type.INTEGER)
        self._rosprm_sma_span_list = RosParam(
            "sma_span_list", Parameter.Type.INTEGER_ARRAY
        )
        self._rosprm_std_coef_list = RosParam(
            "std_coef_list", Parameter.Type.DOUBLE_ARRAY
        )
        self._rosprm_take_profit_pips_list = RosParam(
            "take_profit_pips_list", Parameter.Type.INTEGER_ARRAY
        )

        rosutl.set_parameters(self, self._rosprm_inst)
        rosutl.set_parameters(self, self._rosprm_gran)
        rosutl.set_parameters(self, self._rosprm_units)
        rosutl.set_parameters(self, self._rosprm_sma_span_list)
        rosutl.set_parameters(self, self._rosprm_std_coef_list)
        rosutl.set_parameters(self, self._rosprm_take_profit_pips_list)

        self._DF_LENGTH_MAX = max(self._rosprm_sma_span_list.value)

        # validate ROS Params list length
        rosutl.validate_ros_param_length(
            self,
            [
                self._rosprm_sma_span_list,
                self._rosprm_std_coef_list,
                self._rosprm_take_profit_pips_list,
            ],
        )

        self._inst_param = InstParam.get_member_by_namespace(self._rosprm_inst.value)
        self._gran_param = GranParam.get_member_by_namespace(self._rosprm_gran.value)

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME: self.States.initializing,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: "_on_exit_initializing",
            },
            {
                Tr.NAME: self.States.normal,
                Tr.ON_ENTER: "_on_enter_normal",
                Tr.ON_EXIT: None,
            },
            {
                Tr.NAME: self.States.abnormal,
                Tr.ON_ENTER: None,
                Tr.ON_EXIT: None,
            },
        ]

        transitions = [
            {
                Tr.TRIGGER: "_trans_from_initializing_to_normal",
                Tr.SOURCE: self.States.initializing,
                Tr.DEST: self.States.normal,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_normal_to_abnormal",
                Tr.SOURCE: self.States.normal,
                Tr.DEST: self.States.abnormal,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
            {
                Tr.TRIGGER: "_trans_from_abnormal_to_normal",
                Tr.SOURCE: self.States.abnormal,
                Tr.DEST: self.States.normal,
                Tr.PREPARE: None,
                Tr.BEFORE: None,
                Tr.AFTER: None,
                Tr.CONDITIONS: None,
            },
        ]

        self._sm = HierarchicalMachine(
            model=self,
            states=states,
            initial=self.States.initializing,
            transitions=transitions,
        )

        # --------------- Initialize variable ---------------
        self._timer_start = time.monotonic()
        self._tradeitem_list: list[TradeItem] = []
        self._latest_candle_msg_buff: list[LatestCandle] = []
        self._sub_pri: Subscription | None = None

        # --------------- Initialize ROS callback group ---------------
        self._cb_grp_mutua_timer = MutuallyExclusiveCallbackGroup()

        # --------------- Create ROS Communication ---------------
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create service client "OrderRequest"
        self._srvcli_ord = RosServiceClient(
            self, OrderRegisterSrv, "order_register", use_wait_for_service=False
        )

        # Create service client "TradeCloseRequest"
        self._srvcli_trdcls = RosServiceClient(
            self,
            TradeCloseRequestSrv,
            "trade_close_request",
            use_wait_for_service=False,
        )

        # Create topic subscriber "HeartBeat"
        self._sub_hb = self.create_subscription(
            String, "heart_beat", self._on_sub_heartbeat, qos_profile
        )

        # Create topic subscriber "LatestCandle"
        inst_name = self._inst_param.namespace
        gran_name = self._gran_param.namespace
        TPCNM_LATEST_CANDLE = inst_name + "_" + gran_name + "_latest_candle"
        self._sub_lc = self.create_subscription(
            LatestCandle, TPCNM_LATEST_CANDLE, self._on_sub_latest_candle, qos_profile
        )

        # --------------- Instantiated OhlcStore ---------------
        try:
            self._store = OhlcStore(
                self, self._inst_param, self._gran_param, self._DF_LENGTH_MAX
            )
        except InitializerErrorException as err:
            self.logger.error("{:!^50}".format(" Initialize Error "))
            self.logger.error("{}".format(err))
            raise InitializerErrorException("Failed to create instance.") from err

        # --------------- Create ROS Timer ---------------
        self._timer = self.create_timer(
            1.0, self._do_cyclic_event, callback_group=self._cb_grp_mutua_timer
        )

        self.logger.debug("{:=^50}".format(" Initialize Finish! "))

    def _do_cyclic_event(self) -> None:

        if self.is_normal():
            # "Do" process in Normal State.
            self._on_do_normal()
        elif self.state == self.States.abnormal:
            # "Do" process in Abnormal State.
            pass
        elif self.state == self.States.initializing:
            self._on_do_initializing()
        else:
            pass

    def _on_do_initializing(self) -> None:

        if self._store.is_filled_df():
            self._trans_from_initializing_to_normal()

    def _on_exit_initializing(self) -> None:
        self._tradeitem_list = []

        for idx in range(len(self._rosprm_sma_span_list.value)):
            sma_span = self._rosprm_sma_span_list.value[idx]
            std_coef = self._rosprm_std_coef_list.value[idx]
            tp_th_pips = self._rosprm_take_profit_pips_list.value[idx]
            self._tradeitem_list.append(
                TradeItem(
                    self,
                    self._srvcli_ord,
                    self._srvcli_trdcls,
                    self._inst_param,
                    self._rosprm_units.value,
                    sma_span,
                    std_coef,
                    tp_th_pips,
                    self._store.df_ohlc[ColOhlc.MID_O],
                )
            )

        # --------------- Create topic subscriber "Pricing" ---------------
        topic = "pricing_" + self._inst_param.namespace
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )
        self._sub_pri = self.create_subscription(
            Pricing, topic, self._on_sub_pricing, qos_profile
        )

    def _on_enter_normal(self) -> None:
        self._clear_timer()

    def _on_do_normal(self) -> None:

        if self._watch_timeout():
            self.logger.debug("----- Change State Normal to Abormal")
            self._trans_from_normal_to_abnormal()
        else:
            # ---------- add latest OHLC complete data ----------
            if self._latest_candle_msg_buff:
                self._store.add_latest_comp_ohlc(self._latest_candle_msg_buff)
                self._latest_candle_msg_buff = []

            # ---------- OHLC Data Store cyclic event ----------
            self._store.do_cyclic_event()
            if self._store.is_completed_update():
                for trdsts in self._tradeitem_list:
                    trdsts.refresh_dataframe(self._store.df_ohlc[ColOhlc.MID_O])

            # ---------- Trade cyclic event ----------
            for trdsts in self._tradeitem_list:
                trdsts.do_cyclic_event()

    def _watch_timeout(self) -> bool:
        return self._SUBS_TIMEOUT_SEC < (time.monotonic() - self._timer_start)

    def _clear_timer(self) -> None:
        self._timer_start = time.monotonic()

    def _on_sub_heartbeat(self, msg: String) -> None:  # pylint: disable=W0613

        if self.state == self.States.abnormal:
            self._trans_from_abnormal_to_normal()
        self._clear_timer()

    def _on_sub_pricing(self, msg: Pricing) -> None:

        if self.state == self.States.abnormal:
            self._trans_from_abnormal_to_normal()

        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            # ---------- Update incompete OHLC data ----------
            msg_time = dt.datetime.strptime(msg.time, FMT_YMDHMSF)
            tick_price_ask = msg.asks[0].price
            tick_price_bid = msg.bids[0].price
            self._store.update_incomp_ohlc(msg_time, tick_price_ask, tick_price_bid)

            # ---------- Judge trade "Entry" ----------
            for trdsts in self._tradeitem_list:
                trdsts.do_price_subscribe_event(
                    self._store.df_ohlc[ColOhlc.MID_O],
                    tick_price_ask,
                    tick_price_bid,
                    self._store.latest_record,
                )

        self._clear_timer()

    def _on_sub_latest_candle(self, msg: LatestCandle) -> None:
        self._latest_candle_msg_buff.append(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    app_bb = AppBollingerBand()

    try:
        rclpy.spin(app_bb, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
    finally:
        app_bb.destroy_node()
