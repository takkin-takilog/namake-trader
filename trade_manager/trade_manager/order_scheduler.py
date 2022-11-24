import sys
import os
import gc
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
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from api_msgs.msg import Pricing
from trade_manager_msgs.msg import OrderType, OrderDir
from trade_manager_msgs.srv import OrderRegisterSrv
from trade_manager_msgs.srv import TradeCloseRequestSrv
from trade_manager_msgs.msg import Instrument as InstMng
from api_msgs.srv import AccountQuerySrv
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderState, TradeState
from api_msgs.msg import OrderType as ApiOrderType
from api_msgs.msg import FailReasonCode as frc
from .constant import FMT_YMDHMS, FMT_YMDHMSF
from .constant import BUCKUP_DIR
from .constant import WeekDay
from .constant import Transitions as Tr
from .constant import INST_DICT
from .exception import InitializerErrorException
from .dataclass import RosParam
from .wrapper import RosServiceClient
from .trigger import TimeTrigger
from . import backup as bk
from . import ros_utils as rosutl
from trade_manager.dataclass import RosParamTime

MsgType = TypeVar("MsgType")
SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class ColOrderSchedulerBackup(Enum):
    """
    Order scheduler backup dataframe column name.
    """
    REGISTER_ID = "register_id"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColOrderTicketsBackup(Enum):
    """
    Order tickets backup dataframe column name.
    """
    REGISTER_ID = "register_id"
    ORDER_ID = "order_id"
    TRADE_ID = "trade_id"
    INST_ID = "inst_id"
    ORDER_TYPE = "order_type"
    UNITS = "units"
    ENTRY_PRICE = "entry_price"
    ENTRY_EXP_TIME = "entry_exp_time"
    TAKE_PROFIT_PRICE = "take_profit_price"
    STOP_LOSS_PRICE = "stop_loss_price"
    EXIT_EXP_TIME = "exit_exp_time"
    API_INST_ID = "api_inst_id"
    API_ORDER_TYPE = "api_order_type"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


@dataclass
class _TickPrice():
    """
    Tick price.
    """
    time: str
    tick_ask: float
    tick_bid: float


class OrderTicket():

    class States(Enum):
        EntryOrdering = auto()
        EntryWaiting = auto()
        EntryChecking = auto()
        EntryCanceling = auto()
        ExitWaiting = auto()
        ExitChecking = auto()
        ExitOrdering = auto()
        Complete = auto()

    # --------------- Define class variable ---------------
    _C_CI_CONST = 0

    # --------------- Define class variable ---------------
    _c_srvcli_ordcre = None
    _c_srvcli_orddet = None
    _c_srvcli_ordcnc = None
    _c_srvcli_trddet = None
    _c_srvcli_trdcrc = None
    _c_srvcli_trdcls = None
    _c_is_trans_lock = False
    _c_register_id = 1
    _c_balance = 0
    logger = None

    @classmethod
    def set_class_variable(cls,
                           srvcli_ordcre: RosServiceClient,
                           srvcli_orddet: RosServiceClient,
                           srvcli_ordcnc: RosServiceClient,
                           srvcli_trddet: RosServiceClient,
                           srvcli_trdcrc: RosServiceClient,
                           srvcli_trdcls: RosServiceClient,
                           max_leverage: float,
                           max_position_count: int,
                           logger
                           ) -> None:
        cls._c_srvcli_ordcre = srvcli_ordcre
        cls._c_srvcli_orddet = srvcli_orddet
        cls._c_srvcli_ordcnc = srvcli_ordcnc
        cls._c_srvcli_trddet = srvcli_trddet
        cls._c_srvcli_trdcrc = srvcli_trdcrc
        cls._c_srvcli_trdcls = srvcli_trdcls
        cls._C_CI_CONST = max_leverage / max_position_count
        cls.logger = logger

    @classmethod
    def set_balance(cls, balance: int):
        cls._c_balance = balance

    @classmethod
    def get_register_id(cls):
        return cls._c_register_id

    @classmethod
    def set_register_id(cls, register_id: int):
        cls._c_register_id = register_id

    def __init__(self,
                 req: SrvTypeRequest,
                 tick_price: _TickPrice,
                 use_restore: bool = False
                 ) -> None:

        # --------------- Define Constant value ---------------
        self._POL_INTERVAL = dt.timedelta(minutes=1)

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME.value: self.States.EntryOrdering,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryWaiting,
                Tr.ON_ENTER.value: "_on_enter_EntryWaiting",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryChecking,
                Tr.ON_ENTER.value: "_on_enter_EntryChecking",
                Tr.ON_EXIT.value: "_on_exit_EntryChecking"
            },
            {
                Tr.NAME.value: self.States.EntryCanceling,
                Tr.ON_ENTER.value: "_on_enter_EntryCanceling",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.ExitWaiting,
                Tr.ON_ENTER.value: "_on_enter_ExitWaiting",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.ExitChecking,
                Tr.ON_ENTER.value: "_on_enter_ExitChecking",
                Tr.ON_EXIT.value: "_on_exit_ExitChecking"
            },
            {
                Tr.NAME.value: self.States.ExitOrdering,
                Tr.ON_ENTER.value: "_on_enter_ExitOrdering",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.Complete,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
        ]

        transitions = [
            {
                Tr.TRIGGER.value: "_trans_from_EntryOrdering_to_ExitWaiting",
                Tr.SOURCE.value: self.States.EntryOrdering,
                Tr.DEST.value: self.States.ExitWaiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryOrdering_to_EntryWaiting",
                Tr.SOURCE.value: self.States.EntryOrdering,
                Tr.DEST.value: self.States.EntryWaiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryWaiting_to_EntryChecking",
                Tr.SOURCE.value: self.States.EntryWaiting,
                Tr.DEST.value: self.States.EntryChecking,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: "_conditions_trans_lock"
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryChecking_to_EntryWaiting",
                Tr.SOURCE.value: self.States.EntryChecking,
                Tr.DEST.value: self.States.EntryWaiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryWaiting_to_EntryCanceling",
                Tr.SOURCE.value: self.States.EntryWaiting,
                Tr.DEST.value: self.States.EntryCanceling,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryCanceling_to_Complete",
                Tr.SOURCE.value: self.States.EntryCanceling,
                Tr.DEST.value: self.States.Complete,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryCanceling_to_EntryChecking",
                Tr.SOURCE.value: self.States.EntryCanceling,
                Tr.DEST.value: self.States.EntryChecking,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: "_conditions_trans_lock"
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryChecking_to_ExitWaiting",
                Tr.SOURCE.value: self.States.EntryChecking,
                Tr.DEST.value: self.States.ExitWaiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_ExitWaiting_to_ExitChecking",
                Tr.SOURCE.value: self.States.ExitWaiting,
                Tr.DEST.value: self.States.ExitChecking,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: "_conditions_trans_lock"
            },
            {
                Tr.TRIGGER.value: "_trans_from_ExitChecking_to_ExitWaiting",
                Tr.SOURCE.value: self.States.ExitChecking,
                Tr.DEST.value: self.States.ExitWaiting,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_ExitChecking_to_Complete",
                Tr.SOURCE.value: self.States.ExitChecking,
                Tr.DEST.value: self.States.Complete,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_ExitWaiting_to_ExitOrdering",
                Tr.SOURCE.value: self.States.ExitWaiting,
                Tr.DEST.value: self.States.ExitOrdering,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_ExitOrdering_to_Complete",
                Tr.SOURCE.value: self.States.ExitOrdering,
                Tr.DEST.value: self.States.Complete,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_to_Complete",
                Tr.SOURCE.value: "*",
                Tr.DEST.value: self.States.Complete,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            }
        ]

        self._sm = Machine(model=self,
                           states=states,
                           initial=self.States.EntryOrdering,
                           transitions=transitions)

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        # --------------- Initialize instance variable ---------------
        self._future = None
        self._trade_id = None
        self._order_id = None
        self._is_entry_exp_time_over = False
        self._enable_trade_close = False
        self._enable_weekend_close = False

        if use_restore:
            return

        self._inst_id = req.inst_msg.inst_id
        self._order_type = req.ordtyp_msg.order_type

        if req.units == 0:
            if self._order_type == OrderType.MARKET:
                if req.orddir_msg.order_dir == OrderDir.LONG:
                    price = tick_price.tick_ask
                else:
                    price = tick_price.tick_bid
            else:
                price = req.entry_price

            units = int(self._C_CI_CONST * self._c_balance / price)

            if req.orddir_msg.order_dir == OrderDir.LONG:
                self._units = units
            else:
                self._units = -units
        else:
            if req.orddir_msg.order_dir == OrderDir.LONG:
                self._units = req.units
            else:
                self._units = -req.units

        if self._order_type == OrderType.MARKET:
            self._entry_price = 0.0
        else:
            self._entry_price = req.entry_price

        if (self._order_type == OrderType.MARKET) or (not req.entry_exp_time):
            self._entry_exp_time = None
        else:
            self._entry_exp_time = dt.datetime.strptime(req.entry_exp_time,
                                                        FMT_YMDHMS)

        self._take_profit_price = req.take_profit_price
        self._stop_loss_price = req.stop_loss_price

        if not req.exit_exp_time:
            self._exit_exp_time = None
        else:
            self._exit_exp_time = dt.datetime.strptime(req.exit_exp_time,
                                                       FMT_YMDHMS)

        self._api_inst_id = INST_DICT[req.inst_msg.inst_id]

        if self._order_type == OrderType.MARKET:
            self._api_order_type = ApiOrderType.TYP_MARKET
        elif self._order_type == OrderType.LIMIT:
            self._api_order_type = ApiOrderType.TYP_LIMIT
        elif self._order_type == OrderType.STOP:
            self._api_order_type = ApiOrderType.TYP_STOP
        else:
            self.logger.error("{:!^50}".format(" Unexpected order type "))
            self.logger.error("order_type:[{}]".format(self._order_type))
            raise InitializerErrorException("Unexpected order type.")

        self.logger.debug("---------- Create OrderTicket ----------")
        self.logger.debug("  - Create Time:{}".format(dt.datetime.now()))
        self.logger.debug("  - inst_id:[{}]".format(self._inst_id))
        self.logger.debug("  - order_type:[{}]".format(self._order_type))
        self.logger.debug("  - units:[{}]".format(self._units))
        self.logger.debug("  - entry_price:[{}]".format(self._entry_price))
        self.logger.debug("  - entry_exp_time:[{}]".format(self._entry_exp_time))
        self.logger.debug("  - take_profit_price:[{}]".format(self._take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(self._stop_loss_price))
        self.logger.debug("  - exit_exp_time:[{}]".format(self._exit_exp_time))
        self.logger.debug("  - api_inst_id:[{}]".format(self._api_inst_id))
        self.logger.debug("  - api_order_type:[{}]".format(self._api_order_type))

        # --------------- Update requested id ---------------
        self._register_id = OrderTicket._c_register_id
        OrderTicket._c_register_id += 1

        # --------------- Entry order ---------------
        req = OrderCreateSrv.Request()
        req.ordertype_msg.type = self._api_order_type
        req.price = self._entry_price
        req.units = self._units
        req.inst_msg.inst_id = self._api_inst_id
        req.take_profit_price = self._take_profit_price
        req.stop_loss_price = self._stop_loss_price
        self.logger.debug("----- Requesting \"Order Create\" -----")
        try:
            self._future = self._c_srvcli_ordcre.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{}".format(err))
            raise InitializerErrorException("Call ROS Service Error (Order Create)")

    def __del__(self) -> None:
        self.logger.debug("---------- Delete OrderTicket ----------")
        self.logger.debug("  - order_id:[{}]".format(self._order_id))
        self.logger.debug("  - trade_id:[{}]".format(self._trade_id))

    @property
    def register_id(self) -> int:
        return self._register_id

    def generate_buckup_record(self) -> list:
        bk_rec_list = [
            self._register_id,
            self._order_id,
            self._trade_id,
            self._inst_id,
            self._order_type,
            self._units,
            self._entry_price,
            self._entry_exp_time,
            self._take_profit_price,
            self._stop_loss_price,
            self._exit_exp_time,
            self._api_inst_id,
            self._api_order_type,
        ]
        return bk_rec_list

    def restore(self, rec: pd.Series):
        Col = ColOrderTicketsBackup

        entry_exp_time = rec[Col.ENTRY_EXP_TIME.value]
        if entry_exp_time is not None:
            entry_exp_time = dt.datetime.strptime(entry_exp_time,
                                                  FMT_YMDHMS)
        exit_exp_time = rec[Col.EXIT_EXP_TIME.value]
        if exit_exp_time is not None:
            exit_exp_time = dt.datetime.strptime(exit_exp_time,
                                                 FMT_YMDHMS)

        order_id = rec[Col.ORDER_ID.value]
        if order_id is not None:
            order_id = int(order_id)

        trade_id = rec[Col.TRADE_ID.value]
        if trade_id is not None:
            trade_id = int(trade_id)

        self._register_id = int(rec[Col.REGISTER_ID.value])
        self._order_id = order_id
        self._trade_id = trade_id
        self._inst_id = int(rec[Col.INST_ID.value])
        self._order_type = int(rec[Col.ORDER_TYPE.value])
        self._units = int(rec[Col.UNITS.value])
        self._entry_price = float(rec[Col.ENTRY_PRICE.value])
        self._entry_exp_time = entry_exp_time
        self._take_profit_price = float(rec[Col.TAKE_PROFIT_PRICE.value])
        self._stop_loss_price = float(rec[Col.STOP_LOSS_PRICE.value])
        self._exit_exp_time = exit_exp_time
        self._api_inst_id = int(rec[Col.API_INST_ID.value])
        self._api_order_type = int(rec[Col.API_ORDER_TYPE.value])

        self.logger.info("<<<<<<<<<< Restore:register_id[{}] >>>>>>>>>>"
                         .format(self._register_id))
        self.logger.debug("  - inst_id:[{}]".format(self._inst_id))
        self.logger.debug("  - order_type:[{}]".format(self._order_type))
        self.logger.debug("  - units:[{}]".format(self._units))
        self.logger.debug("  - entry_price:[{}]".format(self._entry_price))
        self.logger.debug("  - entry_exp_time:[{}]".format(self._entry_exp_time))
        self.logger.debug("  - take_profit_price:[{}]".format(self._take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(self._stop_loss_price))
        self.logger.debug("  - exit_exp_time:[{}]".format(self._exit_exp_time))
        self.logger.debug("  - api_inst_id:[{}]".format(self._api_inst_id))
        self.logger.debug("  - api_order_type:[{}]".format(self._api_order_type))

        if self._trade_id is None:
            self.to_EntryChecking()
        else:
            self.to_ExitChecking()

    def enable_weekend_close(self) -> None:
        self._enable_weekend_close = True
        self._enable_trade_close = True
        self.logger.debug("----- << Enable weekend close >> -----")

    def enable_trade_close(self) -> bool:
        success = False
        if (self.state == self.States.ExitWaiting
            or self.state == self.States.ExitChecking
            or self.state == self.States.ExitOrdering
                or self.state == self.States.Complete):
            self._enable_trade_close = True
            success = True
        return success

    def do_cyclic_event(self) -> None:

        # self.logger.debug("state:[{}]".format(self.state))

        if self.state == self.States.EntryOrdering:
            self._on_do_EntryOrdering()
        elif self.state == self.States.EntryWaiting:
            self._on_do_EntryWaiting()
        elif self.state == self.States.EntryChecking:
            self._on_do_EntryChecking()
        elif self.state == self.States.EntryCanceling:
            self._on_do_EntryCanceling()
        elif self.state == self.States.ExitWaiting:
            self._on_do_ExitWaiting()
        elif self.state == self.States.ExitChecking:
            self._on_do_ExitChecking()
        elif self.state == self.States.ExitOrdering:
            self._on_do_ExitOrdering()
        elif self.state == self.States.Complete:
            self._on_do_Complete()
        else:
            pass

    def _on_do_EntryOrdering(self) -> None:

        if not self._future.done():
            self.logger.debug("  Requesting now...")
            if self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
                self.logger.error("  ROS Service Response Timeout.")
                self._trans_to_Complete()
            return

        self.logger.debug("  Request done.")
        rsp = self._future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
            self.logger.error("  future.result() is \"None\".")
            self._trans_to_Complete()
            return

        if not rsp.result:
            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Create) "))
            self._trans_to_Complete()
            return

        if self._order_type == OrderType.MARKET:
            self._trade_id = rsp.id
            self.logger.debug("  - trade_id:[{}]".format(self._trade_id))
            self._trans_from_EntryOrdering_to_ExitWaiting()
        else:
            self._order_id = rsp.id
            self.logger.debug("  - order_id:[{}]".format(self._order_id))
            self._trans_from_EntryOrdering_to_EntryWaiting()

    def _on_enter_EntryWaiting(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._next_pol_time = self._update_next_pollingtime(dt.datetime.now())

    def _on_do_EntryWaiting(self) -> None:
        now = dt.datetime.now()
        if self._is_entry_exp_time_over:
            self._trans_to_Complete()
        elif (self._entry_exp_time is not None) and (self._entry_exp_time < now):
            self._trans_from_EntryWaiting_to_EntryCanceling()
            self._is_entry_exp_time_over = True
        elif self._enable_weekend_close:
            self._trans_from_EntryWaiting_to_EntryCanceling()
        elif self._next_pol_time < now:
            self.logger.debug("<<< Timeout >>> in EntryWaiting")
            self._trans_from_EntryWaiting_to_EntryChecking()
        else:
            pass

    def _conditions_trans_lock(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self.logger.debug("--- trans_lock state:[{}]".format(self._c_is_trans_lock))
        return not self._c_is_trans_lock

    def _on_enter_EntryChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._c_is_trans_lock = True
        self.logger.debug("--- Trans \"Locked\"")
        self.logger.debug("----- Request \"Order Details\" (id:[{}]) -----"
                          .format(self._order_id))
        req = OrderDetailsSrv.Request()
        req.order_id = self._order_id
        self._future = None
        try:
            self._future = self._c_srvcli_orddet.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
            self.logger.error("{}".format(err))

    def _on_do_EntryChecking(self) -> None:

        if self._future is None:
            self._trans_from_EntryChecking_to_EntryWaiting()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))
            if self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
                self.logger.error("  ROS Service Response Timeout.")
                self._trans_from_EntryChecking_to_EntryWaiting()
            return

        self.logger.debug("  Request done.(id:[{}])".format(self._order_id))
        rsp = self._future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
            self.logger.error("  future.result() is \"None\".")
            self._trans_from_EntryChecking_to_EntryWaiting()
            return

        if not rsp.result:
            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Details) "))
            self._trans_from_EntryChecking_to_EntryWaiting()
            return

        if rsp.order_state_msg.state == OrderState.STS_PENDING:
            self.logger.debug("  - order id:[{}] is Pending.".format(self._order_id))
            self._trans_from_EntryChecking_to_EntryWaiting()
        elif rsp.order_state_msg.state == OrderState.STS_FILLED:
            self._trade_id = rsp.open_trade_id
            self.logger.debug("  - order_id:[{}] is Filled.".format(self._order_id))
            self.logger.debug("  - trade_id:[{}] is Opened.".format(self._trade_id))
            self._trans_from_EntryChecking_to_ExitWaiting()
        else:
            self.logger.warn("  - order id:[{}] is Unexpected State! (State No:<{}>)"
                             .format(self._order_id, rsp.order_state_msg.state))
            self._trans_from_EntryChecking_to_EntryWaiting()

    def _on_exit_EntryChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._c_is_trans_lock = False
        self.logger.debug("--- Trans \"Unlocked\"")

    def _on_enter_EntryCanceling(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self.logger.debug("----- Request \"Order Cancel\" (id:[{}]) -----"
                          .format(self._order_id))
        req = OrderCancelSrv.Request()
        req.order_id = self._order_id
        self._future = None
        try:
            self._future = self._c_srvcli_ordcnc.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
            self.logger.error("{}".format(err))

    def _on_do_EntryCanceling(self) -> None:

        if self._future is None:
            self._trans_to_Complete()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))
            if self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
                self.logger.error("  ROS Service Response Timeout.")
                self._trans_to_Complete()
            return

        self.logger.debug("  Request done.(id:[{}])".format(self._order_id))
        rsp = self._future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
            self.logger.error("  future.result() is \"None\".")
            self._trans_to_Complete()
            return

        if rsp.result:
            self.logger.debug("  EntryCanceling complete.(id:[{}])".format(self._order_id))
            self._trans_from_EntryCanceling_to_Complete()
        else:
            if rsp.frc_msg.reason_code == frc.REASON_ORDER_DOESNT_EXIST:
                self.logger.warn("  EntryCanceling fail.(id:[{}])".format(self._order_id))
                self.logger.warn("   Order doesnt exist!")
                self.logger.warn("   Possibility that order have been contracted.")
                self._trans_from_EntryCanceling_to_EntryChecking()
            else:
                self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Cancel) "))
                self._trans_to_Complete()

    def _on_enter_ExitWaiting(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._next_pol_time = self._update_next_pollingtime(dt.datetime.now())

    def _on_do_ExitWaiting(self) -> None:
        now = dt.datetime.now()
        if (self._exit_exp_time is not None) and (self._exit_exp_time < now):
            self._trans_from_ExitWaiting_to_ExitOrdering()
        elif self._enable_trade_close:
            self._trans_from_ExitWaiting_to_ExitOrdering()
        elif self._next_pol_time < now:
            self.logger.debug("<<< Timeout >>> in ExitWaiting")
            self._trans_from_ExitWaiting_to_ExitChecking()
        else:
            pass

    def _on_enter_ExitChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._c_is_trans_lock = True
        self.logger.debug("--- Trans \"Locked\"")
        self.logger.debug("----- Request \"Trade Details\" (id:[{}]) -----"
                          .format(self._trade_id))
        req = TradeDetailsSrv.Request()
        req.trade_id = self._trade_id
        self._future = None
        try:
            self._future = self._c_srvcli_trddet.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
            self.logger.error("{}".format(err))

    def _on_do_ExitChecking(self) -> None:

        if self._future is None:
            self._trans_from_ExitChecking_to_ExitWaiting()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))
            if self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
                self.logger.error("  ROS Service Response Timeout.")
                self._trans_from_ExitChecking_to_ExitWaiting()
            return

        self.logger.debug("  Request done.(id:[{}])".format(self._trade_id))
        rsp = self._future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
            self.logger.error("  future.result() is \"None\".")
            self._trans_from_ExitChecking_to_ExitWaiting()
            return

        if not rsp.result:
            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Trade Details) "))
            self._trans_from_ExitChecking_to_ExitWaiting()
            return

        if rsp.trade_state_msg.state == TradeState.STS_OPEN:
            self.logger.debug("  - trade id:[{}] is Opening.".format(self._trade_id))
            self._trans_from_ExitChecking_to_ExitWaiting()
        elif rsp.trade_state_msg.state == TradeState.STS_CLOSED:
            self.logger.debug("  - trade id:[{}] is Closed.".format(self._trade_id))
            self._trans_from_ExitChecking_to_Complete()
        else:
            self.logger.warn("  - trade id:[{}] is Unexpected State! (State No:<{}>)"
                             .format(self._trade_id, rsp.trade_state_msg.state))
            self._trans_from_ExitChecking_to_ExitWaiting()

    def _on_exit_ExitChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._c_is_trans_lock = False
        self.logger.debug("--- Trans \"Unlocked\"")

    def _on_enter_ExitOrdering(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self.logger.debug("----- Request \"Trade Close\" (id:[{}]) -----"
                          .format(self._trade_id))

        req = TradeCloseSrv.Request()
        req.trade_id = self._trade_id
        self._future = None
        try:
            self._future = self._c_srvcli_trdcls.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
            self.logger.error("{}".format(err))

    def _on_do_ExitOrdering(self) -> None:

        if self._future is None:
            self._trans_to_Complete()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))
            if self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
                self.logger.error("  ROS Service Response Timeout.")
                self._trans_to_Complete()
            return

        self.logger.debug("  Request done.(id:[{}])".format(self._trade_id))
        rsp = self._future.result()
        if rsp is None:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
            self.logger.error("  future.result() is \"None\".")
            self._trans_to_Complete()
            return

        if rsp.result:
            self._trans_from_ExitOrdering_to_Complete()
        else:
            if rsp.frc_msg.reason_code == frc.REASON_TRADE_DOESNT_EXIST:
                self.logger.warn("  ExitOrdering fail.(id:[{}])".format(self._trade_id))
                self.logger.warn("   Trade doesnt exist!")
                self.logger.warn("   Possibility that trade have been contracted.")
                self._trans_from_ExitOrdering_to_Complete()
            else:
                self.logger.error("{:!^50}".format(" Call ROS Service Fail (Trade Close) "))
                self._trans_to_Complete()

    def _on_do_Complete(self) -> None:
        pass

    def _update_next_pollingtime(self, time: dt.datetime) -> dt.datetime:
        next_time = time.replace(second=10, microsecond=0) + self._POL_INTERVAL
        self.logger.debug(" - update polling time:{}".format(next_time))
        return next_time


class OrderScheduler(Node):

    class States(Enum):
        Idle = auto()
        AccountUpdating = auto()

    def __init__(self) -> None:
        super().__init__("order_scheduler")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # --------------- Define constant value ---------------
        self._ACCOUNT_UPDATETIME_SEC = 30

        buckup_dir = os.path.expanduser("~") + BUCKUP_DIR
        filename_os = "bak_order_scheduler.csv"
        filename_ot = "bak_order_tickets.csv"
        self._BUCKUP_FULLPATH_OS = buckup_dir + filename_os
        self._BUCKUP_FULLPATH_OT = buckup_dir + filename_ot

        # --------------- Declare ROS parameter ---------------
        self._rosprm_max_leverage = RosParam("max_leverage",
                                             Parameter.Type.DOUBLE)
        self._rosprm_max_position_count = RosParam("max_position_count",
                                                   Parameter.Type.INTEGER)
        self._rosprm_use_weekend_order_stop = RosParam("use_weekend_order_stop",
                                                       Parameter.Type.BOOL)
        self._rosprm_weekend_order_stop_time = RosParamTime("weekend_order_stop_time",
                                                            Parameter.Type.STRING)
        self._rosprm_use_weekend_all_close = RosParam("use_weekend_all_close",
                                                      Parameter.Type.BOOL)
        self._rosprm_weekend_all_close_time = RosParamTime("weekend_all_close_time",
                                                           Parameter.Type.STRING)

        rosutl.set_parameters(self, self._rosprm_max_leverage)
        rosutl.set_parameters(self, self._rosprm_max_position_count)
        rosutl.set_parameters(self, self._rosprm_use_weekend_order_stop)
        rosutl.set_parameters(self, self._rosprm_weekend_order_stop_time)
        rosutl.set_parameters(self, self._rosprm_use_weekend_all_close)
        rosutl.set_parameters(self, self._rosprm_weekend_all_close_time)

        # --------------- Create State Machine ---------------
        states = [
            {
                Tr.NAME.value: self.States.Idle,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.AccountUpdating,
                Tr.ON_ENTER.value: "_on_enter_AccountUpdating",
                Tr.ON_EXIT.value: None
            }
        ]

        transitions = [
            {
                Tr.TRIGGER.value: "_trans_from_Idle_to_AccountUpdating",
                Tr.SOURCE.value: self.States.Idle,
                Tr.DEST.value: self.States.AccountUpdating,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_AccountUpdating_to_Idle",
                Tr.SOURCE.value: self.States.AccountUpdating,
                Tr.DEST.value: self.States.Idle,
                Tr.PREPARE.value: None,
                Tr.BEFORE.value: None,
                Tr.AFTER.value: None,
                Tr.CONDITIONS.value: None
            }
        ]

        self._sm = Machine(model=self,
                           states=states,
                           initial=self.States.Idle,
                           transitions=transitions)

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        # --------------- Create ROS Communication ---------------
        # Create service server "OrderRegister"
        srv_type = OrderRegisterSrv
        srv_name = "order_register"
        callback = self._on_order_register
        self._ordreq_srv = self.create_service(srv_type,
                                               srv_name,
                                               callback=callback)

        # Create service server "TradeCloseRequest"
        srv_type = TradeCloseRequestSrv
        srv_name = "trade_close_request"
        callback = self._on_requested_close
        self._trdclsreq_srv = self.create_service(srv_type,
                                                  srv_name,
                                                  callback=callback)
        try:
            # Create service client "OrderCreate"
            srv_type = OrderCreateSrv
            srv_name = "order_create"
            srvcli_ordcre = RosServiceClient(self, srv_type, srv_name)

            # Create service client "OrderDetails"
            srv_type = OrderDetailsSrv
            srv_name = "order_details"
            srvcli_orddet = RosServiceClient(self, srv_type, srv_name)

            # Create service client "OrderCancel"
            srv_type = OrderCancelSrv
            srv_name = "order_cancel"
            srvcli_ordcnc = RosServiceClient(self, srv_type, srv_name)

            # Create service client "TradeDetails"
            srv_type = TradeDetailsSrv
            srv_name = "trade_details"
            srvcli_trddet = RosServiceClient(self, srv_type, srv_name)

            # Create service client "TradeCRCDO"
            srv_type = TradeCRCDOSrv
            srv_name = "trade_crcdo"
            srvcli_trdcrc = RosServiceClient(self, srv_type, srv_name)

            # Create service client "TradeClose"
            srv_type = TradeCloseSrv
            srv_name = "trade_close"
            srvcli_trdcls = RosServiceClient(self, srv_type, srv_name)

            # Create service client "AccountQuery"
            srv_type = AccountQuerySrv
            srv_name = "account_query"
            self._srvcli_acc = RosServiceClient(self, srv_type, srv_name)

        except Exception as err:
            self.logger.error(err)
            raise InitializerErrorException("create service client failed.")

        OrderTicket.set_class_variable(srvcli_ordcre,
                                       srvcli_orddet,
                                       srvcli_ordcnc,
                                       srvcli_trddet,
                                       srvcli_trdcrc,
                                       srvcli_trdcls,
                                       self._rosprm_max_leverage.value,
                                       self._rosprm_max_position_count.value,
                                       self.logger)

        req = AccountQuerySrv.Request()
        try:
            rsp = self._srvcli_acc.call(req, timeout_sec=10.0)
        except Exception as err:
            self.logger.error("{}".format(err))
            raise InitializerErrorException("Call ROS Service Error (AccountQuerySrv)")

        OrderTicket.set_balance(rsp.balance)

        # --------------- Create topic subscriber "Pricing" ---------------
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        callback = self._on_sub_pricing_usdjpy
        self._sub_pri_usdjpy = self.create_subscription(Pricing,
                                                        "pricing_usdjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_eurjpy
        self._sub_pri_eurjpy = self.create_subscription(Pricing,
                                                        "pricing_eurjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_gbpjpy
        self._sub_pri_gbpjpy = self.create_subscription(Pricing,
                                                        "pricing_gbpjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_audjpy
        self._sub_pri_audjpy = self.create_subscription(Pricing,
                                                        "pricing_audjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_nzdjpy
        self._sub_pri_nzdjpy = self.create_subscription(Pricing,
                                                        "pricing_nzdjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_cadjpy
        self._sub_pri_cadjpy = self.create_subscription(Pricing,
                                                        "pricing_cadjpy",
                                                        callback,
                                                        qos_profile)

        callback = self._on_sub_pricing_chfjpy
        self._sub_pri_chfjpy = self.create_subscription(Pricing,
                                                        "pricing_chfjpy",
                                                        callback,
                                                        qos_profile)

        # --------------- Initialize variable ---------------
        self._tick_price_dict = {}
        self._tickets: list[OrderTicket] = []
        self._acc_trig = TimeTrigger(minute=0, second=30)

        # --------------- Restore ---------------
        # ----- Order scheduler -----
        try:
            df = bk.load_df_csv(self._BUCKUP_FULLPATH_OS)
        except FileNotFoundError:
            pass
        else:
            Col = ColOrderSchedulerBackup
            sr = df.iloc[0]
            register_id = int(sr[Col.REGISTER_ID.value])
            self.logger.info("========== Restore order scheduler ==========")
            self.logger.info("  - register_id:[{}]".format(register_id))
            OrderTicket.set_register_id(register_id)

        # ----- Order tickets -----
        try:
            df = bk.load_df_csv(self._BUCKUP_FULLPATH_OT)
        except FileNotFoundError:
            pass
        else:
            Col = ColOrderTicketsBackup
            df = df.where(df.notna(), None)
            self.logger.info("========== Restore order tickes ==========\n{}"
                             .format(df))
            for _, row in df.iterrows():
                req = OrderRegisterSrv.Request()
                tick_price = _TickPrice("", 0, 0)   # Dummy
                ticket = OrderTicket(req, tick_price, use_restore=True)
                ticket.restore(row)
                self._tickets.append(ticket)

    def finalize(self):
        # ---------- write csv ----------
        # ----- Backup order scheduler -----
        register_id = OrderTicket.get_register_id()
        bk_tbl = [register_id]
        df = pd.DataFrame(bk_tbl, columns=ColOrderSchedulerBackup.to_list())
        bk.save_df_csv(self._BUCKUP_FULLPATH_OS, df, index=False, date_format=FMT_YMDHMS)

        # ----- Backup order tickets -----
        bk_tbl = []
        for ticket in self._tickets:
            rec = ticket.generate_buckup_record()
            bk_tbl.append(rec)
        df = pd.DataFrame(bk_tbl, columns=ColOrderTicketsBackup.to_list())
        bk.save_df_csv(self._BUCKUP_FULLPATH_OT, df, index=False, date_format=FMT_YMDHMS)

    def do_cyclic_event(self) -> None:

        if self.state == self.States.Idle:
            if self._acc_trig.triggered():
                self._trans_from_Idle_to_AccountUpdating()
        elif self.state == self.States.AccountUpdating:
            if self._future.done():
                rsp = self._future.result()
                OrderTicket.set_balance(rsp.balance)
                self._trans_from_AccountUpdating_to_Idle()
            elif self._future.has_timed_out():
                self.logger.error("{:!^50}".format(" ROS Service [AccountQuerySrv] Timeout. "))
                self._trans_from_AccountUpdating_to_Idle()
        else:
            pass

        # Check weekend close proccess
        if self._rosprm_use_weekend_all_close.value:
            now = dt.datetime.now()
            if ((now.weekday() == WeekDay.SAT.value)
                    and (now.time() > self._rosprm_weekend_all_close_time.time)):
                for ticket in self._tickets:
                    ticket.enable_weekend_close()

        # Do ticket cyclic event
        for ticket in self._tickets:
            ticket.do_cyclic_event()

        # remove "Complete" States element
        len_bfr = len(self._tickets)
        self._tickets = [ticket for ticket in self._tickets
                         if ticket.state != OrderTicket.States.Complete]

        if len_bfr != len(self._tickets):
            gc.collect()
            if self.state == self.States.Idle:
                self._trans_from_Idle_to_AccountUpdating()

    def _on_enter_AccountUpdating(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        req = AccountQuerySrv.Request()
        try:
            self._future = self._srvcli_acc.call_async(req, timeout_sec=5.0)
        except Exception as err:
            self.logger.error("{:!^50}".format(err))
            self._trans_from_AccountUpdating_to_Idle()

    def _on_order_register(self,
                           req: SrvTypeRequest,
                           rsp: SrvTypeResponse
                           ) -> SrvTypeResponse:
        self.logger.debug("{:=^50}".format(" Service[order_register]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - inst_id:[{}]".format(req.inst_msg.inst_id))
        self.logger.debug("  - order_type:[{}]".format(req.ordtyp_msg.order_type))
        self.logger.debug("  - order_dir:[{}]".format(req.orddir_msg.order_dir))
        self.logger.debug("  - units:[{}]".format(req.units))
        self.logger.debug("  - entry_price:[{}]".format(req.entry_price))
        self.logger.debug("  - entry_exp_time:[{}]".format(req.entry_exp_time))
        self.logger.debug("  - take_profit_price:[{}]".format(req.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(req.stop_loss_price))
        self.logger.debug("  - exit_exp_time:[{}]".format(req.exit_exp_time))
        dbg_tm_start = dt.datetime.now()

        rsp.register_id = -1

        if (self._rosprm_use_weekend_order_stop.value
            and (dbg_tm_start.weekday() == WeekDay.SAT.value)
                and (dbg_tm_start.time() > self._rosprm_weekend_order_stop_time.time)):
            self.logger.warn("{:!^50}".format(" Reject order create "))
            self.logger.warn("  - Weekend order stop time has passed ")
            self.logger.debug("{:=^50}".format(" Service[order_register]:End "))
            return rsp

        if len(self._tickets) >= self._rosprm_max_position_count.value:
            self.logger.warn("{:!^50}".format(" Reject order create "))
            self.logger.warn("  - Positon count is full ")
            self.logger.debug("{:=^50}".format(" Service[order_register]:End "))
            return rsp

        if not self._validate_msg(req):
            self.logger.error("{:!^50}".format(" Reject order create "))
            self.logger.error("  - Validate msg: NG ")
            self.logger.debug("{:=^50}".format(" Service[order_register]:End "))
            return rsp

        tick_price = self._tick_price_dict[req.inst_msg.inst_id]
        try:
            ticket = OrderTicket(req, tick_price)
        except InitializerErrorException as err:
            self.logger.error("{:!^50}".format(" OrderTicket initialize Exception "))
            self.logger.error(err)
        else:
            self._tickets.append(ticket)
            rsp.register_id = ticket.register_id

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - register_id:[{}]".format(rsp.register_id))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Requested time:[{}]".format(dbg_tm_start))
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[order_register]:End "))

        return rsp

    def _validate_msg(self, req: SrvTypeRequest) -> Bool:

        if req.units < 0:
            self.logger.warn("units value[{}] < 0".format(req.units))
            return False

        if req.units == 0:
            try:
                tick_price = self._tick_price_dict[req.inst_msg.inst_id]
            except KeyError:
                self.logger.warn("Key[{}] is not found in a dictionary"
                                 .format(req.inst_msg.inst_id))
                return False

            if req.ordtyp_msg.order_type == OrderType.MARKET:
                msg_time = dt.datetime.strptime(tick_price.time, FMT_YMDHMSF)
                now = dt.datetime.now()
                if abs(now - msg_time) > dt.timedelta(minutes=10):
                    self.logger.warn("There's a big timedelta difference")
                    self.logger.warn("  - now time:{}".format(now))
                    self.logger.warn("  - latest tick time:{}".format(msg_time))
                    return False
        return True

    def _on_requested_close(self,
                            req: SrvTypeRequest,
                            rsp: SrvTypeResponse
                            ) -> SrvTypeResponse:
        self.logger.debug("{:=^50}".format(" Service[trade_close_request]:Start "))
        self.logger.debug("<Request>")
        self.logger.debug("  - register_id:[{}]".format(req.register_id))
        dbg_tm_start = dt.datetime.now()

        success = False
        for ticket in self._tickets:
            if ticket.register_id == req.register_id:
                success = ticket.enable_trade_close()

        rsp.success = success

        dbg_tm_end = dt.datetime.now()
        self.logger.debug("<Response>")
        self.logger.debug("  - success:[{}]".format(rsp.success))
        self.logger.debug("[Performance]")
        self.logger.debug("  - Requested time:[{}]".format(dbg_tm_start))
        self.logger.debug("  - Response time:[{}]".format(dbg_tm_end - dbg_tm_start))
        self.logger.debug("{:=^50}".format(" Service[trade_close_request]:End "))

        return rsp

    def _on_sub_pricing_usdjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_USD_JPY] = tick_price

    def _on_sub_pricing_eurjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_EUR_JPY] = tick_price
            self._tick_price_dict[InstMng.INST_EUR_USD] = tick_price

    def _on_sub_pricing_gbpjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_GBP_JPY] = tick_price

    def _on_sub_pricing_audjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_AUD_JPY] = tick_price

    def _on_sub_pricing_nzdjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_NZD_JPY] = tick_price

    def _on_sub_pricing_cadjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_CAD_JPY] = tick_price

    def _on_sub_pricing_chfjpy(self, msg: Pricing):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if (0 < len(msg.asks)) and (0 < len(msg.bids)):
            tick_price = _TickPrice(msg.time, msg.asks[0].price, msg.bids[0].price)
            self._tick_price_dict[InstMng.INST_CHF_JPY] = tick_price


def main(args=None):

    rclpy.init(args=args)
    os = OrderScheduler()

    try:
        while rclpy.ok():
            rclpy.spin_once(os, timeout_sec=1.0)
            os.do_cyclic_event()
    except KeyboardInterrupt:
        pass

    os.finalize()
    os.destroy_node()
    rclpy.shutdown()
