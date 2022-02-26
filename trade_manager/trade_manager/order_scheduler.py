import sys
import gc
from typing import TypeVar
from enum import Enum, auto
import datetime as dt
from transitions import Machine
# from transitions.extensions.factory import GraphMachine as Machine
from transitions.extensions.factory import GraphMachine
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from std_msgs.msg import Bool
from trade_manager_msgs.msg import OrderType, OrderDir
from trade_manager_msgs.srv import OrderRegisterSrv
from trade_manager_msgs.srv import TradeCloseRequestSrv
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderState, TradeState
from api_msgs.msg import OrderType as ApiOrderType
from api_msgs.msg import FailReasonCode as frc
from .constant import FMT_YMDHMS
from .constant import Transitions as Tr
from .constant import INST_DICT
from .exception import InitializerErrorException


MsgType = TypeVar("MsgType")
SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


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
    _c_srvcli_ordcre = None
    _c_srvcli_orddet = None
    _c_srvcli_ordcnc = None
    _c_srvcli_trddet = None
    _c_srvcli_trdcrc = None
    _c_srvcli_trdcls = None
    _c_is_trans_lock = False
    _c_register_id = 1
    logger = None

    @classmethod
    def set_class_variable(cls,
                           srvcli_ordcre: Client,
                           srvcli_orddet: Client,
                           srvcli_ordcnc: Client,
                           srvcli_trddet: Client,
                           srvcli_trdcrc: Client,
                           srvcli_trdcls: Client,
                           logger
                           ) -> None:
        cls._c_srvcli_ordcre = srvcli_ordcre
        cls._c_srvcli_orddet = srvcli_orddet
        cls._c_srvcli_ordcnc = srvcli_ordcnc
        cls._c_srvcli_trddet = srvcli_trddet
        cls._c_srvcli_trdcrc = srvcli_trdcrc
        cls._c_srvcli_trdcls = srvcli_trdcls
        cls.logger = logger

    def __init__(self, req: SrvTypeRequest) -> None:

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

        self._inst_id = req.inst_msg.inst_id
        self._order_type = req.ordtyp_msg.order_type

        if req.orddir_msg.order_dir == OrderDir.LONG:
            self._units = req.units
        else:
            self._units = -req.units

        if self._order_type == OrderType.MARKET:
            self._entry_price = 0.0
        else:
            self._entry_price = req.entry_price

        if ((self._order_type == OrderType.MARKET) or (not req.entry_exp_time)):
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

        self._is_entry_exp_time_over = False
        self._enable_trade_close = False

        self.logger.debug("---------- Create OrderTicket ----------")
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
        self._register_id = self._c_register_id
        self._c_register_id += 1

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
            self._future = self._c_srvcli_ordcre.call_async(req)
        except InitializerErrorException as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
            self.logger.error("{}".format(err))

    def __del__(self) -> None:
        self.logger.debug("---------- Delete OrderTicket ----------")
        self.logger.debug("  - order_id:[{}]".format(self._order_id))
        self.logger.debug("  - trade_id:[{}]".format(self._trade_id))

    @property
    def register_id(self) -> int:
        return self._register_id

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

        self.logger.debug("state:[{}]".format(self.state))

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
        elif ((self._entry_exp_time is not None) and (self._entry_exp_time < now)):
            self._trans_from_EntryWaiting_to_EntryCanceling()
            self._is_entry_exp_time_over = True
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
            self._future = self._c_srvcli_orddet.call_async(req)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
            self.logger.error("{}".format(err))

    def _on_do_EntryChecking(self) -> None:

        if self._future is None:
            self._trans_from_EntryChecking_to_EntryWaiting()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))
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
            self._future = self._c_srvcli_ordcnc.call_async(req)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
            self.logger.error("{}".format(err))

    def _on_do_EntryCanceling(self) -> None:

        if self._future is None:
            self._trans_to_Complete()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))
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
        if (self._enable_trade_close
                or ((self._exit_exp_time is not None) and (self._exit_exp_time < now))):
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
            self._future = self._c_srvcli_trddet.call_async(req)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
            self.logger.error("{}".format(err))

    def _on_do_ExitChecking(self) -> None:

        if self._future is None:
            self._trans_from_ExitChecking_to_ExitWaiting()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))
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
            self._future = self._c_srvcli_trdcls.call_async(req)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
            self.logger.error("{}".format(err))

    def _on_do_ExitOrdering(self) -> None:

        if self._future is None:
            self._trans_to_Complete()
            return

        if not self._future.done():
            self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))
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

    def __init__(self) -> None:
        super().__init__("order_scheduler")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self._tickets: list[OrderTicket] = []

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
            srvcli_ordcre = self._create_service_client(OrderCreateSrv,
                                                        "order_create")

            # Create service client "OrderDetails"
            srvcli_orddet = self._create_service_client(OrderDetailsSrv,
                                                        "order_details")

            # Create service client "OrderCancel"
            srvcli_ordcnc = self._create_service_client(OrderCancelSrv,
                                                        "order_cancel")

            # Create service client "TradeDetails"
            srvcli_trddet = self._create_service_client(TradeDetailsSrv,
                                                        "trade_details")

            # Create service client "TradeCRCDO"
            srvcli_trdcrc = self._create_service_client(TradeCRCDOSrv,
                                                        "trade_crcdo")

            # Create service client "TradeClose"
            srvcli_trdcls = self._create_service_client(TradeCloseSrv,
                                                        "trade_close")

        except Exception as err:
            self.logger.error("{:!^50}".format(" Exception "))
            self.logger.error(err)
            self.destroy_node()
            raise InitializerErrorException("create service client failed.")

        OrderTicket.set_class_variable(srvcli_ordcre,
                                       srvcli_orddet,
                                       srvcli_ordcnc,
                                       srvcli_trddet,
                                       srvcli_trdcrc,
                                       srvcli_trdcls,
                                       self.logger)

    def do_cyclic_event(self) -> None:

        for ticket in self._tickets:
            ticket.do_cyclic_event()

        # remove "Complete" States element
        len_bfr = len(self._tickets)
        self._tickets = [ticket for ticket in self._tickets
                         if ticket.state != OrderTicket.States.Complete]

        if len_bfr != len(self._tickets):
            gc.collect()

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        cli = self.create_client(srv_type, srv_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

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
        if self._validate_msg(req):
            try:
                ticket = OrderTicket(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" OrderTicket initialize Exception "))
                self.logger.error(err)
            else:
                self._tickets.append(ticket)
                rsp.register_id = ticket.register_id
        else:
            self.logger.error("{:!^50}".format(" Validate msg: NG "))

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


def main(args=None):

    rclpy.init(args=args)
    os = OrderScheduler()

    try:
        while rclpy.ok():
            rclpy.spin_once(os, timeout_sec=1.0)
            os.do_cyclic_event()
    except KeyboardInterrupt:
        pass

    os.destroy_node()
    rclpy.shutdown()
