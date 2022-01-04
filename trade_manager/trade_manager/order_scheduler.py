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
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.client import Client
from std_msgs.msg import Bool
from trade_manager.constant import FMT_YMDHMS, FMT_YMDHMSF
from trade_manager.exception import InitializerErrorException
from trade_manager.data import Transitions as Tr
from trade_manager.data import INST_DICT, ORDER_TYP_DICT
from trade_manager_msgs.msg import OrderRequest
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderState, TradeState
from api_msgs.msg import FailReasonCode as frc

MsgType = TypeVar("MsgType")


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

    cli_ordcre = None
    cli_orddet = None
    cli_ordcnc = None
    cli_trddet = None
    cli_trdcrc = None
    cli_trdcls = None

    logger = None

    _is_trans_lock = False

    def __init__(self, msg: MsgType) -> None:

        # Define Constant value.
        self._POL_INTERVAL = dt.timedelta(minutes=1)

        # ---------- Create State Machine ----------
        states = [
            {
                Tr.NAME.value: self.States.EntryOrdering,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryWaiting,
                Tr.ON_ENTER.value: "_on_entry_EntryWaiting",
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
                Tr.ON_ENTER.value: "_on_entry_ExitWaiting",
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
            },
        ]

        self._sm = Machine(model=self,
                           states=states,
                           initial=self.States.EntryOrdering,
                           transitions=transitions)

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        self._msg = msg
        self._future = None
        self._trade_id = None
        self._order_id = None

        if ((self._msg.order_type == OrderRequest.ORDER_TYP_MARKET)
                or (not self._msg.entry_exp_time)):
            self._entry_exp_time = None
        else:
            self._entry_exp_time = dt.datetime.strptime(self._msg.entry_exp_time,
                                                        FMT_YMDHMS)

        if not self._msg.exit_exp_time:
            self._exit_exp_time = None
        else:
            self._exit_exp_time = dt.datetime.strptime(self._msg.exit_exp_time,
                                                       FMT_YMDHMS)

        self._is_entry_exp_time_over = False

        self.logger.debug("----- init -----")
        if not self._msg.order_type == OrderRequest.ORDER_TYP_MARKET:
            self.logger.debug("  - entry_exp_time:[{}]".format(self._entry_exp_time))
        self.logger.debug("  - exit_exp_time:[{}]".format(self._exit_exp_time))

        self.do_cyclic_event()

    def __del__(self) -> None:
        self.logger.debug("----- del -----")
        self.logger.debug("  - order_id:[{}]".format(self._order_id))
        self.logger.debug("  - trade_id:[{}]".format(self._trade_id))

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

        if self._future is None:
            req = OrderCreateSrv.Request()
            req.ordertype_msg.type = ORDER_TYP_DICT[self._msg.order_type]

            if self._msg.order_type == OrderRequest.ORDER_TYP_MARKET:
                req.price = 0.0
            else:
                req.price = self._msg.entry_price

            if self._msg.order_dir == OrderRequest.DIR_LONG:
                req.units = self._msg.units
            else:
                req.units = -self._msg.units

            req.inst_msg.inst_id = INST_DICT[self._msg.inst_msg.inst_id]
            req.take_profit_price = self._msg.take_profit_price
            req.stop_loss_price = self._msg.stop_loss_price

            self.logger.debug("----- Requesting \"Order Create\" -----")
            try:
                self._future = OrderTicket.cli_ordcre.call_async(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
                self.logger.error("{}".format(err))
                self._trans_to_Complete()
        else:
            if self._future.done():
                self.logger.debug("  Request done.")
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        if self._msg.order_type == OrderRequest.ORDER_TYP_MARKET:
                            self._trade_id = rsp.id
                            self.logger.debug("  - trade_id:[{}]".format(self._trade_id))
                            self._trans_from_EntryOrdering_to_ExitWaiting()
                        else:
                            self._order_id = rsp.id
                            self.logger.debug("  - order_id:[{}]".format(self._order_id))
                            self._trans_from_EntryOrdering_to_EntryWaiting()
                    else:
                        self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Create) "))
                        self._trans_to_Complete()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Create) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_to_Complete()
            else:
                self.logger.debug("  Requesting now...")

    def _on_entry_EntryWaiting(self) -> None:
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
        self.logger.debug("--- trans_lock state:[{}]".format(OrderTicket._is_trans_lock))
        return not OrderTicket._is_trans_lock

    def _on_enter_EntryChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None
        OrderTicket._is_trans_lock = True
        self.logger.debug("--- Trans \"Locked\"")

    def _on_do_EntryChecking(self) -> None:

        if self._future is None:
            req = OrderDetailsSrv.Request()
            req.order_id = self._order_id
            self.logger.debug("----- Requesting \"Order Details\" (id:[{}]) -----"
                              .format(self._order_id))
            try:
                self._future = OrderTicket.cli_orddet.call_async(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
                self.logger.error("{}".format(err))
                self._trans_from_EntryChecking_to_EntryWaiting()
        else:
            if self._future.done():
                self.logger.debug("  Request done.(id:[{}])".format(self._order_id))
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        if rsp.order_state_msg.state == OrderState.STS_PENDING:
                            self.logger.debug("  - order id:[{}] is Pending.".format(self._order_id))
                            self._trans_from_EntryChecking_to_EntryWaiting()
                        elif rsp.order_state_msg.state == OrderState.STS_FILLED:
                            self._trade_id = rsp.open_trade_id
                            self.logger.debug("  - order_id:[{}] is Filled.".format(self._order_id))
                            self.logger.debug("  - trade_id:[{}] is Opened.".format(self._trade_id))
                            self._trans_from_EntryChecking_to_ExitWaiting()
                        else:
                            self.logger.debug("  - order id:[{}] is Unexpected State! (State No:<{}>)"
                                              .format(self._order_id, rsp.order_state_msg.state))
                            self._trans_from_EntryChecking_to_EntryWaiting()
                    else:
                        self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Details) "))
                        self._trans_from_EntryChecking_to_EntryWaiting()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Details) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_from_EntryChecking_to_EntryWaiting()
            else:
                self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))

    def _on_exit_EntryChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        OrderTicket._is_trans_lock = False
        self.logger.debug("--- Trans \"Unlocked\"")

    def _on_enter_EntryCanceling(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None
        self._on_do_EntryCanceling()

    def _on_do_EntryCanceling(self) -> None:

        if self._future is None:
            req = OrderCancelSrv.Request()
            req.order_id = self._order_id
            self.logger.debug("----- Requesting \"Order Cancel\" (id:[{}]) -----"
                              .format(self._order_id))
            try:
                self._future = OrderTicket.cli_ordcnc.call_async(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
                self.logger.error("{}".format(err))
                self._trans_to_Complete()
        else:
            if self._future.done():
                self.logger.debug("  Request done.(id:[{}])".format(self._order_id))
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        self.logger.debug("  EntryCanceling complete.(id:[{}])".format(self._order_id))
                        self._trans_from_EntryCanceling_to_Complete()
                    else:
                        if rsp.frc_msg.reason_code == frc.REASON_ORDER_DOESNT_EXIST:
                            self.logger.debug("  EntryCanceling fail.(id:[{}])".format(self._order_id))
                            self.logger.debug("    - Order doesnt exist!")
                            self._trans_from_EntryCanceling_to_EntryChecking()
                        else:
                            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Order Cancel) "))
                            self._trans_to_Complete()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Order Cancel) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_to_Complete()
            else:
                self.logger.debug("  Requesting now...(id:[{}])".format(self._order_id))

    def _on_entry_ExitWaiting(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._next_pol_time = self._update_next_pollingtime(dt.datetime.now())

    def _on_do_ExitWaiting(self) -> None:
        now = dt.datetime.now()
        if ((self._exit_exp_time is not None) and (self._exit_exp_time < now)):
            self._trans_from_ExitWaiting_to_ExitOrdering()
        elif self._next_pol_time < now:
            self.logger.debug("<<< Timeout >>> in ExitWaiting")
            self._trans_from_ExitWaiting_to_ExitChecking()
        else:
            pass

    def _on_enter_ExitChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None
        OrderTicket._is_trans_lock = True
        self.logger.debug("--- Trans \"Locked\"")

    def _on_do_ExitChecking(self) -> None:

        if self._future is None:
            req = TradeDetailsSrv.Request()
            req.trade_id = self._trade_id
            self.logger.debug("----- Requesting \"Trade Details\" (id:[{}]) -----"
                              .format(self._trade_id))
            try:
                self._future = OrderTicket.cli_trddet.call_async(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
                self.logger.error("{}".format(err))
                self._trans_from_ExitChecking_to_ExitWaiting()
        else:
            if self._future.done():
                self.logger.debug("  Request done.(id:[{}])".format(self._trade_id))
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        if rsp.trade_state_msg.state == TradeState.STS_OPEN:
                            self.logger.debug("  - trade id:[{}] is Opening.".format(self._trade_id))
                            self._trans_from_ExitChecking_to_ExitWaiting()
                        elif rsp.trade_state_msg.state == TradeState.STS_CLOSED:
                            self.logger.debug("  - trade id:[{}] is Closed.".format(self._trade_id))
                            self._trans_from_ExitChecking_to_Complete()
                        else:
                            self.logger.debug("  - trade id:[{}] is Unexpected State! (State No:<{}>)"
                                              .format(self._trade_id, rsp.trade_state_msg.state))
                            self._trans_from_ExitChecking_to_ExitWaiting()
                    else:
                        self.logger.error("{:!^50}".format(" Call ROS Service Fail (Trade Details) "))
                        self._trans_from_ExitChecking_to_ExitWaiting()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Details) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_from_ExitChecking_to_ExitWaiting()
            else:
                self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))

    def _on_exit_ExitChecking(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        OrderTicket._is_trans_lock = False
        self.logger.debug("--- Trans \"Unlocked\"")

    def _on_enter_ExitOrdering(self) -> None:
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None
        self._on_do_ExitOrdering()

    def _on_do_ExitOrdering(self) -> None:

        if self._future is None:
            req = TradeCloseSrv.Request()
            req.trade_id = self._trade_id
            self.logger.debug("----- Requesting \"Trade Close\" (id:[{}]) -----"
                              .format(self._trade_id))
            try:
                self._future = OrderTicket.cli_trdcls.call_async(req)
            except Exception as err:
                self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
                self.logger.error("{}".format(err))
                self._trans_to_Complete()
        else:
            if self._future.done():
                self.logger.debug("  Request done.(id:[{}])".format(self._trade_id))
                if self._future.result() is not None:
                    rsp = self._future.result()
                    if rsp.result:
                        self._trans_from_ExitOrdering_to_Complete()
                    else:
                        if rsp.frc_msg.reason_code == frc.REASON_TRADE_DOESNT_EXIST:
                            self._trans_from_ExitOrdering_to_Complete()
                        else:
                            self.logger.error("{:!^50}".format(" Call ROS Service Fail (Trade Close) "))
                            self._trans_to_Complete()
                else:
                    self.logger.error("{:!^50}".format(" Call ROS Service Error (Trade Close) "))
                    self.logger.error("  future.result() is \"None\".")
                    self._trans_to_Complete()
            else:
                self.logger.debug("  Requesting now...(id:[{}])".format(self._trade_id))

    def _on_do_Complete(self) -> None:
        pass

    def _update_next_pollingtime(self, time: dt.datetime) -> dt.datetime:
        next_time = time.replace(second=10, microsecond=0) + self._POL_INTERVAL
        self.logger.debug(" - update polling time:{}".format(next_time))
        return next_time


class OrderScheduler(Node):

    def __init__(self) -> None:
        super().__init__("order_scheduler")

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        OrderTicket.logger = self.logger

        self._tickets: list[OrderTicket] = []

        TPCNM_ORDER_REQUEST = "order_request"

        # Declare publisher and subscriber
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        msg_type = OrderRequest
        topic = TPCNM_ORDER_REQUEST
        callback = self._on_sub_order_request
        self._sub_req = self.create_subscription(msg_type,
                                                 topic,
                                                 callback,
                                                 qos_profile)

        try:
            # Create service client "OrderCreate"
            OrderTicket.cli_ordcre = self._create_service_client(
                OrderCreateSrv,
                "order_create")

            # Create service client "OrderDetails"
            OrderTicket.cli_orddet = self._create_service_client(
                OrderDetailsSrv,
                "order_details")

            # Create service client "OrderCancel"
            OrderTicket.cli_ordcnc = self._create_service_client(
                OrderCancelSrv,
                "order_cancel")

            # Create service client "TradeDetails"
            OrderTicket.cli_trddet = self._create_service_client(
                TradeDetailsSrv,
                "trade_details")

            # Create service client "TradeCRCDO"
            OrderTicket.cli_trdcrc = self._create_service_client(
                TradeCRCDOSrv,
                "trade_crcdo")

            # Create service client "TradeClose"
            OrderTicket.cli_trdcls = self._create_service_client(
                TradeCloseSrv,
                "trade_close")

        except Exception as err:
            self.logger.error("{:!^50}".format(" Exception "))
            self.logger.error(err)
            self.destroy_node()
            raise InitializerErrorException("create service client failed.")

    def do_cyclic_event(self) -> None:

        for ticket in self._tickets:
            ticket.do_cyclic_event()

        # remove "Complete" States element
        self._tickets = [ticket for ticket in self._tickets
                         if ticket.state != OrderTicket.States.Complete]
        gc.collect()

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        cli = self.create_client(srv_type, srv_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

    def _on_sub_order_request(self, msg: MsgType) -> None:
        dt_now = dt.datetime.now().strftime(FMT_YMDHMSF)
        self.logger.debug("{:=^50}".format(" Topic[order_request]:Start "))
        self.logger.debug("  - inst_id:[{}]".format(msg.inst_msg.inst_id))
        self.logger.debug("  - order_type:[{}]".format(msg.order_type))
        self.logger.debug("  - order_dir:[{}]".format(msg.order_dir))
        self.logger.debug("  - units:[{}]".format(msg.units))
        self.logger.debug("  - entry_price:[{}]".format(msg.entry_price))
        self.logger.debug("  - entry_exp_time:[{}]".format(msg.entry_exp_time))
        self.logger.debug("  - take_profit_price:[{}]".format(msg.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(msg.stop_loss_price))
        self.logger.debug("  - exit_exp_time:[{}]".format(msg.exit_exp_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - request time:[{}]".format(dt_now))

        if self._validate_msg(msg):
            try:
                ticket = OrderTicket(msg)
            except Exception as err:
                self.logger.error("{:!^50}".format(" OrderTicket initialize Exception "))
                self.logger.error(err)
            else:
                self._tickets.append(ticket)
        else:
            self.logger.error("{:!^50}".format(" Validate msg: NG "))

    def _validate_msg(self, msg: MsgType) -> Bool:

        if msg.units < 0:
            return False

        return True


def main(args=None):

    rclpy.init(args=args)

    try:
        os = OrderScheduler()
    except InitializerErrorException:
        pass
    else:
        try:
            while rclpy.ok():
                rclpy.spin_once(os, timeout_sec=1.0)
                os.do_cyclic_event()
        except KeyboardInterrupt:
            pass

        os.destroy_node()

    rclpy.shutdown()
