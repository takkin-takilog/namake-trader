import sys
from typing import TypeVar
from enum import Enum, auto
import datetime as dt
from transitions import Machine
# from transitions.extensions.factory import GraphMachine as Machine
from transitions.extensions.factory import GraphMachine
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.task import Future
from rclpy.client import Client
from std_msgs.msg import Bool
from trade_manager.constant import Transitions as Tr
from trade_manager.constant import FMT_DTTM_YMDHMS, FMT_DTTM_YMDHMSF
from trade_manager.exception import InitializerErrorException
from trade_manager_msgs.msg import OrderRequest
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType as OrderTypeMsg
from api_msgs.msg import OrderState as OrderStateMsg
from api_msgs.msg import TradeState as TradeStateMsg

MsgType = TypeVar("MsgType")


class OrderTicket():

    cli_ordcre = None
    cli_tradet = None
    cli_tracrc = None
    cli_tracls = None
    cli_orddet = None
    cli_ordcnc = None

    logger = None

    class States(Enum):
        EntryOrdering = auto()
        EntryWaiting = auto()
        EntryChecking = auto()
        EntryCanceling = auto()
        ExitWaiting = auto()
        ExitChecking = auto()
        ExitOrdering = auto()
        Complete = auto()

    def __init__(self, msg: MsgType) -> None:

        req = OrderCreateSrv.Request()

        if req.order_type == OrderRequest.ORDER_TYP_MARKET:
            req.ordertype_msg.type = OrderTypeMsg.TYP_MARKET
        else:
            if req.order_type == OrderRequest.ORDER_TYP_LIMIT:
                req.ordertype_msg.type = OrderTypeMsg.TYP_LIMIT
            else:
                req.ordertype_msg.type = OrderTypeMsg.TYP_STOP
            req.price = msg.entry_price

            if not msg.entry_exp_time:
                self._entry_exp_time = None
            else:
                self._entry_exp_time = dt.datetime.strptime(
                    msg.entry_exp_time, FMT_DTTM_YMDHMS)

        if not msg.exit_exp_time:
            self._exit_exp_time = None
        else:
            self._exit_exp_time = dt.datetime.strptime(
                msg.exit_exp_time, FMT_DTTM_YMDHMS)

        if req.order_dir == OrderRequest.DIR_LONG:
            req.units = req.units
        else:
            req.units = -req.units

        req.inst_msg.inst_id = msg.inst_id
        req.take_profit_price = msg.take_profit_price
        req.stop_loss_price = msg.stop_loss_price

        try:
            self._future = OrderTicket.cli_ordcre.call_async(req)
        except Exception as err:
            self.logger.error("{:!^50}".format(" Call Async ROS Service Error "))
            self.logger.error("{}".format(err))
            raise InitializerErrorException("Call Async ROS Service failed.")

        # ---------- Create State Machine ----------
        states = [
            {
                Tr.NAME.value: self.States.EntryOrdering,
                Tr.ON_ENTER.value: "_on_entry_EntryOrdering",
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryWaiting,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryChecking,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.EntryCanceling,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.ExitWaiting,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.ExitChecking,
                Tr.ON_ENTER.value: None,
                Tr.ON_EXIT.value: None
            },
            {
                Tr.NAME.value: self.States.ExitOrdering,
                Tr.ON_ENTER.value: None,
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
                Tr.CONDITIONS.value: None
            },
            {
                Tr.TRIGGER.value: "_trans_from_EntryChecking_to_EntryCanceling",
                Tr.SOURCE.value: self.States.EntryChecking,
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
                Tr.TRIGGER.value: "_trans_from_ExitChecking_to_ExitOrdering",
                Tr.SOURCE.value: self.States.ExitChecking,
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
        ]

        self._sm = Machine(model=self,
                           states=states,
                           initial=self.States.EntryOrdering,
                           transitions=transitions)

        if isinstance(self._sm, GraphMachine):
            self._sm.get_graph().view()

        self.logger.debug("{:=^50}".format(" Initialize Finish! "))

    def do_timeout_event(self):

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

    def _on_entry_EntryOrdering(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._future = None

    def _on_do_EntryOrdering(self):

        if self._future is None:
            pass
        else:
            self._trans_from_EntryOrdering_to_ExitWaiting()
            self._trans_from_EntryOrdering_to_EntryWaiting()

    def _on_do_EntryWaiting(self):
        self._trans_from_EntryWaiting_to_EntryChecking()

    def _on_do_EntryChecking(self):
        self._trans_from_EntryChecking_to_ExitWaiting()
        self._trans_from_EntryChecking_to_EntryCanceling()

    def _on_do_EntryCanceling(self):
        self._trans_from_EntryCanceling_to_Complete()

    def _on_do_ExitWaiting(self):
        self._trans_from_ExitWaiting_to_ExitChecking()

    def _on_do_ExitChecking(self):
        self._trans_from_ExitChecking_to_ExitOrdering()
        self._trans_from_ExitChecking_to_Complete()

    def _on_do_ExitOrdering(self):
        self._trans_from_ExitOrdering_to_Complete()

    def _on_do_Complete(self):
        pass


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

            # Create service client "TradeDetails"
            OrderTicket.cli_tradet = self._create_service_client(
                TradeDetailsSrv,
                "trade_details")

            # Create service client "TradeCRCDO"
            OrderTicket.cli_tracrc = self._create_service_client(
                TradeCRCDOSrv,
                "trade_crcdo")

            # Create service client "TradeClose"
            OrderTicket.cli_tracls = self._create_service_client(
                TradeCloseSrv,
                "trade_close")

            # Create service client "OrderDetails"
            OrderTicket.cli_orddet = self._create_service_client(
                OrderDetailsSrv,
                "order_details")

            # Create service client "OrderCancel"
            OrderTicket.cli_ordcnc = self._create_service_client(
                OrderCancelSrv,
                "order_cancel")

        except Exception as err:
            self.logger.error("!!!!!!!!!! Exception !!!!!!!!!!")
            self.logger.error(err)
            self.destroy_node()
            raise InitializerErrorException("create service client failed.")

    def do_timeout_event(self):
        for ticket in self._tickets:
            ticket.do_timeout_event()

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

    def _on_sub_order_request(self, msg: MsgType) -> None:
        dt_now = dt.datetime.now().strftime(FMT_DTTM_YMDHMSF)
        self.logger.debug("{:=^50}".format(" Topic[order_request]:Start "))
        self.logger.debug("  - inst_id:[{}]".format(msg.inst_id))
        self.logger.debug("  - inst_id:[{}]".format(msg.order_type))
        self.logger.debug("  - units:[{}]".format(msg.units))
        self.logger.debug("  - price:[{}]".format(msg.entry_price))
        self.logger.debug("  - entry_exp_time:[{}]".format(msg.entry_exp_time))
        self.logger.debug("  - take_profit_price:[{}]".format(msg.take_profit_price))
        self.logger.debug("  - stop_loss_price:[{}]".format(msg.stop_loss_price))
        self.logger.debug("  - exit_exp_time:[{}]".format(msg.exit_exp_time))
        self.logger.debug("[Performance]")
        self.logger.debug("  - request time:[{}]".format(dt_now))

        if msg.units < 0:
            pass
        else:
            try:
                ticket = OrderTicket(msg)
            except InitializerErrorException:
                pass
            else:
                self._tickets.append(ticket)


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
                os.do_timeout_event()
        except KeyboardInterrupt:
            pass

    os.destroy_node()
    rclpy.shutdown()
