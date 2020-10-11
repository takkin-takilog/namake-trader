from typing import TypeVar
import datetime as dt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.task import Future
from rclpy.client import Client
from std_msgs.msg import Bool
from trade_manager_msgs.msg import (MarketOrderRequest,
                                    LimitOrderRequest,
                                    StopOrderRequest)
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType as OrderTypeMsg
from api_msgs.msg import OrderState as OrderStateMsg
from api_msgs.msg import TradeState as TradeStateMsg

MsgType = TypeVar("MsgType")

_DT_FMT = "%Y-%m-%d %H:%M:%S.%f"


class OrderState(object):

    # Declare order type
    ORDER_TYP_MARKET = 1
    ORDER_TYP_LIMIT = 2
    ORDER_TYP_STOP = 3

    # Declare order state
    STS_NEW_ORD_WAIT_RSP = 1        # [1]New order wait response
    STS_NEW_ORD_PENDING = 2         # [2]New order pending
    STS_NEW_ORD_DET_WAIT_RSP = 3    # [3]Order details wait response
    STS_NEW_ORD_CNC_REQ = 4         # [4]Order cancel wait response
    STS_SET_ORD_PENDING = 5         # [5]Settlement order pending
    STS_TRADE_DET_WAIT_RSP = 6      # [6]Trade details wait response
    STS_TRADE_CLS_WAIT_RSP = 7      # [7]Trade close wait response
    STS_NOR_END = 254               # Normal end
    STS_ABN_END = 255               # Abnormal end

    def __init__(self,
                 order_typ: int,
                 msg: MsgType,
                 future: Future,
                 logger
                 ) -> None:

        self._logger = logger

        self._req_id = msg.req_id
        self._order_typ = order_typ
        self._state = self.STS_NEW_ORD_WAIT_RSP
        self._future = future
        self._order_id = 0
        self._trade_id = 0

        self._inst_id = msg.inst_id
        self._units = msg.units

        if not order_typ == self.ORDER_TYP_MARKET:
            self._price = msg.price
            if not msg.valid_period_new:
                self._dt_new = None
            else:
                self._dt_new = dt.datetime.strptime(
                    msg.valid_period_new, _DT_FMT)

        self._tp_price = msg.take_profit_price
        self._sl_price = msg.stop_loss_price

        if not msg.valid_period_settlement:
            self._dt_settlement = None
        else:
            self._dt_settlement = dt.datetime.strptime(
                msg.valid_period_settlement, _DT_FMT)

    @property
    def request_id(self) -> int:
        return self._req_id

    @property
    def state(self) -> int:
        return self._state

    @property
    def future(self) -> Future:
        return self._future

    @future.setter
    def future(self, future: Future) -> None:
        self._future = future

    @property
    def order_id(self) -> int:
        return self._order_id

    @property
    def trade_id(self) -> int:
        return self._trade_id

    def update(self, pol_flg: bool) -> None:

        sts_bfr = self._state

        if sts_bfr == self.STS_NEW_ORD_WAIT_RSP:
            sts_aft = self._check_future_by_order_create(sts_bfr)
        elif sts_bfr == self.STS_NEW_ORD_PENDING:
            sts_aft = self._check_state_new_order_pending(sts_bfr, pol_flg)
        elif sts_bfr == self.STS_NEW_ORD_DET_WAIT_RSP:
            sts_aft = self._check_future_by_order_details(sts_bfr)
        elif sts_bfr == self.STS_NEW_ORD_CNC_REQ:
            sts_aft = self._check_future_by_order_cancel(sts_bfr)
        elif sts_bfr == self.STS_SET_ORD_PENDING:
            sts_aft = self._check_state_settlement_order_pending(sts_bfr, pol_flg)
        elif sts_bfr == self.STS_TRADE_DET_WAIT_RSP:
            sts_aft = self._check_future_by_trade_details(sts_bfr)
        elif sts_bfr == self.STS_TRADE_CLS_WAIT_RSP:
            sts_aft = self._check_future_by_trade_close(sts_bfr)
        else:
            sts_aft = sts_bfr

        if not sts_bfr == sts_aft:
            self._logger.debug("Change state:[{}] -> [{}]".format(sts_bfr, sts_aft))

        self._state = sts_aft

    def _check_future_by_order_create(self, sts_bfr: int) -> int:

        sts_aft = sts_bfr
        if self._future.done():
            rsp = self._future.result()
            if rsp is None:
                self._logger.error("Error while calling service of node."
                                   "[order_create]")
                # change state: 1 -> Abnormal End
                sts_aft = self.STS_ABN_END
            else:
                if rsp.result is True:
                    if self._order_typ == self.ORDER_TYP_MARKET:
                        self._trade_id = rsp.id
                        # change state: 1 -> 5
                        sts_aft = self.STS_SET_ORD_PENDING
                    else:
                        self._order_id = rsp.id
                        # change state: 1 -> 2
                        sts_aft = self.STS_NEW_ORD_PENDING
                else:
                    self._logger.error("Service <order_create> request Failed!")
                    self._logger.error("  [reason code:{}]".format(rsp.frc_msg.reason_code))
                    # change state: 1 -> Abnormal End
                    sts_aft = self.STS_ABN_END

        return sts_aft

    def _check_state_new_order_pending(self, sts_bfr: int, pol_flg: bool) -> int:

        sts_aft = sts_bfr
        if pol_flg is True:
            # change state: 2 -> 3
            sts_aft = self.STS_NEW_ORD_DET_WAIT_RSP
        elif self._dt_new is not None:
            dt_now = dt.datetime.now()
            if self._dt_new < dt_now:
                self._logger.debug("[new_order_pending]datetime over")
                # change state: 2 -> 3
                sts_aft = self.STS_NEW_ORD_DET_WAIT_RSP

        return sts_aft

    def _check_future_by_order_details(self, sts_bfr: int) -> int:

        sts_aft = sts_bfr
        if self._future.done():
            rsp = self._future.result()
            if rsp is None:
                self._logger.error("Error while calling service of node."
                                   "[order_details]")
                # change state: 3 -> Abnormal End
                sts_aft = self.STS_ABN_END
            else:
                if rsp.result is True:
                    if rsp.order_state_msg.state == OrderStateMsg.STS_FILLED:
                        self._trade_id = rsp.open_trade_id
                        # change state: 3 -> 5
                        sts_aft = self.STS_SET_ORD_PENDING
                    elif rsp.order_state_msg.state == OrderStateMsg.STS_PENDING:
                        if self._dt_new is not None:
                            dt_now = dt.datetime.now()
                            if self._dt_new < dt_now:
                                self._logger.debug("[order_details]datetime over")
                                # change state: 3 -> 4
                                sts_aft = self.STS_NEW_ORD_CNC_REQ
                            else:
                                # change state: 3 -> 2
                                sts_aft = self.STS_NEW_ORD_PENDING
                        else:
                            # change state: 3 -> 2
                            sts_aft = self.STS_NEW_ORD_PENDING
                    else:
                        pass
                else:
                    self._logger.error("Service <order_details> request Failed!")
                    self._logger.error("  [reason code:{}]".format(rsp.frc_msg.reason_code))
                    # change state: 1 -> Abnormal End
                    sts_aft = self.STS_ABN_END

        return sts_aft

    def _check_future_by_order_cancel(self, sts_bfr: int) -> int:

        sts_aft = sts_bfr
        if self._future.done():
            rsp = self._future.result()
            if rsp is None:
                self._logger.error("Error while calling service of node."
                                   "[order_cancel]")
                # change state: 4 -> Abnormal End
                sts_aft = self.STS_ABN_END
            else:
                if rsp.result is True:
                    # change state: 4 -> Normal END
                    sts_aft = self.STS_NOR_END
                else:
                    self._logger.error("Service <order_cancel> request Failed!")
                    self._logger.error("  [reason code:{}]".format(rsp.frc_msg.reason_code))
                    # change state: 4 -> Abnormal End
                    sts_aft = self.STS_ABN_END

        return sts_aft

    def _check_state_settlement_order_pending(self, sts_bfr: int, pol_flg: bool) -> int:

        sts_aft = sts_bfr
        if pol_flg is True:
            # change state: 5 -> 6
            sts_aft = self.STS_TRADE_DET_WAIT_RSP
        elif self._dt_settlement is not None:
            dt_now = dt.datetime.now()
            if self._dt_settlement < dt_now:
                self._logger.debug("[settlement_order_pending]datetime over")
                # change state: 5 -> 6
                sts_aft = self.STS_TRADE_DET_WAIT_RSP

        return sts_aft

    def _check_future_by_trade_details(self, sts_bfr: int) -> int:

        sts_aft = sts_bfr
        if self._future.done():
            rsp = self._future.result()
            if rsp is None:
                self._logger.error("Error while calling service of node."
                                   "[trade_details]")
                # change state: 6 -> Abnormal End
                sts_aft = self.STS_ABN_END
            else:
                if rsp.result is True:
                    if rsp.trade_state_msg.state == TradeStateMsg.STS_CLOSED:
                        # change state: 6 -> Normal END
                        sts_aft = self.STS_NOR_END
                    elif rsp.trade_state_msg.state == TradeStateMsg.STS_OPEN:
                        dt_now = dt.datetime.now()
                        if self._dt_settlement < dt_now:
                            self._logger.debug("[trade_details]datetime over")
                            # change state: 6 -> 7
                            sts_aft = self.STS_TRADE_CLS_WAIT_RSP
                        else:
                            # change state: 6 -> 5
                            sts_aft = self.STS_SET_ORD_PENDING
                    else:
                        # change state: 6 -> 5
                        sts_aft = self.STS_SET_ORD_PENDING
                else:
                    self._logger.error("Service <trade_details> request Failed!")
                    self._logger.error("  [reason code:{}]".format(rsp.frc_msg.reason_code))
                    # change state: 6 -> Abnormal End
                    sts_aft = self.STS_ABN_END

        return sts_aft

    def _check_future_by_trade_close(self, sts_bfr: int) -> int:

        sts_aft = sts_bfr
        if self._future.done():
            rsp = self._future.result()
            if rsp is None:
                self._logger.error("Error while calling service of node."
                                   "[trade_close]")
                # change state: 7 -> Abnormal End
                sts_aft = self.STS_ABN_END
            else:
                if rsp.result is True:
                    # change state: 7 -> Normal END
                    sts_aft = self.STS_NOR_END
                else:
                    self._logger.error("Service <trade_close> request Failed!")
                    self._logger.error("  [reason code:{}]".format(rsp.frc_msg.reason_code))
                    # change state: 7 -> Abnormal End
                    sts_aft = self.STS_ABN_END

        return sts_aft


class OrderManager(Node):

    def __init__(self) -> None:
        super().__init__("order_manager")

        # Set logger lebel
        self._logger = super().get_logger()
        self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        TPCNM_MARKET_ORDER_REQUEST = "market_order_request"
        TPCNM_LIMIT_ORDER_REQUEST = "limit_order_request"
        TPCNM_STOP_ORDER_REQUEST = "stop_order_request"
        TPCNM_POLLING = "polling"

        # Declare publisher and subscriber
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        msg_type = MarketOrderRequest
        topic = TPCNM_MARKET_ORDER_REQUEST
        callback = self._on_subs_market_order_request
        self._sub_mrk_req = self.create_subscription(msg_type,
                                                     topic,
                                                     callback,
                                                     qos_profile)
        msg_type = LimitOrderRequest
        topic = TPCNM_LIMIT_ORDER_REQUEST
        callback = self._on_subs_limit_order_request
        self._sub_lim_req = self.create_subscription(msg_type,
                                                     topic,
                                                     callback,
                                                     qos_profile)
        msg_type = StopOrderRequest
        topic = TPCNM_STOP_ORDER_REQUEST
        callback = self._on_subs_stop_order_request
        self._sub_stp_req = self.create_subscription(msg_type,
                                                     topic,
                                                     callback,
                                                     qos_profile)

        msg_type = Bool
        topic = TPCNM_POLLING
        callback = self._on_subs_polling
        self._sub_stp_req = self.create_subscription(msg_type,
                                                     topic,
                                                     callback,
                                                     qos_profile)

        try:
            # Create service client "OrderCreate"
            srv_type = OrderCreateSrv
            srv_name = "order_create"
            self._cli_ordcre = self._create_service_client(srv_type,
                                                           srv_name)

            # Create service client "TradeDetails"
            srv_type = TradeDetailsSrv
            srv_name = "trade_details"
            self._cli_tradet = self._create_service_client(srv_type,
                                                           srv_name)

            # Create service client "TradeCRCDO"
            srv_type = TradeCRCDOSrv
            srv_name = "trade_crcdo"
            self._cli_tracrc = self._create_service_client(srv_type,
                                                           srv_name)

            # Create service client "TradeClose"
            srv_type = TradeCloseSrv
            srv_name = "trade_close"
            self._cli_tracls = self._create_service_client(srv_type,
                                                           srv_name)

            # Create service client "OrderDetails"
            srv_type = OrderDetailsSrv
            srv_name = "order_details"
            self._cli_orddet = self._create_service_client(srv_type,
                                                           srv_name)

            # Create service client "OrderCancel"
            srv_type = OrderCancelSrv
            srv_name = "order_cancel"
            self._cli_ordcnc = self._create_service_client(srv_type,
                                                           srv_name)
        except RuntimeError as err:
            self._logger.error(err)
            self.destroy_node()
            rclpy.shutdown()

        self._ordlist = []
        self._timer_1sec = self.create_timer(1.0, self._on_timeout_1sec)

    def _on_timeout_1sec(self) -> None:
        self._update_state()

    def _update_state(self, pol_flg: bool=False) -> None:

        ordlist = self._ordlist
        for order in ordlist:
            pre_sts = order.state
            # Update order state.
            order.update(pol_flg)
            # change state action.
            self._change_state_action(order, pre_sts)

            # ---------- for debug ----------
            """
            dt_now = dt.datetime.now().strftime(_DT_FMT)
            self._logger.debug("========== chack! ==========")
            self._logger.debug("----- Time: {}".format(dt_now))
            self._logger.debug("----- ReqID: {} -----".format(order.request_id))
            self._logger.debug("      State: {}".format(order.state))
            self._logger.debug("      OrderID: {}".format(order.order_id))
            self._logger.debug("      TradeID: {}".format(order.trade_id))
            """

        # remove end state
        self._ordlist = [o for o in ordlist if not (
            (o.state == OrderState.STS_NOR_END) or
            (o.state == OrderState.STS_ABN_END))]

    def _change_state_action(self, order: OrderState, pre_sts: int) -> None:

        if pre_sts == OrderState.STS_NEW_ORD_PENDING:
            if order.state == OrderState.STS_NEW_ORD_DET_WAIT_RSP:
                # change state: 2 -> 3
                req = OrderDetailsSrv.Request()
                req.order_id = order.order_id
                order.future = self._cli_orddet.call_async(req)
        elif pre_sts == OrderState.STS_NEW_ORD_DET_WAIT_RSP:
            if order.state == OrderState.STS_NEW_ORD_CNC_REQ:
                # change state: 3 -> 4
                req = OrderCancelSrv.Request()
                req.order_id = order.order_id
                order.future = self._cli_ordcnc.call_async(req)
        elif pre_sts == OrderState.STS_SET_ORD_PENDING:
            if order.state == OrderState.STS_TRADE_DET_WAIT_RSP:
                # change state: 5 -> 6
                req = TradeDetailsSrv.Request()
                req.trade_id = order.trade_id
                order.future = self._cli_tradet.call_async(req)
        elif pre_sts == OrderState.STS_TRADE_DET_WAIT_RSP:
            if order.state == OrderState.STS_TRADE_CLS_WAIT_RSP:
                # change state: 6 -> 7
                req = TradeCloseSrv.Request()
                req.trade_id = order.trade_id
                order.future = self._cli_tracls.call_async(req)
        else:
            pass

    def _create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self._logger.info("Waiting for [{}] service...".format(srv_name))
        return cli

    def _on_subs_market_order_request(self, msg: MsgType) -> None:
        dt_now = dt.datetime.now().strftime(_DT_FMT)
        logger = self._logger
        logger.debug("{:=^50}".format(" Topic[market_order_request]:Start "))
        logger.debug("  - req_id:[{}]".format(msg.req_id))
        logger.debug("  - inst_id:[{}]".format(msg.inst_id))
        logger.debug("  - units:[{}]".format(msg.units))
        logger.debug("  - take_profit_price:[{}]".format(msg.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(msg.stop_loss_price))
        logger.debug("  - valid_period_settlement:[{}]".format(msg.valid_period_settlement))
        logger.debug("[Performance]")
        logger.debug("  - request time:[{}]".format(dt_now))

        req = OrderCreateSrv.Request()
        req.ordertype_msg.type = OrderTypeMsg.TYP_MARKET
        req.inst_msg.inst_id = msg.inst_id
        req.units = msg.units
        req.take_profit_price = msg.take_profit_price
        req.stop_loss_price = msg.stop_loss_price
        future = self._cli_ordcre.call_async(req)

        order_typ = OrderState.ORDER_TYP_MARKET
        obj = OrderState(order_typ, msg, future, logger)
        self._ordlist.append(obj)

    def _on_subs_limit_order_request(self, msg: MsgType) -> None:
        dt_now = dt.datetime.now().strftime(_DT_FMT)
        logger = self._logger
        logger.debug("{:=^50}".format(" Topic[limit_order_request]:Start "))
        logger.debug("  - req_id:[{}]".format(msg.req_id))
        logger.debug("  - inst_id:[{}]".format(msg.inst_id))
        logger.debug("  - units:[{}]".format(msg.units))
        logger.debug("  - price:[{}]".format(msg.price))
        logger.debug("  - valid_period_new:[{}]".format(msg.valid_period_new))
        logger.debug("  - take_profit_price:[{}]".format(msg.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(msg.stop_loss_price))
        logger.debug("  - valid_period_settlement:[{}]".format(msg.valid_period_settlement))
        logger.debug("[Performance]")
        logger.debug("  - request time:[{}]".format(dt_now))

        req = OrderCreateSrv.Request()
        req.ordertype_msg.type = OrderTypeMsg.TYP_LIMIT
        req.inst_msg.inst_id = msg.inst_id
        req.units = msg.units
        req.price = msg.price
        req.take_profit_price = msg.take_profit_price
        req.stop_loss_price = msg.stop_loss_price
        future = self._cli_ordcre.call_async(req)

        order_typ = OrderState.ORDER_TYP_LIMIT
        obj = OrderState(order_typ, msg, future, logger)
        self._ordlist.append(obj)

    def _on_subs_stop_order_request(self, msg: MsgType) -> None:
        dt_now = dt.datetime.now().strftime(_DT_FMT)
        logger = self._logger
        logger.debug("{:=^50}".format(" Topic[stop_order_request]:Start "))
        logger.debug("  - req_id:[{}]".format(msg.req_id))
        logger.debug("  - inst_id:[{}]".format(msg.inst_id))
        logger.debug("  - units:[{}]".format(msg.units))
        logger.debug("  - price:[{}]".format(msg.price))
        logger.debug("  - valid_period_new:[{}]".format(msg.valid_period_new))
        logger.debug("  - take_profit_price:[{}]".format(msg.take_profit_price))
        logger.debug("  - stop_loss_price:[{}]".format(msg.stop_loss_price))
        logger.debug("  - valid_period_settlement:[{}]".format(msg.valid_period_settlement))
        logger.debug("[Performance]")
        logger.debug("  - request time:[{}]".format(dt_now))

        req = OrderCreateSrv.Request()
        req.ordertype_msg.type = OrderTypeMsg.TYP_STOP
        req.inst_msg.inst_id = msg.inst_id
        req.units = msg.units
        req.price = msg.price
        req.take_profit_price = msg.take_profit_price
        req.stop_loss_price = msg.stop_loss_price
        future = self._cli_ordcre.call_async(req)

        order_typ = OrderState.ORDER_TYP_STOP
        obj = OrderState(order_typ, msg, future, logger)
        self._ordlist.append(obj)

    def _on_subs_polling(self, msg: MsgType) -> None:
        logger = self._logger
        logger.debug("{:=^50}".format(" Topic[polling]:Start "))
        if msg.data is True:
            self._update_state(True)


def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()

    try:
        rclpy.spin(order_manager)
    except KeyboardInterrupt:
        pass

    order_manager.destroy_node()
    rclpy.shutdown()
