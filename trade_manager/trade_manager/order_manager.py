from abc import ABCMeta, abstractmethod
import datetime as dt
import sched
import time
import rclpy
from rclpy.node import Node
from trade_manager_msgs.msg import (MarketOrderRequest,
                                    LimitOrderRequest,
                                    StopOrderRequest)
from api_msgs.srv import (OrderCreateSrv, TradeDetailsSrv,
                          TradeCRCDOSrv, TradeCloseSrv,
                          OrderDetailsSrv, OrderCancelSrv)
from api_msgs.msg import OrderType, Instrument


class OrderStateAbs(metaclass=ABCMeta):

    def __init__(self):
        self._DT_FMT = "%Y-%m-%d %H:%M:%S"
        self._sts = 0

    @abstractmethod
    def set_future(self, future):
        pass


class LimitOrderState(OrderStateAbs):

    def __init__(self, msg):
        super().__init__()

        self.__inst_id = msg.instrument_id
        self.__units = msg.units
        self.__price = msg.price
        self.__dt_new = dt.datetime.strptime(
            msg.valid_period_new, self._DT_FMT)
        self.__tp_price = msg.take_profit_price
        self.__sl_price = msg.stop_loss_price
        self.__dt_settlement = dt.datetime.strptime(
            msg.valid_period_new, self._DT_FMT)
        self.__future = None

    def set_future(self, future):
        self.__future = future

    def get_future(self):
        return self.__future


class OrderManager(Node):

    def __init__(self):
        super().__init__("order_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        TPCNM_MARKET_ORDER_REQUEST = "market_order_request"
        TPCNM_LIMIT_ORDER_REQUEST = "limit_order_request"
        TPCNM_STOP_ORDER_REQUEST = "stop_order_request"

        # Declare publisher and subscriber
        msg_type = MarketOrderRequest
        topic = TPCNM_MARKET_ORDER_REQUEST
        callback = self.__on_recv_market_order_request
        self.__sub_mrk_req = self.create_subscription(msg_type,
                                                      topic,
                                                      callback)
        msg_type = LimitOrderRequest
        topic = TPCNM_LIMIT_ORDER_REQUEST
        callback = self.__on_recv_limit_order_request
        self.__sub_lim_req = self.create_subscription(msg_type,
                                                      topic,
                                                      callback)
        msg_type = StopOrderRequest
        topic = TPCNM_STOP_ORDER_REQUEST
        callback = self.__on_recv_stop_order_request
        self.__sub_stp_req = self.create_subscription(msg_type,
                                                      topic,
                                                      callback)

        # Create service client "OrderCreate"
        srv_type = OrderCreateSrv
        srv_name = "order_create"
        self.__cli_ordcre = self.__create_service_client(srv_type, srv_name)

        # Create service client "TradeDetails"
        srv_type = TradeDetailsSrv
        srv_name = "trade_details"
        self.__cli_tradet = self.__create_service_client(srv_type, srv_name)

        # Create service client "TradeCRCDO"
        srv_type = TradeCRCDOSrv
        srv_name = "trade_crcdo"
        self.__cli_tracrc = self.__create_service_client(srv_type, srv_name)

        # Create service client "TradeClose"
        srv_type = TradeCloseSrv
        srv_name = "trade_close"
        self.__cli_tracls = self.__create_service_client(srv_type, srv_name)

        # Create service client "OrderDetails"
        srv_type = OrderDetailsSrv
        srv_name = "order_details"
        self.__cli_orddet = self.__create_service_client(srv_type, srv_name)

        # Create service client "OrderCancel"
        srv_type = OrderCancelSrv
        srv_name = "order_cancel"
        self.__cli_ordcnc = self.__create_service_client(srv_type, srv_name)

        self.__ordlist = []

        DT_FMT = "%Y-%m-%d %H:%M:%S"
        dt_now = dt.datetime.now()

        self.__logger.debug("<Timer set>:dt_now")
        self.__logger.debug(dt_now.strftime(DT_FMT))
        scheduler = sched.scheduler(time.time, time.sleep)
        run_at = dt_now + dt.timedelta(seconds=10)
        self.__logger.debug("<Timer set>:run_at")
        self.__logger.debug(run_at.strftime(DT_FMT))
        run_at = int(time.mktime(run_at.utctimetuple()))
        scheduler.enterabs(run_at, 1, self.__on_date)
        scheduler.run()

    def __on_date(self):
        JST = dt.timezone(dt.timedelta(hours=+9), "JST")
        DT_FMT = "%Y-%m-%d %H:%M:%S"
        dt_now = dt.datetime.now(JST).strftime(DT_FMT)
        self.__logger.debug("AAAAAAAAAAAAAAAAAAAAAA")
        self.__logger.debug(dt_now)

    def __on_timeout(self):
        self.__logger.debug("Time out")
        lng = len(self.__ordlist)
        self.__logger.debug("list length: " + str(lng))

        for ord in self.__ordlist:
            future = ord.get_future()

    def __create_service_client(self, srv_type, srv_name):
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            self.__logger.info("Waiting for \"" + srv_name + "\" service...")
        return cli

    def __on_recv_market_order_request(self, msg):
        self.__logger.debug("topic rcv:%s" % msg)

    def __on_recv_limit_order_request(self, msg):
        self.__logger.debug("topic rcv:%s" % msg)
        obj = LimitOrderState(msg)

        req = OrderCreateSrv.Request()
        req.ordertype_msg.type = OrderType.TYP_MARKET
        req.inst_msg.instrument_id = Instrument.INST_USD_JPY
        req.units = 1000
        req.take_profit_price = 150.0
        req.stop_loss_price = 100.0
        future = self.__cli_ordcre.call_async(req)
        obj.set_future(future)

        self.__ordlist.append(obj)
        self.__tmr = self.create_timer(1.0, self.__on_timeout)

    def __on_recv_stop_order_request(self, msg):
        self.__logger.debug("topic rcv:%s" % msg)


def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()

    try:
        rclpy.spin(order_manager)
    except KeyboardInterrupt:
        pass

    order_manager.destroy_node()
    rclpy.shutdown()
