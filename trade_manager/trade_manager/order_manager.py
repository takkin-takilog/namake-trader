import queue
import rclpy
from rclpy.node import Node
from trade_manager_msgs.msg import MarketOrderRequest, LimitStopOrderRequest


class OrderManager(Node):

    def __init__(self):
        super().__init__("order_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        TPCNM_MARKET_ORDER_REQUEST = "mamrket_order_request"
        TPCNM_LIMIT_STOP_ORDER_REQUEST = "limit_stop_order_request"

        # Declare publisher and subscriber
        msg_type = MarketOrderRequest
        topic = TPCNM_MARKET_ORDER_REQUEST
        callback = self.__on_recv_market_order_request
        self.__sub_mrk_req = self.create_subscription(msg_type,
                                                      topic,
                                                      callback)
        msg_type = LimitStopOrderRequest
        topic = TPCNM_LIMIT_STOP_ORDER_REQUEST
        callback = self.__on_recv_lmst_order_request
        self.__sub_mrk_req = self.create_subscription(msg_type,
                                                      topic,
                                                      callback)

        self.__mark_que = queue.Queue()   # Queue for market order
        self.__lmst_que = queue.Queue()   # Queue for limit/stop order

    def background(self):

        # Dequeue until queue is empty.
        while not self.__mark_que.empty():
            msg = self.__mark_que.get()
            self.__handle_market_request(msg)

        while not self.__lmst_que.empty():
            msg = self.__lmst_que.get()
            self.__handle_lmst_request(msg)

    def __on_recv_market_order_request(self, msg):
        self.__mark_que.put(msg)

    def __on_recv_lmst_order_request(self, msg):
        self.__lmst_que.put(msg)

    def __handle_market_request(self, msg):
        pass

    def __handle_lmst_request(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(order_manager)
            order_manager.background()
    except KeyboardInterrupt:
        pass

    order_manager.destroy_node()
    rclpy.shutdown()
