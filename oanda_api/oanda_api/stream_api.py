import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from oandapyV20 import API
from oandapyV20.endpoints.pricing import PricingStream
from oandapyV20.exceptions import V20Error, StreamTerminated


class StreamApi(Node):

    def __init__(self):
        super().__init__("stream_api")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"
        TPCNM_BIDS_PRICE = "bids_price"
        TPCNM_ASKS_PRICE = "asks_price"
        TPCNM_ACT_FLG = "activate_flag"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)

        # initialize
        self.__act_flg = True

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self.__logger.debug("[OANDA]Account Number:%s" % account_number)
        self.__logger.debug("[OANDA]Access Token:%s" % access_token)

        self.__api = API(access_token=access_token)
        params = {"instruments": "USD_JPY"}
        self.__ps = PricingStream(account_number, params)

        # Declare publisher and subscriber
        self.__pub_bids = self.create_publisher(Float32, TPCNM_BIDS_PRICE)
        self.__pub_asks = self.create_publisher(Float32, TPCNM_ASKS_PRICE)
        self.__sub_act = self.create_subscription(Bool, TPCNM_ACT_FLG,
                                                  self.__act_flg_callback)

    def background(self):
        self.__logger.debug("background")
        if self.__act_flg:
            self.__request()

    def __act_flg_callback(self, msg):
        self.__logger.debug("Called callbak func: %s" % msg.data)
        if msg.data:
            self.__act_flg = True
        else:
            self.__act_flg = False

    def __request(self):
        BIDS = "bids"
        ASKS = "asks"
        PRICE = "price"
        bids = Float32()
        asks = Float32()
        try:
            for rsp in self.__api.request(self.__ps):

                rclpy.spin_once(self, timeout_sec=0)
                if not self.__act_flg:
                    self.__logger.debug("terminate PricingStream")
                    self.__ps.terminate()

                if (BIDS and ASKS) in rsp.keys():
                    bids.data = float(rsp[BIDS][0][PRICE])
                    asks.data = float(rsp[ASKS][0][PRICE])
                    # Publish topics
                    self.__pub_bids.publish(bids)
                    self.__pub_asks.publish(asks)
                    # output log
                    self.__logger.info(str(bids.data))
                    self.__logger.info(str(asks.data))

        except V20Error as e:
            print("Error: {}".format(e))
        except StreamTerminated as e:
            print("StreamTerminated: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    stream_api = StreamApi()
    try:
        while True:
            rclpy.spin_once(stream_api, timeout_sec=1.0)
            stream_api.background()
    except KeyboardInterrupt:
        stream_api.destroy_node()
        rclpy.shutdown()
