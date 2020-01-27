import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from oanda_api_msgs.msg import Pricing
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
        PRMNM_INSTRUMENTS = "instruments"
        TPCNM_PRICING = "pricing_"
        TPCNM_ACT_FLG = "activate_flag"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_INSTRUMENTS)

        # initialize
        self.__act_flg = True

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        instrumentslist = self.get_parameter(PRMNM_INSTRUMENTS).value
        self.__logger.debug("[OANDA]Account Number:%s" % account_number)
        self.__logger.debug("[OANDA]Access Token:%s" % access_token)
        self.__logger.debug("[OANDA]Instruments:%s" % instrumentslist)

        self.__api = API(access_token=access_token)
        instruments = ",".join(instrumentslist)
        params = {"instruments": instruments}
        self.__ps = PricingStream(account_number, params)

        # Declare publisher and subscriber
        self.__pub_dict = {}
        for instrument in instrumentslist:
            suffix = instrument.replace("_", "").lower()
            pub = self.create_publisher(Pricing, TPCNM_PRICING + suffix)
            self.__pub_dict[instrument] = pub.publish

        self.__sub_act = self.create_subscription(Bool, TPCNM_ACT_FLG,
                                                  self.__act_flg_callback)

    def background(self):
        if self.__act_flg:
            self.__request()

    def __act_flg_callback(self, msg):
        if msg.data:
            self.__act_flg = True
        else:
            self.__act_flg = False

    def __request(self):
        TIME = "time"
        INSTRUMENT = "instrument"
        BID = "closeoutBid"
        ASK = "closeoutAsk"
        TRADEABLE = "tradeable"

        try:
            for rsp in self.__api.request(self.__ps):

                rclpy.spin_once(self, timeout_sec=0)
                if not self.__act_flg:
                    self.__logger.debug("terminate PricingStream")
                    self.__ps.terminate()

                if TRADEABLE in rsp.keys():
                    msg = Pricing()
                    msg.time = rsp[TIME]
                    msg.instrument = rsp[INSTRUMENT]
                    msg.closeout_bid = float(rsp[BID])
                    msg.closeout_ask = float(rsp[ASK])
                    msg.tradeable = rsp[TRADEABLE]

                    # Publish topics
                    self.__pub_dict[msg.instrument](msg)

        except V20Error as e:
            print("Error: {}".format(e))
        except StreamTerminated as e:
            print("StreamTerminated: {}".format(e))

    def __handler(self, func, msg):
        func(msg)


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
