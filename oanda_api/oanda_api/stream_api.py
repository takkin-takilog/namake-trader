import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from oandapyV20 import API
from oandapyV20.endpoints.pricing import PricingStream
from oandapyV20.exceptions import V20Error


class StreamApi(Node):

    def __init__(self):
        super().__init__("stream_api")

        # Set logger lebel
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"
        TPCNM_BIDS_PRICE = "bids_price"
        TPCNM_ASKS_PRICE = "asks_price"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self.get_logger().debug("[OANDA]Account Number:%s" % account_number)
        self.get_logger().debug("[OANDA]Access Token:%s" % access_token)

        self.__api = API(access_token=access_token)
        params = {"instruments": "USD_JPY"}
        self.__ps = PricingStream(account_number, params)

        # Declare publisher
        self.__pub_bids = self.create_publisher(Float32, TPCNM_BIDS_PRICE)
        self.__pub_asks = self.create_publisher(Float32, TPCNM_ASKS_PRICE)

    def request(self):
        BIDS = "bids"
        ASKS = "asks"
        PRICE = "price"
        bids = Float32()
        asks = Float32()
        try:
            for rsp in self.__api.request(self.__ps):
                if (BIDS and ASKS) in rsp.keys():
                    bids.data = float(rsp[BIDS][0][PRICE])
                    asks.data = float(rsp[ASKS][0][PRICE])
                    # Publish topics
                    self.__pub_bids.publish(bids)
                    self.__pub_asks.publish(asks)
                    # output log
                    self.get_logger().info(str(bids.data))
                    self.get_logger().info(str(asks.data))
        except V20Error as e:
            print("Error: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    stream_api = StreamApi()
    stream_api.request()
    rclpy.shutdown()
