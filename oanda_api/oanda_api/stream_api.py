import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        TPCNM_PRICE = "price"

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

        # String型のchatterトピックを送信するpublisherの定義
        self.publisher = self.create_publisher(String, TPCNM_PRICE)

    def request(self):
        msg = String()
        try:
            for rsp in self.__api.request(self.__ps):
                if "bids" in rsp.keys():
                    msg.data = rsp["bids"][0]["price"]
                    # chatterトピックにmsgを送信
                    self.publisher.publish(msg)
                    # msgの中身を標準出力にログ
                    self.get_logger().info(msg.data)
        except V20Error as e:
            print("Error: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    stream_api = StreamApi()
    stream_api.request()
    rclpy.shutdown()
