from typing import TypeVar
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from api_msgs.msg import PriceBucket, Pricing, Instrument
from oandapyV20 import API
from oandapyV20.endpoints import pricing as pr
from oandapyV20.exceptions import V20Error, StreamTerminated
from oanda_api.service_common import INST_DICT

MsgType = TypeVar("MsgType")


class PricingStreamPublisher(Node):

    def __init__(self) -> None:
        super().__init__("pricing_stream")

        # Set logger lebel
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"
        ENA_INST = "enable_instrument."
        PRMNM_ENA_INST_USDJPY = ENA_INST + "usdjpy"
        PRMNM_ENA_INST_EURJPY = ENA_INST + "eurjpy"
        PRMNM_ENA_INST_EURUSD = ENA_INST + "eurusd"

        TPCNM_PRICING_USDJPY = "pricing_usdjpy"
        TPCNM_PRICING_EURJPY = "pricing_eurjpy"
        TPCNM_PRICING_EURUSD = "pricing_eurusd"
        TPCNM_HEARTBEAT = "heart_beat"
        TPCNM_ACT_FLG = "activate_flag"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_ENA_INST_USDJPY)
        self.declare_parameter(PRMNM_ENA_INST_EURJPY)
        self.declare_parameter(PRMNM_ENA_INST_EURUSD)

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        ena_inst_usdjpy = self.get_parameter(PRMNM_ENA_INST_USDJPY).value
        ena_inst_eurjpy = self.get_parameter(PRMNM_ENA_INST_EURJPY).value
        ena_inst_eurusd = self.get_parameter(PRMNM_ENA_INST_EURUSD).value

        logger.debug("[Param]Account Number:[{}]".format(account_number))
        logger.debug("[Param]Access Token:[{}]".format(access_token))
        logger.debug("[Param]Enable instrument:")
        logger.debug("        USD/JPY:[{}]".format(ena_inst_usdjpy))
        logger.debug("        EUR/JPY:[{}]".format(ena_inst_eurjpy))
        logger.debug("        EUR/USD:[{}]".format(ena_inst_eurusd))

        # Declare publisher and subscriber
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        inst_name_list = []
        self.__pub_dict = {}
        if ena_inst_usdjpy:
            inst_name = INST_DICT[Instrument.INST_USD_JPY].name
            pub = self.create_publisher(Pricing,
                                        TPCNM_PRICING_USDJPY,
                                        qos_profile)
            self.__pub_dict[inst_name] = pub.publish
            inst_name_list.append(inst_name)
        if ena_inst_eurjpy:
            inst_name = INST_DICT[Instrument.INST_EUR_JPY].name
            pub = self.create_publisher(Pricing,
                                        TPCNM_PRICING_EURJPY,
                                        qos_profile)
            self.__pub_dict[inst_name] = pub.publish
            inst_name_list.append(inst_name)
        if ena_inst_eurusd:
            inst_name = INST_DICT[Instrument.INST_EUR_USD].name
            pub = self.create_publisher(Pricing,
                                        TPCNM_PRICING_EURUSD,
                                        qos_profile)
            self.__pub_dict[inst_name] = pub.publish
            inst_name_list.append(inst_name)

        self.__pub_hb = self.create_publisher(String,
                                              TPCNM_HEARTBEAT,
                                              qos_profile)

        callback = self.__on_subs_act_flg
        self.__sub_act = self.create_subscription(Bool,
                                                  TPCNM_ACT_FLG,
                                                  callback,
                                                  qos_profile)

        # Initialize
        self.__act_flg = True
        self.__api = API(access_token=access_token)

        instruments = ",".join(inst_name_list)
        params = {"instruments": instruments}
        self.__pi = pr.PricingStream(account_number, params)

        self.__logger = logger

    def background(self) -> None:

        if self.__act_flg:
            try:
                self.__request()
            except V20Error as e:
                self.__logger.error("!!!!!!!!!! V20Error !!!!!!!!!!")
                self.__logger.error("{}".format(e))
            except StreamTerminated as e:
                self.__logger.debug("Stream Terminated: {}".format(e))

    def __on_subs_act_flg(self, msg: MsgType) -> None:
        if msg.data:
            self.__act_flg = True
        else:
            self.__act_flg = False

    def __request(self) -> None:

        for rsp in self.__api.request(self.__pi):

            rclpy.spin_once(self, timeout_sec=0)
            if not self.__act_flg:
                self.__pi.terminate()

            if "type" in rsp.keys():
                typ = rsp["type"]
                if typ == "PRICE":
                    msg = Pricing()
                    msg.time = rsp["time"]
                    for bid in rsp["bids"]:
                        pb = PriceBucket()
                        pb.price = float(bid["price"])
                        pb.liquidity = bid["liquidity"]
                        msg.bids.append(pb)
                    for ask in rsp["asks"]:
                        pb = PriceBucket()
                        pb.price = float(ask["price"])
                        pb.liquidity = ask["liquidity"]
                        msg.asks.append(pb)
                    msg.closeout_bid = float(rsp["closeoutBid"])
                    msg.closeout_ask = float(rsp["closeoutAsk"])
                    msg.tradeable = rsp["tradeable"]
                    # Publish topics
                    self.__pub_dict[rsp["instrument"]](msg)

                elif typ == "HEARTBEAT":
                    msg = String()
                    msg.data = rsp["time"]
                    # Publish topics
                    self.__pub_hb.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    stream_api = PricingStreamPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(stream_api, timeout_sec=1.0)
            stream_api.background()
    except KeyboardInterrupt:
        pass

    stream_api.destroy_node()
    rclpy.shutdown()