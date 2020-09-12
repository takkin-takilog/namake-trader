from typing import TypeVar
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from api_msgs.msg import PriceBucket, Pricing, Instrument
from oandapyV20 import API
from oandapyV20.endpoints.pricing import PricingStream
from oandapyV20.exceptions import V20Error, StreamTerminated
from oanda_api.service_common import INST_DICT

MsgType = TypeVar("MsgType")


class PricingStreamer(Node):

    def __init__(self) -> None:
        super().__init__("pricing_streamer")

        # Set logger lebel
        logger = super().get_logger()
        logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"
        ENA_INST = "enable_instrument."
        PRMNM_ENA_INST_USDJPY = ENA_INST + "usdjpy"
        PRMNM_ENA_INST_EURJPY = ENA_INST + "eurjpy"
        PRMNM_ENA_INST_EURUSD = ENA_INST + "eurusd"

        TPCNM_PRICING = "pricing_"
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

        # Initialize
        self.__act_flg = True

        inst_name_list = []
        if ena_inst_usdjpy:
            inst_name = INST_DICT[Instrument.INST_USD_JPY].name
            inst_name_list.append(inst_name)
        if ena_inst_eurjpy:
            inst_name = INST_DICT[Instrument.INST_EUR_JPY].name
            inst_name_list.append(inst_name)
        if ena_inst_eurusd:
            inst_name = INST_DICT[Instrument.INST_EUR_USD].name
            inst_name_list.append(inst_name)

        self.__api = API(access_token=access_token)

        instruments = ",".join(inst_name_list)
        params = {"instruments": instruments}
        self.__ps = PricingStream(account_number, params)

        # Declare publisher and subscriber
        self.__pub_dict = {}
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        for inst_name in inst_name_list:
            suffix = inst_name.replace("_", "").lower()
            pub = self.create_publisher(Pricing,
                                        TPCNM_PRICING + suffix,
                                        qos_profile)
            self.__pub_dict[inst_name] = pub.publish
        self.__pub_hb = self.create_publisher(String,
                                              TPCNM_HEARTBEAT,
                                              qos_profile)
        callback = self.__on_subs_act_flg
        self.__sub_act = self.create_subscription(Bool,
                                                  TPCNM_ACT_FLG,
                                                  callback,
                                                  qos_profile)

        self.__logger = logger

    def background(self) -> None:
        if self.__act_flg:
            self.__request()

    def __on_subs_act_flg(self, msg: MsgType) -> None:
        if msg.data:
            self.__act_flg = True
        else:
            self.__act_flg = False

    def __request(self) -> None:
        TYPE = "type"
        TYP_PRICE = "PRICE"
        TYP_HB = "HEARTBEAT"
        TIME = "time"
        INSTRUMENT = "instrument"
        BIDS = "bids"
        ASKS = "asks"
        PRICE = "price"
        LIQUIDITY = "liquidity"
        CLOSEOUT_BID = "closeoutBid"
        CLOSEOUT_ASK = "closeoutAsk"
        TRADEABLE = "tradeable"

        try:
            for rsp in self.__api.request(self.__ps):

                rclpy.spin_once(self, timeout_sec=0)
                if not self.__act_flg:
                    self.__ps.terminate()

                if TYPE in rsp.keys():
                    typ = rsp[TYPE]
                    if typ == TYP_PRICE:
                        msg = Pricing()
                        msg.time = rsp[TIME]
                        for bid in rsp[BIDS]:
                            pb = PriceBucket()
                            pb.price = float(bid[PRICE])
                            pb.liquidity = bid[LIQUIDITY]
                            msg.bids.append(pb)
                        for ask in rsp[ASKS]:
                            pb = PriceBucket()
                            pb.price = float(ask[PRICE])
                            pb.liquidity = ask[LIQUIDITY]
                            msg.asks.append(pb)
                        msg.closeout_bid = float(rsp[CLOSEOUT_BID])
                        msg.closeout_ask = float(rsp[CLOSEOUT_ASK])
                        msg.tradeable = rsp[TRADEABLE]
                        # Publish topics
                        self.__pub_dict[rsp[INSTRUMENT]](msg)

                    elif typ == TYP_HB:
                        msg = String()
                        msg.data = rsp[TIME]
                        # Publish topics
                        self.__pub_hb.publish(msg)

        except V20Error as e:
            self.__logger.error("!!!!!!!!!! V20Error !!!!!!!!!!")
            self.__logger.error("{}".format(e))
        except StreamTerminated as e:
            self.__logger.debug("Stream Terminated: {}".format(e))


def main(args=None):
    rclpy.init(args=args)
    stream_api = PricingStreamer()

    try:
        while rclpy.ok():
            rclpy.spin_once(stream_api, timeout_sec=1.0)
            stream_api.background()
    except KeyboardInterrupt:
        pass

    stream_api.destroy_node()
    rclpy.shutdown()
