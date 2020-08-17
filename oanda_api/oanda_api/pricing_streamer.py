from typing import TypeVar
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from api_msgs.msg import PriceBucket, Pricing
from oandapyV20 import API
from oandapyV20.endpoints.pricing import PricingStream
from oandapyV20.exceptions import V20Error, StreamTerminated
from oanda_api.service_common import INST_ID_DICT

MsgType = TypeVar("MsgType")


class PricingStreamer(Node):

    def __init__(self) -> None:
        super().__init__("pricing_streamer")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        PRMNM_ACCOUNT_NUMBER = "account_number"
        PRMNM_ACCESS_TOKEN = "access_token"
        PRMNM_INSTRUMENT_ID = "instrument_id"
        TPCNM_PRICING = "pricing_"
        TPCNM_HEARTBEAT = "heart_beat"
        TPCNM_ACT_FLG = "activate_flag"

        # Declare parameter
        self.declare_parameter(PRMNM_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_INSTRUMENT_ID)

        # Initialize
        self.__act_flg = True

        account_number = self.get_parameter(PRMNM_ACCOUNT_NUMBER).value
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        inst_id_list = self.get_parameter(PRMNM_INSTRUMENT_ID).value
        self.__logger.debug("[Param]Account Number:[%s]" % account_number)
        self.__logger.debug("[Param]Access Token:[%s]" % access_token)
        self.__logger.debug("[Param]Instruments:[%s]" % inst_id_list)

        inst_name_list = []
        for inst_id in inst_id_list:
            inst_name = INST_ID_DICT[inst_id]
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
        self.__sub_act = self.create_subscription(Bool,
                                                  TPCNM_ACT_FLG,
                                                  self.__on_recv_act_flg,
                                                  qos_profile)

    def background(self) -> None:
        if self.__act_flg:
            self.__request()

    def __on_recv_act_flg(self, msg: MsgType) -> None:
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
            self.__logger.error("%s" % e)
        except StreamTerminated as e:
            self.__logger.debug("Stream Terminated: %s" % e)


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
