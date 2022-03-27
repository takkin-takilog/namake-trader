from typing import TypeVar
import requests
from requests.exceptions import ConnectionError, ReadTimeout
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from api_msgs.msg import PriceBucket, Pricing
from oandapyV20 import API
from oandapyV20.endpoints import pricing as pr
from oandapyV20.exceptions import V20Error, StreamTerminated
from .constant import ADD_CIPHERS
from .parameter import InstParam
from . import utility as utl

MsgType = TypeVar("MsgType")


class PricingStreamPublisher(Node):

    def __init__(self) -> None:
        super().__init__("pricing_stream")

        # --------------- Set logger lebel ---------------
        self._logger = super().get_logger()
        self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # --------------- Define Constant value ---------------
        PRMNM_USE_ENV_LIVE = "use_env_live"
        ENV_PRAC = "env_practice."
        PRMNM_PRAC_ACCOUNT_NUMBER = ENV_PRAC + "account_number"
        PRMNM_PRAC_ACCESS_TOKEN = ENV_PRAC + "access_token"
        ENV_LIVE = "env_live."
        PRMNM_LIVE_ACCOUNT_NUMBER = ENV_LIVE + "account_number"
        PRMNM_LIVE_ACCESS_TOKEN = ENV_LIVE + "access_token"
        ENA_INST = "enable_instrument."
        PRMNM_CONN_TIMEOUT = "connection_timeout"
        TPCNM_HEARTBEAT = "heart_beat"
        TPCNM_ACT_FLG = "activate_flag"

        # --------------- Declare ROS parameter ---------------
        self.declare_parameter(PRMNM_USE_ENV_LIVE)
        self.declare_parameter(PRMNM_PRAC_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_PRAC_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_LIVE_ACCOUNT_NUMBER)
        self.declare_parameter(PRMNM_LIVE_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_CONN_TIMEOUT)
        enable_inst_dict = {}
        for i in InstParam:
            param_name = ENA_INST + i.param_name
            self.declare_parameter(param_name)
            enable_inst = self.get_parameter(param_name).value
            enable_inst_dict[i.name] = enable_inst

        USE_ENV_LIVE = self.get_parameter(PRMNM_USE_ENV_LIVE).value
        if USE_ENV_LIVE:
            ACCOUNT_NUMBER = self.get_parameter(PRMNM_LIVE_ACCOUNT_NUMBER).value
            ACCESS_TOKEN = self.get_parameter(PRMNM_LIVE_ACCESS_TOKEN).value
        else:
            ACCOUNT_NUMBER = self.get_parameter(PRMNM_PRAC_ACCOUNT_NUMBER).value
            ACCESS_TOKEN = self.get_parameter(PRMNM_PRAC_ACCESS_TOKEN).value
        CONN_TIMEOUT = self.get_parameter(PRMNM_CONN_TIMEOUT).value

        self._logger.debug("[Param]Use Env Live:[{}]".format(USE_ENV_LIVE))
        self._logger.debug("[Param]Account Number:[{}]".format(ACCOUNT_NUMBER))
        self._logger.debug("[Param]Access Token:[{}]".format(ACCESS_TOKEN))
        self._logger.debug("[Param]Connection Timeout:[{}]".format(CONN_TIMEOUT))
        self._logger.debug("[Param]Enable instrument:")
        for i in InstParam:
            enable_inst = enable_inst_dict[i.name]
            self._logger.debug("  - {}:[{}]".format(i.name.replace("_", "/"),
                                                    enable_inst))

        # --------------- Initialize ROS topic ---------------
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        inst_name_list = []
        self._pub_dict = {}

        # Create topic publisher "pricing_*****"
        for i in InstParam:
            enable_inst = enable_inst_dict[i.name]
            if enable_inst:
                pub = self.create_publisher(Pricing,
                                            i.topic_name,
                                            qos_profile)
                self._pub_dict[i.name] = pub.publish
                inst_name_list.append(i.name)

        # Create topic publisher "HeartBeat"
        self._pub_hb = self.create_publisher(String,
                                             TPCNM_HEARTBEAT,
                                             qos_profile)

        # Create topic subscriber "ActivateFlag"
        callback = self._on_subs_act_flg
        self._sub_act = self.create_subscription(Bool,
                                                 TPCNM_ACT_FLG,
                                                 callback,
                                                 qos_profile)

        # --------------- Initialize variable ---------------
        self._act_flg = True

        if CONN_TIMEOUT <= 0:
            request_params = None
            self._logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": CONN_TIMEOUT}

        environment = "live" if USE_ENV_LIVE else "practice"
        self._api = API(access_token=ACCESS_TOKEN,
                        environment=environment,
                        request_params=request_params)

        instruments = ",".join(inst_name_list)
        params = {"instruments": instruments}
        self._pi = pr.PricingStream(ACCOUNT_NUMBER, params)

    def background(self) -> None:

        if self._act_flg:
            try:
                self._request()
            except StreamTerminated as err:
                self._logger.debug("Stream Terminated: {}".format(err))
            except V20Error as err:
                self._logger.error("{:!^50}".format(" V20Error "))
                self._logger.error("{}".format(err))
            except ConnectionError as err:
                self._logger.error("{:!^50}".format(" ConnectionError "))
                self._logger.error("{}".format(err))
            except ReadTimeout as err:
                self._logger.error("{:!^50}".format(" ReadTimeout "))
                self._logger.error("{}".format(err))
            except Exception as err:
                self._logger.error("{:!^50}".format(" OthersError "))
                self._logger.error("{}".format(err))

    def _on_subs_act_flg(self, msg: MsgType) -> None:
        if msg.data:
            self._act_flg = True
        else:
            self._act_flg = False

    def _request(self) -> None:

        for rsp in self._api.request(self._pi):

            rclpy.spin_once(self, timeout_sec=0)
            if not self._act_flg:
                self._pi.terminate()

            if "type" in rsp.keys():
                typ = rsp["type"]
                if typ == "PRICE":
                    msg = Pricing()
                    msg.time = utl.convert_datetime_jst(rsp["time"])
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
                    self._pub_dict[rsp["instrument"]](msg)

                elif typ == "HEARTBEAT":
                    msg = String()
                    msg.data = utl.convert_datetime_jst(rsp["time"])
                    # Publish topics
                    self._pub_hb.publish(msg)


def main(args=None):

    requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ADD_CIPHERS

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
