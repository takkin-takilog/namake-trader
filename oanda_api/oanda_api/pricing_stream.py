from typing import TypeVar
import requests
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from api_msgs.msg import PriceBucket, Pricing
from oandapyV20 import API
from oandapyV20.endpoints import pricing as pr
from oandapyV20.exceptions import V20Error, StreamTerminated
from .constant import ADD_CIPHERS
from .parameter import InstParam
from .dataclass import RosParam
from . import utils as utl
from . import ros_utils as rosutl

MsgType = TypeVar("MsgType")


class PricingStreamPublisher(Node):
    """
    Pricing stream publisher class.

    """

    def __init__(self) -> None:
        super().__init__("pricing_stream")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # --------------- Define Constant value ---------------
        USE_INST = "use_instrument."
        TPCNM_HEARTBEAT = "heart_beat"
        TPCNM_ACT_FLG = "activate_flag"

        # --------------- Initialize ROS parameter ---------------
        self._rosprm_use_env_live = RosParam("use_env_live", Parameter.Type.BOOL)
        self._rosprm_pra_account_number = RosParam(
            "env_practice.account_number", Parameter.Type.STRING
        )
        self._rosprm_pra_access_token = RosParam(
            "env_practice.access_token", Parameter.Type.STRING
        )
        self._rosprm_liv_account_number = RosParam(
            "env_live.account_number", Parameter.Type.STRING
        )
        self._rosprm_liv_access_token = RosParam(
            "env_live.access_token", Parameter.Type.STRING
        )
        self._rosprm_connection_timeout = RosParam(
            "connection_timeout", Parameter.Type.INTEGER
        )

        rosutl.set_parameters(self, self._rosprm_use_env_live)
        rosutl.set_parameters(self, self._rosprm_pra_account_number)
        rosutl.set_parameters(self, self._rosprm_pra_access_token)
        rosutl.set_parameters(self, self._rosprm_liv_account_number)
        rosutl.set_parameters(self, self._rosprm_liv_access_token)
        rosutl.set_parameters(self, self._rosprm_connection_timeout)

        use_inst_dict: dict[str, int] = {}
        for i in InstParam:
            param_name = USE_INST + i.param_name
            rosprm_use_inst = RosParam(param_name, Parameter.Type.BOOL)
            rosutl.set_parameters(self, rosprm_use_inst)
            use_inst_dict[i.name] = rosprm_use_inst.value  # type: ignore[assignment]

        # --------------- Initialize instance variable ---------------
        self._pub_dict = {}
        self._act_flg = True

        # --------------- Create oandapyV20 api ---------------
        if self._rosprm_use_env_live.value:
            access_token = self._rosprm_liv_access_token.value
            account_number = self._rosprm_liv_account_number.value
        else:
            access_token = self._rosprm_pra_access_token.value
            account_number = self._rosprm_pra_account_number.value

        environment = "live" if self._rosprm_use_env_live.value else "practice"

        if self._rosprm_connection_timeout.value <= 0:  # type: ignore[operator]
            request_params = None
            self.logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": self._rosprm_connection_timeout.value}

        self._api = API(
            access_token=access_token,
            environment=environment,
            request_params=request_params,
        )

        # --------------- Create ROS Communication ---------------
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create topic publisher "pricing_*****"
        inst_name_list = []
        for i in InstParam:
            if use_inst_dict[i.name]:
                pub = self.create_publisher(Pricing, i.topic_name, qos_profile)
                self._pub_dict[i.name] = pub.publish
                inst_name_list.append(i.name)

        # Create topic publisher "HeartBeat"
        self._pub_hb = self.create_publisher(String, TPCNM_HEARTBEAT, qos_profile)

        # Create topic subscriber "ActivateFlag"
        self._sub_act = self.create_subscription(
            Bool, TPCNM_ACT_FLG, self._on_subs_act_flg, qos_profile
        )

        # --------------- Initialize oandapyV20 ---------------
        instruments = ",".join(inst_name_list)
        params = {"instruments": instruments}
        self._pi = pr.PricingStream(account_number, params)

    def background(self) -> None:

        if self._act_flg:
            try:
                self._request()
            except StreamTerminated as err:
                self.logger.debug("Stream Terminated: {}".format(err))
            except V20Error as err:
                self.logger.error("{:!^50}".format(" Oanda-V20 Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
            except requests.exceptions.ConnectionError as err:
                self.logger.error("{:!^50}".format(" HTTP-Connection Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
            except requests.exceptions.Timeout as err:
                self.logger.error("{:!^50}".format(" HTTP-Timeout Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
            except requests.exceptions.RequestException as err:
                self.logger.error("{:!^50}".format(" HTTP-Request Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()
            except KeyboardInterrupt as err:
                raise err
            except BaseException as err:  # pylint: disable=W0703
                self.logger.error("{:!^50}".format(" Unexpected Error "))
                self.logger.error("{}".format(err))
                traceback.print_exc()

    def _on_subs_act_flg(self, msg: MsgType) -> None:

        if msg.data:  # type: ignore[attr-defined]
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
                    # ---------- Publish topics ----------
                    self._pub_dict[rsp["instrument"]](msg)

                elif typ == "HEARTBEAT":
                    msg = String()
                    msg.data = utl.convert_datetime_jst(rsp["time"])
                    # ---------- Publish topics ----------
                    self._pub_hb.publish(msg)


def main(args=None):

    # pylint: disable=E1101
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
