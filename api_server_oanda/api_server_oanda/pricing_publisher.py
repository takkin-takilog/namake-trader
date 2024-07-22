from typing import TypeVar
import requests  # type: ignore
import traceback
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from api_server_msgs.msg import PriceBucket, Pricing
from oandapyV20 import API
from oandapyV20.endpoints import pricing as pr
from oandapyV20.exceptions import V20Error, StreamTerminated

# from .constant import ADD_CIPHERS
from .parameter import InstParam
from .dataclass import RosParam
from . import utils as utl
from . import ros_utils as rosutl

MsgType = TypeVar("MsgType")


class PricingPublisher(Node):
    """
    Pricing publisher class.

    """

    def __init__(self) -> None:
        super().__init__("pricing_publisher")

        # --------------- Set logger lebel ---------------
        self.logger = super().get_logger()

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
            param_name = "use_instrument." + i.param_name
            rosprm_use_inst = RosParam(param_name, Parameter.Type.BOOL)
            rosutl.set_parameters(self, rosprm_use_inst)
            use_inst_dict[i.name] = rosprm_use_inst.value

        # --------------- Initialize instance variable ---------------
        self._pub_dict = {}

        # --------------- Create oandapyV20 api ---------------
        if self._rosprm_use_env_live.value:
            access_token = self._rosprm_liv_access_token.value
            account_number = self._rosprm_liv_account_number.value
        else:
            access_token = self._rosprm_pra_access_token.value
            account_number = self._rosprm_pra_account_number.value

        environment = "live" if self._rosprm_use_env_live.value else "practice"

        if self._rosprm_connection_timeout.value <= 0:
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
        self._pub_hb = self.create_publisher(String, "heart_beat", qos_profile)

        # --------------- Initialize oandapyV20 ---------------
        instruments = ",".join(inst_name_list)
        params = {"instruments": instruments}
        self._pi = pr.PricingStream(account_number, params)

    def publish(self) -> None:
        try:
            self._publish()
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
        except BaseException as err:
            self.logger.error("{:!^50}".format(" Unexpected Error "))
            self.logger.error("{}".format(err))
            traceback.print_exc()

    def _publish(self) -> None:
        for rsp in self._api.request(self._pi):
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


def main(args: list[str] | None = None) -> None:
    # requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ADD_CIPHERS

    rclpy.init(args=args)
    pricing_publisher = PricingPublisher()

    try:
        while rclpy.ok():
            pricing_publisher.publish()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
    finally:
        pricing_publisher.destroy_node()
