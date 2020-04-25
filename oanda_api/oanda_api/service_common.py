from typing import TypeVar
from typing import Tuple
import rclpy
from rclpy.node import Node
from oandapyV20 import API
from oandapyV20.exceptions import V20Error
from api_msgs.msg import Instrument
from api_msgs.msg import FailReasonCode as frc

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")
EndPoint = TypeVar("EndPoint")

INST_ID_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
}


class ServiceAbs(Node):

    def __init__(self,
                 node_name: str
                 ) -> None:
        super().__init__(node_name)

        PRMNM_ACCESS_TOKEN = "access_token"

        # Set logger lebel
        self._logger = super().get_logger()
        self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare parameter
        self.declare_parameter(PRMNM_ACCESS_TOKEN)
        access_token = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self._logger.debug("[OANDA]Access Token:%s" % access_token)

        self.__api = API(access_token=access_token)

    def _request_api(self,
                     endpoint: EndPoint,
                     rsp: SrvTypeResponse
                     ) -> Tuple[ApiRsp, SrvTypeResponse]:

        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self.__api.request(endpoint)
        except ConnectionError as err:
            self._logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except V20Error as err:
            self._logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR

        return apirsp, rsp
