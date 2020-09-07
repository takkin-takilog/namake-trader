from typing import TypeVar
from typing import Tuple
import rclpy
from rclpy.node import Node
from oandapyV20 import API
from oandapyV20.exceptions import V20Error
from api_msgs.msg import Instrument
from api_msgs.msg import Granularity
from api_msgs.msg import FailReasonCode as frc

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")
ApiRsp = TypeVar("ApiRsp")
EndPoint = TypeVar("EndPoint")

"""
INST_ID_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
}
"""

MIN_UNIT_DICT = {
    Instrument.INST_USD_JPY: "0.001",
    Instrument.INST_EUR_JPY: "0.001",
    Instrument.INST_EUR_USD: "0.00001",
}

GRAN_ID_DICT = {
    Granularity.GRAN_M1: "M1",  # 1 minute
    Granularity.GRAN_M2: "M2",  # 2 minutes
    Granularity.GRAN_M3: "M3",  # 3 minutes
    Granularity.GRAN_M4: "M4",  # 4 minutes
    Granularity.GRAN_M5: "M5",  # 5 minutes
    Granularity.GRAN_M10: "M10",  # 10 minutes
    Granularity.GRAN_M15: "M15",  # 15 minutes
    Granularity.GRAN_M30: "M30",  # 30 minutes
    Granularity.GRAN_H1: "H1",  # 1 hour
    Granularity.GRAN_H2: "H2",  # 2 hours
    Granularity.GRAN_H3: "H3",  # 3 hours
    Granularity.GRAN_H4: "H4",  # 4 hours
    Granularity.GRAN_H6: "H6",  # 6 hours
    Granularity.GRAN_H8: "H8",  # 8 hours
    Granularity.GRAN_H12: "H12",  # 12 hours
    Granularity.GRAN_D: "D",  # 1 Day
    Granularity.GRAN_W: "W",  # 1 Week
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
        self._logger.debug("[Param]Access Token:[%s]" % (access_token))

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
            self._logger.error("!!!!!!!!!! ConnectionError !!!!!!!!!!")
            self._logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except V20Error as err:
            self._logger.error("!!!!!!!!!! V20Error !!!!!!!!!!")
            self._logger.error("%s" % err)
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR

        return apirsp, rsp
