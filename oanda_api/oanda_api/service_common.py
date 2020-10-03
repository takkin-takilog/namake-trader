from abc import ABCMeta
from typing import TypeVar
from typing import Tuple
import datetime as dt
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


class InstInfo():

    def __init__(self,
                 name: str,
                 min_unit: str
                 ) -> None:
        self.__name = name
        self.__min_unit = min_unit

    @property
    def name(self) -> str:
        return self.__name

    @property
    def min_unit(self) -> str:
        return self.__min_unit


class GranInfo():

    def __init__(self,
                 name: str,
                 timedelta: dt.timedelta
                 ) -> None:
        self.__name = name
        self.__timedelta = timedelta

    @property
    def name(self) -> str:
        return self.__name

    @property
    def timedelta(self) -> dt.timedelta:
        return self.__timedelta


INST_DICT = {
    Instrument.INST_USD_JPY: InstInfo("USD_JPY", "0.001"),
    Instrument.INST_EUR_JPY: InstInfo("EUR_JPY", "0.001"),
    Instrument.INST_EUR_USD: InstInfo("EUR_USD", "0.00001")
}

GRAN_DICT = {
    Granularity.GRAN_M1: GranInfo("M1", dt.timedelta(minutes=1)),
    Granularity.GRAN_M2: GranInfo("M2", dt.timedelta(minutes=2)),
    Granularity.GRAN_M3: GranInfo("M3", dt.timedelta(minutes=3)),
    Granularity.GRAN_M4: GranInfo("M4", dt.timedelta(minutes=4)),
    Granularity.GRAN_M5: GranInfo("M5", dt.timedelta(minutes=5)),
    Granularity.GRAN_M10: GranInfo("M10", dt.timedelta(minutes=10)),
    Granularity.GRAN_M15: GranInfo("M15", dt.timedelta(minutes=15)),
    Granularity.GRAN_M30: GranInfo("M30", dt.timedelta(minutes=30)),
    Granularity.GRAN_H1: GranInfo("H1", dt.timedelta(hours=1)),
    Granularity.GRAN_H2: GranInfo("H2", dt.timedelta(hours=2)),
    Granularity.GRAN_H3: GranInfo("H3", dt.timedelta(hours=3)),
    Granularity.GRAN_H4: GranInfo("H4", dt.timedelta(hours=4)),
    Granularity.GRAN_H6: GranInfo("H6", dt.timedelta(hours=6)),
    Granularity.GRAN_H8: GranInfo("H8", dt.timedelta(hours=8)),
    Granularity.GRAN_H12: GranInfo("H12", dt.timedelta(hours=12)),
    Granularity.GRAN_D: GranInfo("D", dt.timedelta(days=1)),
    Granularity.GRAN_W: GranInfo("W", dt.timedelta(weeks=1))
}


class AbstractService(Node, metaclass=ABCMeta):

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
        ACCESS_TOKEN = self.get_parameter(PRMNM_ACCESS_TOKEN).value
        self._logger.debug("[Param]Access Token:[{}]".format(ACCESS_TOKEN))

        self._api = API(access_token=ACCESS_TOKEN)

    def _request_api(self,
                     endpoint: EndPoint,
                     rsp: SrvTypeResponse
                     ) -> Tuple[ApiRsp, SrvTypeResponse]:

        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self._api.request(endpoint)
        except ConnectionError as err:
            self._logger.error("!!!!!!!!!! ConnectionError !!!!!!!!!!")
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except V20Error as err:
            self._logger.error("!!!!!!!!!! V20Error !!!!!!!!!!")
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR

        return apirsp, rsp
