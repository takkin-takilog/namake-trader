from typing import TypeVar
from typing import Tuple
import datetime as dt
from requests.exceptions import ConnectionError, ReadTimeout
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

ADD_CIPHERS = "HIGH:!DH"


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


class BaseService(Node):

    def __init__(self,
                 node_name: str
                 ) -> None:
        super().__init__(node_name)

        PRMNM_USE_ENV_LIVE = "use_env_live"
        ENV_PRAC = "env_practice."
        PRMNM_PRAC_ACCESS_TOKEN = ENV_PRAC + "access_token"
        ENV_LIVE = "env_live."
        PRMNM_LIVE_ACCESS_TOKEN = ENV_LIVE + "access_token"
        PRMNM_CONN_TIMEOUT = "connection_timeout"

        # Set logger lebel
        self._logger = super().get_logger()
        self._logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare parameter
        self.declare_parameter(PRMNM_USE_ENV_LIVE)
        self.declare_parameter(PRMNM_PRAC_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_LIVE_ACCESS_TOKEN)
        self.declare_parameter(PRMNM_CONN_TIMEOUT)

        USE_ENV_LIVE = self.get_parameter(PRMNM_USE_ENV_LIVE).value
        if USE_ENV_LIVE:
            ACCESS_TOKEN = self.get_parameter(PRMNM_LIVE_ACCESS_TOKEN).value
        else:
            ACCESS_TOKEN = self.get_parameter(PRMNM_PRAC_ACCESS_TOKEN).value
        CONN_TIMEOUT = self.get_parameter(PRMNM_CONN_TIMEOUT).value

        self._logger.debug("[Param]Use Env Live:[{}]".format(USE_ENV_LIVE))
        self._logger.debug("[Param]Access Token:[{}]".format(ACCESS_TOKEN))
        self._logger.debug("[Param]Connection Timeout:[{}]".format(CONN_TIMEOUT))

        if USE_ENV_LIVE:
            environment = "live"
        else:
            environment = "practice"

        if CONN_TIMEOUT <= 0:
            request_params = None
            self._logger.debug("Not set Timeout")
        else:
            request_params = {"timeout": CONN_TIMEOUT}

        self._api = API(access_token=ACCESS_TOKEN,
                        environment=environment,
                        request_params=request_params)

    def _request_api(self,
                     endpoint: EndPoint,
                     rsp: SrvTypeResponse
                     ) -> Tuple[ApiRsp, SrvTypeResponse]:

        rsp.frc_msg.reason_code = frc.REASON_UNSET
        apirsp = None
        try:
            apirsp = self._api.request(endpoint)
        except V20Error as err:
            self._logger.error("{:!^50}".format(" V20Error "))
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OANDA_V20_ERROR
        except ConnectionError as err:
            self._logger.error("{:!^50}".format(" ConnectionError "))
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except ReadTimeout as err:
            self._logger.error("{:!^50}".format(" ReadTimeout "))
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_CONNECTION_ERROR
        except Exception as err:
            self._logger.error("{:!^50}".format(" OthersError "))
            self._logger.error("{}".format(err))
            rsp.frc_msg.reason_code = frc.REASON_OTHERS

        return apirsp, rsp
