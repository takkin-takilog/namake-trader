from typing import TypeVar
from dataclasses import dataclass
import datetime as dt
import pandas as pd
import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from trade_manager.utility import RosParam
from trade_manager.constant import FMT_YMDHMS
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.srv import HistoricalCandlesSrv
from trade_manager_msgs.msg import Candle
from trade_manager_msgs.msg import CandleMnt
from trade_manager_msgs.msg import Instrument as InstTm
from trade_manager_msgs.msg import Granularity as GranTm

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


@dataclass
class _RosParams():
    """
    ROS Parameter.
    """
    ENA_INST_USDJPY = RosParam("enable_instrument.usdjpy")
    ENA_INST_EURJPY = RosParam("enable_instrument.eurjpy")
    ENA_INST_EURUSD = RosParam("enable_instrument.eurusd")
    ENA_GRAN_M1 = RosParam("enable_granularity.m1")
    ENA_GRAN_M2 = RosParam("enable_granularity.m2")
    ENA_GRAN_M3 = RosParam("enable_granularity.m3")
    ENA_GRAN_M4 = RosParam("enable_granularity.m4")
    ENA_GRAN_M5 = RosParam("enable_granularity.m5")
    ENA_GRAN_M10 = RosParam("enable_granularity.m10")
    ENA_GRAN_M15 = RosParam("enable_granularity.m15")
    ENA_GRAN_M30 = RosParam("enable_granularity.m30")
    ENA_GRAN_H1 = RosParam("enable_granularity.h1")
    ENA_GRAN_H2 = RosParam("enable_granularity.h2")
    ENA_GRAN_H3 = RosParam("enable_granularity.h3")
    ENA_GRAN_H4 = RosParam("enable_granularity.h4")
    ENA_GRAN_H6 = RosParam("enable_granularity.h6")
    ENA_GRAN_H8 = RosParam("enable_granularity.h8")
    ENA_GRAN_H12 = RosParam("enable_granularity.h12")
    ENA_GRAN_D = RosParam("enable_granularity.d")
    ENA_GRAN_W = RosParam("enable_granularity.w")
    LENG_M1 = RosParam("data_length.m1")
    LENG_M2 = RosParam("data_length.m2")
    LENG_M3 = RosParam("data_length.m3")
    LENG_M4 = RosParam("data_length.m4")
    LENG_M5 = RosParam("data_length.m5")
    LENG_M10 = RosParam("data_length.m10")
    LENG_M15 = RosParam("data_length.m15")
    LENG_M30 = RosParam("data_length.m30")
    LENG_H1 = RosParam("data_length.h1")
    LENG_H2 = RosParam("data_length.h2")
    LENG_H3 = RosParam("data_length.h3")
    LENG_H4 = RosParam("data_length.h4")
    LENG_H6 = RosParam("data_length.h6")
    LENG_H8 = RosParam("data_length.h8")
    LENG_H12 = RosParam("data_length.h12")
    LENG_D = RosParam("data_length.d")
    LENG_W = RosParam("data_length.w")


class CandlesData():

    cli_ordcre = None

    logger = None


class HistoricalCandles(Node):

    def __init__(self) -> None:
        super().__init__("historical_candles")

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        CandlesData.logger = self.logger

        # Define Constant value.

        # Declare ROS parameter
        self._rosprm = _RosParams()
        self.declare_parameter(self._rosprm.ENA_INST_USDJPY.name)
        self.declare_parameter(self._rosprm.ENA_INST_EURJPY.name)
        self.declare_parameter(self._rosprm.ENA_INST_EURUSD.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M1.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M2.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M3.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M4.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M5.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M10.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M15.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_M30.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H1.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H2.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H3.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H4.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H6.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H8.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_H12.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_D.name)
        self.declare_parameter(self._rosprm.ENA_GRAN_W.name)
        self.declare_parameter(self._rosprm.LENG_M1.name)
        self.declare_parameter(self._rosprm.LENG_M2.name)
        self.declare_parameter(self._rosprm.LENG_M3.name)
        self.declare_parameter(self._rosprm.LENG_M4.name)
        self.declare_parameter(self._rosprm.LENG_M5.name)
        self.declare_parameter(self._rosprm.LENG_M10.name)
        self.declare_parameter(self._rosprm.LENG_M15.name)
        self.declare_parameter(self._rosprm.LENG_M30.name)
        self.declare_parameter(self._rosprm.LENG_H1.name)
        self.declare_parameter(self._rosprm.LENG_H2.name)
        self.declare_parameter(self._rosprm.LENG_H3.name)
        self.declare_parameter(self._rosprm.LENG_H4.name)
        self.declare_parameter(self._rosprm.LENG_H6.name)
        self.declare_parameter(self._rosprm.LENG_H8.name)
        self.declare_parameter(self._rosprm.LENG_H12.name)
        self.declare_parameter(self._rosprm.LENG_D.name)
        self.declare_parameter(self._rosprm.LENG_W.name)

        # Set ROS parameter
        para = self.get_parameter(self._rosprm.ENA_INST_USDJPY.name)
        self._rosprm.ENA_INST_USDJPY.value = para.value
        para = self.get_parameter(self._rosprm.ENA_INST_EURJPY.name)
        self._rosprm.ENA_INST_EURJPY.value = para.value
        para = self.get_parameter(self._rosprm.ENA_INST_EURUSD.name)
        self._rosprm.ENA_INST_EURUSD.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M1.name)
        self._rosprm.ENA_GRAN_M1.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M2.name)
        self._rosprm.ENA_GRAN_M2.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M3.name)
        self._rosprm.ENA_GRAN_M3.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M4.name)
        self._rosprm.ENA_GRAN_M4.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M5.name)
        self._rosprm.ENA_GRAN_M5.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M10.name)
        self._rosprm.ENA_GRAN_M10.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M15.name)
        self._rosprm.ENA_GRAN_M15.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_M30.name)
        self._rosprm.ENA_GRAN_M30.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H1.name)
        self._rosprm.ENA_GRAN_H1.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H2.name)
        self._rosprm.ENA_GRAN_H2.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H3.name)
        self._rosprm.ENA_GRAN_H3.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H4.name)
        self._rosprm.ENA_GRAN_H4.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H6.name)
        self._rosprm.ENA_GRAN_H6.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H8.name)
        self._rosprm.ENA_GRAN_H8.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_H12.name)
        self._rosprm.ENA_GRAN_H12.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_D.name)
        self._rosprm.ENA_GRAN_D.value = para.value
        para = self.get_parameter(self._rosprm.ENA_GRAN_W.name)
        self._rosprm.ENA_GRAN_W.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M1.name)
        self._rosprm.LENG_M1.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M2.name)
        self._rosprm.LENG_M2.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M3.name)
        self._rosprm.LENG_M3.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M4.name)
        self._rosprm.LENG_M4.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M5.name)
        self._rosprm.LENG_M5.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M10.name)
        self._rosprm.LENG_M10.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M15.name)
        self._rosprm.LENG_M15.value = para.value
        para = self.get_parameter(self._rosprm.LENG_M30.name)
        self._rosprm.LENG_M30.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H1.name)
        self._rosprm.LENG_H1.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H2.name)
        self._rosprm.LENG_H2.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H3.name)
        self._rosprm.LENG_H3.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H4.name)
        self._rosprm.LENG_H4.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H6.name)
        self._rosprm.LENG_H6.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H8.name)
        self._rosprm.LENG_H8.value = para.value
        para = self.get_parameter(self._rosprm.LENG_H12.name)
        self._rosprm.LENG_H12.value = para.value
        para = self.get_parameter(self._rosprm.LENG_D.name)
        self._rosprm.LENG_D.value = para.value
        para = self.get_parameter(self._rosprm.LENG_W.name)
        self._rosprm.LENG_W.value = para.value

        self.logger.debug("[Param]Enable instrument:")
        self.logger.debug("  - USD/JPY:[{}]".format(self._rosprm.ENA_INST_USDJPY.value))
        self.logger.debug("  - EUR/JPY:[{}]".format(self._rosprm.ENA_INST_EURJPY.value))
        self.logger.debug("  - EUR/USD:[{}]".format(self._rosprm.ENA_INST_EURUSD.value))
        self.logger.debug("[Param]Enable granularity:")
        self.logger.debug("  - M1: [{}]".format(self._rosprm.ENA_GRAN_M1.value))
        self.logger.debug("  - M2: [{}]".format(self._rosprm.ENA_GRAN_M2.value))
        self.logger.debug("  - M3: [{}]".format(self._rosprm.ENA_GRAN_M3.value))
        self.logger.debug("  - M4: [{}]".format(self._rosprm.ENA_GRAN_M4.value))
        self.logger.debug("  - M5: [{}]".format(self._rosprm.ENA_GRAN_M5.value))
        self.logger.debug("  - M10:[{}]".format(self._rosprm.ENA_GRAN_M10.value))
        self.logger.debug("  - M15:[{}]".format(self._rosprm.ENA_GRAN_M15.value))
        self.logger.debug("  - M30:[{}]".format(self._rosprm.ENA_GRAN_M30.value))
        self.logger.debug("  - H1: [{}]".format(self._rosprm.ENA_GRAN_H1.value))
        self.logger.debug("  - H2: [{}]".format(self._rosprm.ENA_GRAN_H2.value))
        self.logger.debug("  - H3: [{}]".format(self._rosprm.ENA_GRAN_H3.value))
        self.logger.debug("  - H4: [{}]".format(self._rosprm.ENA_GRAN_H4.value))
        self.logger.debug("  - H6: [{}]".format(self._rosprm.ENA_GRAN_H6.value))
        self.logger.debug("  - H8: [{}]".format(self._rosprm.ENA_GRAN_H8.value))
        self.logger.debug("  - H12:[{}]".format(self._rosprm.ENA_GRAN_H12.value))
        self.logger.debug("  - D:  [{}]".format(self._rosprm.ENA_GRAN_D.value))
        self.logger.debug("  - W:  [{}]".format(self._rosprm.ENA_GRAN_W.value))
        self.logger.debug("[Param]Data length:")
        self.logger.debug("  - M1: [{}]".format(self._rosprm.LENG_M1.value))
        self.logger.debug("  - M2: [{}]".format(self._rosprm.LENG_M2.value))
        self.logger.debug("  - M3: [{}]".format(self._rosprm.LENG_M3.value))
        self.logger.debug("  - M4: [{}]".format(self._rosprm.LENG_M4.value))
        self.logger.debug("  - M5: [{}]".format(self._rosprm.LENG_M5.value))
        self.logger.debug("  - M10:[{}]".format(self._rosprm.LENG_M10.value))
        self.logger.debug("  - M15:[{}]".format(self._rosprm.LENG_M15.value))
        self.logger.debug("  - M30:[{}]".format(self._rosprm.LENG_M30.value))
        self.logger.debug("  - H1: [{}]".format(self._rosprm.LENG_H1.value))
        self.logger.debug("  - H2: [{}]".format(self._rosprm.LENG_H2.value))
        self.logger.debug("  - H3: [{}]".format(self._rosprm.LENG_H3.value))
        self.logger.debug("  - H4: [{}]".format(self._rosprm.LENG_H4.value))
        self.logger.debug("  - H6: [{}]".format(self._rosprm.LENG_H6.value))
        self.logger.debug("  - H8: [{}]".format(self._rosprm.LENG_H8.value))
        self.logger.debug("  - H12:[{}]".format(self._rosprm.LENG_H12.value))
        self.logger.debug("  - D:  [{}]".format(self._rosprm.LENG_D.value))
        self.logger.debug("  - W:  [{}]".format(self._rosprm.LENG_W.value))


def main(args=None):

    rclpy.init(args=args)

    hc = HistoricalCandles()
    try:
        while rclpy.ok():
            rclpy.spin_once(hc, timeout_sec=1.0)
            hc.do_timeout_event()
    except KeyboardInterrupt:
        pass

    hc.destroy_node()
    rclpy.shutdown()
