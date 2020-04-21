import rclpy
from rclpy.node import Node
from api_msgs.msg import Granularity, Instrument

GRAN_ID_DICT = {
    Granularity.GRAN_S5: "S5",  # 5 seconds
    Granularity.GRAN_S10: "S10",  # 10 seconds
    Granularity.GRAN_S15: "S15",  # 15 seconds
    Granularity.GRAN_S30: "S30",  # 30 seconds
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

ORDER_INST_ID_DICT = {
    Instrument.INST_USD_JPY: "USD_JPY",
    Instrument.INST_EUR_JPY: "EUR_JPY",
    Instrument.INST_EUR_USD: "EUR_USD",
}


class CandlestickService(Node):

    def __init__(self):
        super().__init__("candlestick_service")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)


def main(args=None):
    rclpy.init(args=args)
    cs = CandlestickService()

    try:
        rclpy.spin(cs)
    except KeyboardInterrupt:
        pass

    cs.destroy_node()
    rclpy.shutdown()
