import math
from enum import Enum
import datetime as dt
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from . import utils as utl


class InstParam(Enum):
    """
    Instrument parameter.
    """

    USDJPY = (InstApi.INST_USD_JPY, "usdjpy", "USD/JPY", 3)
    EURJPY = (InstApi.INST_EUR_JPY, "eurjpy", "EUR/JPY", 3)
    EURUSD = (InstApi.INST_EUR_USD, "eurusd", "EUR/USD", 5)
    GBPJPY = (InstApi.INST_GBP_JPY, "gbpjpy", "GBP/JPY", 3)
    AUDJPY = (InstApi.INST_AUD_JPY, "audjpy", "AUD/JPY", 3)
    NZDJPY = (InstApi.INST_NZD_JPY, "nzdjpy", "NZD/JPY", 3)
    CADJPY = (InstApi.INST_CAD_JPY, "cadjpy", "CAD/JPY", 3)
    CHFJPY = (InstApi.INST_CHF_JPY, "chfjpy", "CHF/JPY", 3)

    def __init__(
        self,
        msg_id: int,  # ROS message ID
        namespace: str,  # ROS message namespace
        text: str,  # For text shown on the widget
        digit: int,  # Number of digits after the decimal point
    ) -> None:
        self.msg_id = msg_id
        self.namespace = namespace
        self.text = text
        self.digit = digit
        self._one_pip = math.pow(10, -digit)
        self._one_pip_inv = int(math.pow(10, digit))
        self._one_pip_str = format(self._one_pip, "." + str(digit) + "f")

    @classmethod
    def get_member_by_msgid(cls, msg_id: int) -> "InstParam":
        for m in cls:
            if msg_id == m.msg_id:
                return m
        raise ValueError("msg_id:[{}] is not found in InstParam".format(msg_id))

    @property
    def one_pip(self) -> float:
        """
        One pip value.
        """
        return self._one_pip

    @property
    def one_pip_inv(self) -> int:
        """
        One pip inverse value.
        return type is "int".
        """
        return self._one_pip_inv

    @property
    def one_pip_str(self) -> str:
        """
        One pip string value.
        """
        return self._one_pip_str

    def convert_pips2phy(self, pips_value: int) -> float:
        """
        convert pips value to physical value.
        """
        return pips_value * self.one_pip

    def convert_phy2pips(self, physical_value: float) -> int:
        """
        convert physical value to pips value.
        """
        return utl.roundi(physical_value * self.one_pip_inv)

    def round_pips(self, physical_value: float) -> float:
        """
        round a price value to the digits.
        """
        p = 10**self.digit
        return float((physical_value * p * 2 + 1) // 2 / p)


class GranParam(Enum):
    """
    Granularity parameter.
    """

    M1 = (GranApi.GRAN_M1, "m1", "1分足", "1min", dt.timedelta(minutes=1))
    M2 = (GranApi.GRAN_M2, "m2", "2分足", "2min", dt.timedelta(minutes=2))
    M3 = (GranApi.GRAN_M3, "m3", "3分足", "3min", dt.timedelta(minutes=3))
    M4 = (GranApi.GRAN_M4, "m4", "4分足", "4min", dt.timedelta(minutes=4))
    M5 = (GranApi.GRAN_M5, "m5", "5分足", "5min", dt.timedelta(minutes=5))
    M10 = (GranApi.GRAN_M10, "m10", "10分足", "10min", dt.timedelta(minutes=10))
    M15 = (GranApi.GRAN_M15, "m15", "15分足", "15min", dt.timedelta(minutes=15))
    M30 = (GranApi.GRAN_M30, "m30", "30分足", "30min", dt.timedelta(minutes=30))
    H1 = (GranApi.GRAN_H1, "h1", "1時間足", "1H", dt.timedelta(hours=1))
    H2 = (GranApi.GRAN_H2, "h2", "2時間足", "2H", dt.timedelta(hours=2))
    H3 = (GranApi.GRAN_H3, "h3", "3時間足", "3H", dt.timedelta(hours=3))
    H4 = (GranApi.GRAN_H4, "h4", "4時間足", "4H", dt.timedelta(hours=4))
    H6 = (GranApi.GRAN_H6, "h6", "6時間足", "6H", dt.timedelta(hours=6))
    H8 = (GranApi.GRAN_H8, "h8", "8時間足", "8H", dt.timedelta(hours=8))
    H12 = (GranApi.GRAN_H12, "h12", "12時間足", "12H", dt.timedelta(hours=12))
    D = (GranApi.GRAN_D, "d", "日足", "D", dt.timedelta(days=1))
    W = (GranApi.GRAN_W, "w", "週足", "W", dt.timedelta(weeks=1))

    def __init__(
        self,
        msg_id: int,  # ROS message ID
        namespace: str,  # ROS message namespace
        text: str,  # For text shown on the widget
        freq: str,  # Frequency
        timedelta: dt.timedelta,  # Time delta
    ) -> None:
        self.msg_id = msg_id
        self.namespace = namespace
        self.text = text
        self.freq = freq
        self.timedelta = timedelta

    @classmethod
    def get_member_by_msgid(cls, msg_id: int) -> "GranParam":
        for m in cls:
            if msg_id == m.msg_id:
                return m
        raise ValueError("msg_id:[{}] is not found in GranParam".format(msg_id))
