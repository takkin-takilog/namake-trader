import math
from dataclasses import dataclass
from enum import Enum, IntEnum, auto
from trade_monitor import utility as utl
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran

FMT_DTTM_API = "%Y-%m-%dT%H:%M:00.000000000Z"
FMT_DATE_YMD = "%Y-%m-%d"
FMT_TIME_HM = "%H:%M"
FMT_TIME_HMS = "%H:%M:%S"

FMT_QT_DATE_YMD = "yyyy-MM-dd"
FMT_QT_TIME = "HH:mm"


class CandleColumnName(Enum):
    """
    Candlestick dataframe column name.
    """
    TIME = "time"
    ASK_OP = "open(Ask)"
    ASK_HI = "high(Ask)"
    ASK_LO = "low(Ask)"
    ASK_CL = "close(Ask)"
    MID_OP = "open(Mid)"
    MID_HI = "high(Mid)"
    MID_LO = "low(Mid)"
    MID_CL = "close(Mid)"
    BID_OP = "open(Bid)"
    BID_HI = "high(Bid)"
    BID_LO = "low(Bid)"
    BID_CL = "close(Bid)"
    COMP = "complete"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


GRAN_FREQ_DICT = {
    Gran.GRAN_M1: "1min",
    Gran.GRAN_M2: "2min",
    Gran.GRAN_M3: "3min",
    Gran.GRAN_M4: "4min",
    Gran.GRAN_M5: "5min",
    Gran.GRAN_M10: "10min",
    Gran.GRAN_M15: "15min",
    Gran.GRAN_M30: "30min",
    Gran.GRAN_H1: "1H",
    Gran.GRAN_H2: "2H",
    Gran.GRAN_H3: "3H",
    Gran.GRAN_H4: "4H",
    Gran.GRAN_H6: "6H",
    Gran.GRAN_H8: "8H",
    Gran.GRAN_H12: "12H",
    Gran.GRAN_D: "D"
}


@dataclass
class _MsgInstDict:
    msg_id: int
    namespace: str
    text: str
    decimal_digit: int
    min_unit: str


INST_MSG_LIST = [
    _MsgInstDict(Inst.INST_USD_JPY, "usdjpy", "USD/JPY", 3, "0.001"),
    _MsgInstDict(Inst.INST_EUR_JPY, "eurjpy", "EUR/JPY", 3, "0.001"),
    _MsgInstDict(Inst.INST_EUR_USD, "eurusd", "EUR/USD", 5, "0.00001"),
]


class InstInfo(Enum):
    """
    Instrument info.
    """
    USDJPY = (Inst.INST_USD_JPY, "usdjpy", "USD/JPY", 3)
    EURJPY = (Inst.INST_EUR_JPY, "eurjpy", "EUR/JPY", 3)
    EURUSD = (Inst.INST_EUR_USD, "eurusd", "EUR/USD", 5)

    def __init__(self,
                 msg_id: int,       # ROS message ID
                 namespace: str,    # ROS message namespace
                 text: str,         # For text shown on the widget
                 digit: int,        # Number of digits after the decimal point
                 ) -> None:
        self.msg_id = msg_id
        self.namespace = namespace
        self.text = text
        self.digit = digit

    @property
    def lsb_value(self) -> float:
        """
        Least significant bit (Resolution).
        return type is "float".
        """
        return math.pow(10, -self.digit)

    @property
    def lsb_str(self) -> str:
        """
        Least significant bit (Resolution).
        return type is "string".
        """
        return format(self.lsb_value, "." + str(self.digit) + "f")

    def convert_raw2phy(self, raw_value: int) -> float:
        return raw_value * self.lsb_value

    def convert_phy2raw(self, physical_value: float) -> int:
        return utl.roundi(physical_value / self.lsb_value)









@dataclass
class _MsgGranDict:
    msg_id: int
    text: str


GRAN_MSG_LIST = [
    _MsgGranDict(Gran.GRAN_D, "日足"),
    _MsgGranDict(Gran.GRAN_H4, "４時間足"),
    _MsgGranDict(Gran.GRAN_H1, "１時間足"),
    _MsgGranDict(Gran.GRAN_M10, "１０分足"),
]

SPREAD_MSG_LIST = [
    "Mid",
    "Ask",
    "Bid"
]
