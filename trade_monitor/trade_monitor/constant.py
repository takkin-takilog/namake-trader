import math
from enum import Enum
from trade_monitor import utility as utl
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran

FMT_DTTM_API = "%Y-%m-%dT%H:%M:00.000000000Z"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_DATE_YMD = "%Y-%m-%d"
FMT_TIME_HM = "%H:%M"
FMT_TIME_HMS = "%H:%M:%S"
FMT_YMDHMS_DISP = "%Y-%m-%d %H:%M:%S"

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


class InstParam(Enum):
    """
    Instrument parameter.
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

    @classmethod
    def get_member_by_msgid(cls, msg_id: int):
        for m in cls:
            if msg_id == m.msg_id:
                return m
        return None

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
        """
        convert raw value to physical value.
        """
        return raw_value * self.lsb_value

    def convert_phy2raw(self, physical_value: float) -> int:
        """
        convert physical value to raw value.
        """
        return utl.roundi(physical_value / self.lsb_value)


class GranParam(Enum):
    """
    Granularity parameter.
    """
    M1 = (Gran.GRAN_M1, "1分足", "1min")
    M2 = (Gran.GRAN_M2, "2分足", "2min")
    M3 = (Gran.GRAN_M3, "3分足", "3min")
    M4 = (Gran.GRAN_M4, "4分足", "4min")
    M5 = (Gran.GRAN_M5, "5分足", "5min")
    M10 = (Gran.GRAN_M10, "10分足", "10min")
    M15 = (Gran.GRAN_M15, "15分足", "15min")
    M30 = (Gran.GRAN_M30, "30分足", "30min")
    H1 = (Gran.GRAN_H1, "1時間足", "1H")
    H2 = (Gran.GRAN_H2, "2時間足", "2H")
    H3 = (Gran.GRAN_H3, "3時間足", "3H")
    H4 = (Gran.GRAN_H4, "4時間足", "4H")
    H6 = (Gran.GRAN_H6, "6時間足", "6H")
    H8 = (Gran.GRAN_H8, "8時間足", "8H")
    H12 = (Gran.GRAN_H12, "12時間足", "12H")
    D = (Gran.GRAN_D, "日足", "D")

    def __init__(self,
                 msg_id: int,       # ROS message ID
                 text: str,         # For text shown on the widget
                 freq: str,         # Frequency
                 ) -> None:
        self.msg_id = msg_id
        self.text = text
        self.freq = freq

    @classmethod
    def get_member_by_msgid(cls, msg_id: int):
        for m in cls:
            if msg_id == m.msg_id:
                return m
        return None


SPREAD_MSG_LIST = [
    "Mid",
    "Ask",
    "Bid"
]
