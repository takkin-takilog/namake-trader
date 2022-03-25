from enum import Enum

FMT_DTTM_API = "%Y-%m-%dT%H:%M:00.000000000Z"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_DATE_YMD = "%Y-%m-%d"
FMT_TIME_HM = "%H:%M"
FMT_TIME_HMS = "%H:%M:%S"

FMT_QT_YMDHMS = "yyyy-MM-ddTHH:mm:ss"
FMT_QT_DATE_YMD = "yyyy-MM-dd"
FMT_QT_TIME = "HH:mm"

FMT_DISP_YMDHMS = "%Y-%m-%d %H:%M:%S"


class QtColor(Enum):
    """
    Qt color name.
    """
    WHITESMOKE = "#f5f5f5"
    DARKVIOLET = "#9400d3"
    CRIMSON = "#dc143c"
    ORANGERED = "#ff4500"
    SEAGREEN = "#2e8b57"
    ROYALBLUE = "#4169e1"
    DEEPSKYBLUE = "#00bfff"
    SKYBLUE = "#87ceeb"
    TOMATO = "#ff6347"


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


SPREAD_MSG_LIST = [
    "Mid",
    "Ask",
    "Bid"
]

TRADE_TYP_LIST = [
    "Follower",
    "Contrarian",
]
