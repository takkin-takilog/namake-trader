from enum import Enum, IntEnum
import datetime as dt
from trade_manager_msgs.msg import Instrument as InstMng
from trade_manager_msgs.msg import Granularity as GranMng
from trade_manager_msgs.msg import OrderRequest
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from api_msgs.msg import OrderType

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_TIME_HMS = "%H:%M:%S"

MIN_TIME = dt.time(0, 0, 0)
MAX_TIME = dt.time(23, 59, 59)


class Transitions(Enum):
    """
    Transitions const string.
    """
    NAME = "name"
    CHILDREN = "children"
    ON_ENTER = "on_enter"
    ON_EXIT = "on_exit"
    TRIGGER = "trigger"
    SOURCE = "source"
    DEST = "dest"
    PREPARE = "prepare"
    BEFORE = "before"
    AFTER = "after"
    CONDITIONS = "conditions"
    UNLESS = "unless"


class WeekDay(IntEnum):
    """
    Weekday.
    """
    MON = 0  # Monday
    TUE = 1  # Tuesday
    WED = 2  # Wednesday
    THU = 3  # Thursday
    FRI = 4  # Friday
    SAT = 5  # Saturday
    SUN = 6  # Sunday

    @classmethod
    def get_member_by_id(cls, id_: int):
        for m in cls:
            if id_ == m.value:
                return m
        return None


class CandleColumnNames(Enum):
    """
    Candle column names.
    """
    DATETIME = "datetime"
    ASK_OP = "open(Ask)"
    ASK_HI = "high(Ask)"
    ASK_LO = "low(Ask)"
    ASK_CL = "close(Ask)"
    BID_OP = "open(Bid)"
    BID_HI = "high(Bid)"
    BID_LO = "low(Bid)"
    BID_CL = "close(Bid)"
    MID_OP = "open(Mid)"
    MID_HI = "high(Mid)"
    MID_LO = "low(Mid)"
    MID_CL = "close(Mid)"
    COMP = "complete"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class GranParam(Enum):
    """
    Granularity parameter.
    """
    M1 = (GranApi.GRAN_M1, dt.timedelta(minutes=1))
    M2 = (GranApi.GRAN_M2, dt.timedelta(minutes=2))
    M3 = (GranApi.GRAN_M3, dt.timedelta(minutes=3))
    M4 = (GranApi.GRAN_M4, dt.timedelta(minutes=4))
    M5 = (GranApi.GRAN_M5, dt.timedelta(minutes=5))
    M10 = (GranApi.GRAN_M10, dt.timedelta(minutes=10))
    M15 = (GranApi.GRAN_M15, dt.timedelta(minutes=15))
    M30 = (GranApi.GRAN_M30, dt.timedelta(minutes=30))
    H1 = (GranApi.GRAN_H1, dt.timedelta(hours=1))
    H2 = (GranApi.GRAN_H2, dt.timedelta(hours=2))
    H3 = (GranApi.GRAN_H3, dt.timedelta(hours=3))
    H4 = (GranApi.GRAN_H4, dt.timedelta(hours=4))
    H6 = (GranApi.GRAN_H6, dt.timedelta(hours=6))
    H8 = (GranApi.GRAN_H8, dt.timedelta(hours=8))
    H12 = (GranApi.GRAN_H12, dt.timedelta(hours=12))
    D = (GranApi.GRAN_D, dt.timedelta(days=1))
    W = (GranApi.GRAN_W, dt.timedelta(weeks=1))

    def __init__(self,
                 msg_id: int,
                 timedelta: dt.timedelta,
                 ) -> None:
        self.msg_id = msg_id
        self.timedelta = timedelta

    @classmethod
    def get_member_by_msgid(cls, msg_id: int):
        for m in cls:
            if msg_id == m.msg_id:
                return m
        return None


ORDER_TYP_DICT = {
    OrderRequest.ORDER_TYP_MARKET: OrderType.TYP_MARKET,
    OrderRequest.ORDER_TYP_LIMIT: OrderType.TYP_LIMIT,
    OrderRequest.ORDER_TYP_STOP: OrderType.TYP_STOP
}

INST_DICT = {
    InstMng.INST_USD_JPY: InstApi.INST_USD_JPY,
    InstMng.INST_EUR_JPY: InstApi.INST_EUR_JPY,
    InstMng.INST_EUR_USD: InstApi.INST_EUR_USD
}

GRAN_DICT = {
    GranMng.GRAN_S5: GranApi.GRAN_S5,
    GranMng.GRAN_S10: GranApi.GRAN_S10,
    GranMng.GRAN_S15: GranApi.GRAN_S15,
    GranMng.GRAN_S30: GranApi.GRAN_S30,
    GranMng.GRAN_M1: GranApi.GRAN_M1,
    GranMng.GRAN_M2: GranApi.GRAN_M2,
    GranMng.GRAN_M3: GranApi.GRAN_M3,
    GranMng.GRAN_M4: GranApi.GRAN_M4,
    GranMng.GRAN_M5: GranApi.GRAN_M5,
    GranMng.GRAN_M10: GranApi.GRAN_M10,
    GranMng.GRAN_M15: GranApi.GRAN_M15,
    GranMng.GRAN_M30: GranApi.GRAN_M30,
    GranMng.GRAN_H1: GranApi.GRAN_H1,
    GranMng.GRAN_H2: GranApi.GRAN_H2,
    GranMng.GRAN_H3: GranApi.GRAN_H3,
    GranMng.GRAN_H4: GranApi.GRAN_H4,
    GranMng.GRAN_H6: GranApi.GRAN_H6,
    GranMng.GRAN_H8: GranApi.GRAN_H8,
    GranMng.GRAN_H12: GranApi.GRAN_H12,
    GranMng.GRAN_D: GranApi.GRAN_D,
    GranMng.GRAN_W: GranApi.GRAN_W
}
