from enum import Enum
import datetime as dt
from trade_manager_msgs.msg import Instrument as MngInst
from trade_manager_msgs.msg import OrderRequest
from api_msgs.msg import Instrument as ApiInst
from api_msgs.msg import OrderType
from api_msgs.msg import Granularity as Gran

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"


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


class CandleColumnNames(Enum):
    """
    Candle column names.
    """
    DATE = "date"
    TIME = "time"
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
    M1 = (Gran.GRAN_M1, dt.timedelta(minutes=1))
    M2 = (Gran.GRAN_M2, dt.timedelta(minutes=2))
    M3 = (Gran.GRAN_M3, dt.timedelta(minutes=3))
    M4 = (Gran.GRAN_M4, dt.timedelta(minutes=4))
    M5 = (Gran.GRAN_M5, dt.timedelta(minutes=5))
    M10 = (Gran.GRAN_M10, dt.timedelta(minutes=10))
    M15 = (Gran.GRAN_M15, dt.timedelta(minutes=15))
    M30 = (Gran.GRAN_M30, dt.timedelta(minutes=30))
    H1 = (Gran.GRAN_H1, dt.timedelta(hours=1))
    H2 = (Gran.GRAN_H2, dt.timedelta(hours=2))
    H3 = (Gran.GRAN_H3, dt.timedelta(hours=3))
    H4 = (Gran.GRAN_H4, dt.timedelta(hours=4))
    H6 = (Gran.GRAN_H6, dt.timedelta(hours=6))
    H8 = (Gran.GRAN_H8, dt.timedelta(hours=8))
    H12 = (Gran.GRAN_H12, dt.timedelta(hours=12))
    D = (Gran.GRAN_D, dt.timedelta(days=1))
    W = (Gran.GRAN_W, dt.timedelta(weeks=1))

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


INST_DICT = {
    OrderRequest.ORDER_TYP_MARKET: OrderType.TYP_MARKET,
    OrderRequest.ORDER_TYP_LIMIT: OrderType.TYP_LIMIT,
    OrderRequest.ORDER_TYP_STOP: OrderType.TYP_STOP,
}

ORDER_TYP_DICT = {
    MngInst.INST_USD_JPY: ApiInst.INST_USD_JPY,
    MngInst.INST_EUR_JPY: ApiInst.INST_EUR_JPY,
    MngInst.INST_EUR_USD: ApiInst.INST_EUR_USD,
}
