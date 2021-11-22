from enum import Enum
from trade_monitor.constant import InstParam, GranParam


VALID_INST_LIST = [
    InstParam.USDJPY,
    InstParam.EURJPY,
    InstParam.EURUSD
]

VALID_GRAN_LIST = [
    GranParam.D,
    GranParam.H1,
    GranParam.M10,
    GranParam.M1,
]


class ColOhlc(Enum):
    """
    Pandas OHLC dataframe column name.
    """
    DATETIME = "datetime"
    # ---------- OHLC ----------
    ASK_O = "ask_o"
    ASK_H = "ask_h"
    ASK_L = "ask_l"
    ASK_C = "ask_c"
    MID_O = "mid_o"
    MID_H = "mid_h"
    MID_L = "mid_l"
    MID_C = "mid_c"
    BID_O = "bid_o"
    BID_H = "bid_h"
    BID_L = "bid_l"
    BID_C = "bid_c"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class SpreadTyp(Enum):
    """
    Spread type.
    """
    MID = "Mid"
    ASK = "Ask"
    BID = "Bid"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]

    @classmethod
    def get_index(cls, mem):
        idx = 0
        for m in cls:
            if m == mem:
                return idx
            else:
                idx += 1
        return -1
