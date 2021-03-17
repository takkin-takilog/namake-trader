from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from trade_monitor.constant import InstParam

VALID_INST_LIST = [InstParam.USDJPY,
                   InstParam.EURJPY,
                   InstParam.EURUSD
                   ]


class ColNameOhlc(Enum):
    """
    Pandas OHLC dataframe column name.
    """
    DATETIME = "datetime"
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


class ColNameSma(Enum):
    """
    Pandas SMA(Simple Moving Average) dataframe column name.
    """
    SMA_S = "sma_s"
    SMA_M = "sma_m"
    SMA_L = "sma_l"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
