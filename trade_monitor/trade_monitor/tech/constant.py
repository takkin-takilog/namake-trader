from enum import Enum, IntEnum, auto
from dataclasses import dataclass
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


class ColNameOhlc(Enum):
    """
    Pandas OHLC dataframe column name.
    """
    DATETIME = "datetime"
    MID_O = "mid_o"
    MID_H = "mid_h"
    MID_L = "mid_l"
    MID_C = "mid_c"
    SMA_S = "sma_s"
    SMA_M = "sma_m"
    SMA_L = "sma_l"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColNameSma(Enum):
    """
    Pandas SMA(Simple Moving Average) dataframe column name.
    """
    DATETIME = "datetime"
    CRS_TYP = "cross_type"
    CRS_LVL = "cross_level"
    ANG_S = "angle_s"
    ANG_M = "angle_m"
    ANG_L = "angle_l"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
