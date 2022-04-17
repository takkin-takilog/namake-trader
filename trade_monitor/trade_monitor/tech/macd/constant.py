from enum import Enum


class ColOhlcChart(Enum):
    """
    Pandas MACD backtest result dataframe column name.
    OHLC data.
    """
    TIME = "time"
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
    EMA_L = "ema_l"
    EMA_S = "ema_s"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColMacdChart(Enum):
    """
    Pandas MACD backtest result dataframe column name.
    MACD data.
    """
    TIME = "time"
    MACD = "macd"
    SIGNAL = "signal"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
