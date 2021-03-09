from enum import Enum
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
    DATA_ID = "data_id"
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


class ColNameGap(Enum):
    """
    Pandas gap dataframe column name.
    """
    DATE = "date"
    DATA_ID = "data_id"
    GAP_DIR = "gap_dir"
    CLOSE_PRICE_MID = "close_price_mid"
    OPEN_PRICE_MID = "open_price_mid"
    GAP_PRICE_MID = "gap_price_mid"
    GAP_PRICE_REAL = "gap_price_real"
    VALID_FLAG = "valid_flag"
    SUCCESS_FLAG = "success_flag"
    GAP_FILLED_TIME = "gap_filled_time"
    MAX_OPEN_RANGE = "max_open_range"
    END_CLOSE_PRICE = "end_close_price"
    END_DIFF_PRICE = "end_diff_price"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
