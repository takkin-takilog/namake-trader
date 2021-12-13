from enum import Enum


class ColChart(Enum):
    """
    Pandas SMA back test result dataframe column name.
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
    BASE_SMA = "base_sma"
    POS_STD = "pos_std"
    NEG_STD = "neg_std"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
