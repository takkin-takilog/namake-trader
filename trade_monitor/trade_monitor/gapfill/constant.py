from enum import Enum
from trade_monitor.constant import InstInfo

VALID_INST_LIST = [InstInfo.USDJPY,
                   InstInfo.EURJPY,
                   InstInfo.EURUSD
                   ]


class ColumnName(Enum):
    """
    Pandas dataframe column name.
    """
    DATE = "date"
    GPA_DIR = "gap dir"
    GPA_CLOSE_PRICE = "gap close price"
    GPA_OPEN_PRICE = "gap open price"
    GPA_PRICE_MID = "gap price(mid)"
    GPA_PRICE_REAL = "gap price(real)"
    VALID_FLAG = "valid flag"
    SUCCESS_FLAG = "success flag"
    GAP_FILLED_TIME = "gap filled time"
    MAX_OPEN_RANGE = "max open range"
    END_CLOSE_PRICE = "end close price"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
