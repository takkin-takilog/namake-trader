from enum import Enum

from trade_manager_msgs.msg import Instrument as MngInst
from trade_manager_msgs.msg import OrderRequest
from api_msgs.msg import Instrument as ApiInst
from api_msgs.msg import OrderType

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

