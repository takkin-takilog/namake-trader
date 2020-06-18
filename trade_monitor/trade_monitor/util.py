from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran

DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

# Candlestick data frame column name
COL_NAME_TIME = "time"
COL_NAME_ASK_OP = "open(Ask)"
COL_NAME_ASK_HI = "high(Ask)"
COL_NAME_ASK_LO = "low(Ask)"
COL_NAME_ASK_CL = "close(Ask)"
COL_NAME_MID_OP = "open(Mid)"
COL_NAME_MID_HI = "high(Mid)"
COL_NAME_MID_LO = "low(Mid)"
COL_NAME_MID_CL = "close(Mid)"
COL_NAME_BID_OP = "open(Bid)"
COL_NAME_BID_HI = "high(Bid)"
COL_NAME_BID_LO = "low(Bid)"
COL_NAME_BID_CL = "close(Bid)"
COL_NAME_COMP = "complete"

CANDLE_COL_NAME_LIST = [COL_NAME_TIME,
                        COL_NAME_ASK_OP,
                        COL_NAME_ASK_HI,
                        COL_NAME_ASK_LO,
                        COL_NAME_ASK_CL,
                        COL_NAME_MID_OP,
                        COL_NAME_MID_HI,
                        COL_NAME_MID_LO,
                        COL_NAME_MID_CL,
                        COL_NAME_BID_OP,
                        COL_NAME_BID_HI,
                        COL_NAME_BID_LO,
                        COL_NAME_BID_CL,
                        COL_NAME_COMP
                        ]


class MsgGranDict():

    def __init__(self,
                 msg_id: int,
                 text: str
                 ) -> None:
        self.__msg_id = msg_id
        self.__text = text

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text


class MsgInstDict():

    def __init__(self,
                 msg_id: int,
                 text: str,
                 decimal_digit: int
                 ) -> None:
        self.__msg_id = msg_id
        self.__text = text
        self.__decimal_digit = decimal_digit

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text

    @property
    def decimal_digit(self) -> int:
        return self.__decimal_digit


INST_MSG_LIST = [
    MsgInstDict(Inst.INST_USD_JPY, "USD/JPY", 3),
    MsgInstDict(Inst.INST_EUR_JPY, "EUR/JPY", 3),
    MsgInstDict(Inst.INST_EUR_USD, "EUR/USD", 5),
]

GRAN_MSG_LIST = [
    MsgGranDict(Gran.GRAN_D, "日足"),
    MsgGranDict(Gran.GRAN_H4, "４時間足"),
    MsgGranDict(Gran.GRAN_H1, "１時間足"),
    MsgGranDict(Gran.GRAN_M10, "１０分足"),
]

SPREAD_MSG_LIST = [
    "Mid",
    "Ask",
    "Bid"
]


def remove_all_items_of_comboBox(combo_box):

    idx = combo_box.currentIndex()
    while -1 < idx:
        combo_box.removeItem(idx)
        idx = combo_box.currentIndex()
