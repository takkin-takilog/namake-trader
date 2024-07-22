from typing import Any
import datetime as dt
from trade_manager_msgs.msg import Instrument as InstMng
from trade_manager_msgs.msg import Granularity as GranMng
from api_server_msgs.msg import Instrument as InstApi
from api_server_msgs.msg import Granularity as GranApi

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_TIME_HMS = "%H:%M:%S"

MIN_TIME = dt.time(0, 0, 0)
MAX_TIME = dt.time(23, 59, 59)

BUCKUP_DIR = "/trade_backup/"


class ConstantGroup:
    """
    Container class for grouped constants.
    """

    @classmethod
    def to_list(cls) -> list[Any]:
        """
        Returns all constant values in class with the attribute requirements.

        """
        member = []
        for k, v in cls.__dict__.items():
            if not k.startswith("__") and not callable(v):
                member.append(v)
        return member


class WeekDay(ConstantGroup):
    """
    Weekday.
    """

    MON = 0  # Monday
    TUE = 1  # Tuesday
    WED = 2  # Wednesday
    THU = 3  # Thursday
    FRI = 4  # Friday
    SAT = 5  # Saturday
    SUN = 6  # Sunday


class Transitions(ConstantGroup):
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


class CandleColumnNames(ConstantGroup):
    """
    Candle column names.
    """

    DATETIME = "datetime"
    ASK_OP = "ask_op"
    ASK_HI = "ask_hi"
    ASK_LO = "ask_lo"
    ASK_CL = "ask_cl"
    BID_OP = "bid_op"
    BID_HI = "bid_hi"
    BID_LO = "bid_lo"
    BID_CL = "bid_cl"
    MID_OP = "mid_op"
    MID_HI = "mid_hi"
    MID_LO = "mid_lo"
    MID_CL = "mid_cl"
    COMP = "comp"


INST_DICT = {
    InstMng.INST_USD_JPY: InstApi.INST_USD_JPY,
    InstMng.INST_EUR_JPY: InstApi.INST_EUR_JPY,
    InstMng.INST_EUR_USD: InstApi.INST_EUR_USD,
    InstMng.INST_GBP_JPY: InstApi.INST_GBP_JPY,
    InstMng.INST_AUD_JPY: InstApi.INST_AUD_JPY,
    InstMng.INST_NZD_JPY: InstApi.INST_NZD_JPY,
    InstMng.INST_CAD_JPY: InstApi.INST_CAD_JPY,
    InstMng.INST_CHF_JPY: InstApi.INST_CHF_JPY,
}

GRAN_DICT = {
    GranMng.GRAN_S5: GranApi.GRAN_S5,
    GranMng.GRAN_S10: GranApi.GRAN_S10,
    GranMng.GRAN_S15: GranApi.GRAN_S15,
    GranMng.GRAN_S30: GranApi.GRAN_S30,
    GranMng.GRAN_M1: GranApi.GRAN_M1,
    GranMng.GRAN_M2: GranApi.GRAN_M2,
    GranMng.GRAN_M3: GranApi.GRAN_M3,
    GranMng.GRAN_M4: GranApi.GRAN_M4,
    GranMng.GRAN_M5: GranApi.GRAN_M5,
    GranMng.GRAN_M10: GranApi.GRAN_M10,
    GranMng.GRAN_M15: GranApi.GRAN_M15,
    GranMng.GRAN_M30: GranApi.GRAN_M30,
    GranMng.GRAN_H1: GranApi.GRAN_H1,
    GranMng.GRAN_H2: GranApi.GRAN_H2,
    GranMng.GRAN_H3: GranApi.GRAN_H3,
    GranMng.GRAN_H4: GranApi.GRAN_H4,
    GranMng.GRAN_H6: GranApi.GRAN_H6,
    GranMng.GRAN_H8: GranApi.GRAN_H8,
    GranMng.GRAN_H12: GranApi.GRAN_H12,
    GranMng.GRAN_D: GranApi.GRAN_D,
    GranMng.GRAN_W: GranApi.GRAN_W,
}
