import math
from enum import Enum
import datetime as dt
from api_msgs.msg import Instrument as Inst
from api_msgs.msg import Granularity as Gran

FMT_DTTM_API = "%Y-%m-%dT%H:%M:00.000000000Z"
FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"

ADD_CIPHERS = "HIGH:!DH"


class InstParam(Enum):
    """
    Instrument parameter.
    """
    USD_JPY = (Inst.INST_USD_JPY, 3)
    EUR_JPY = (Inst.INST_EUR_JPY, 3)
    EUR_USD = (Inst.INST_EUR_USD, 5)

    def __init__(self,
                 msg_id: int,       # ROS message ID
                 digit: int,        # Number of digits after the decimal point
                 ) -> None:
        self.msg_id = msg_id
        self.digit = digit

    @classmethod
    def get_member_by_msgid(cls, msg_id: int):
        for m in cls:
            if msg_id == m.msg_id:
                return m
        return None

    @classmethod
    def get_member_by_name(cls, name: str):
        for m in cls:
            if name == m.name:
                return m
        return None

    @property
    def lsb_value(self) -> float:
        """
        Least significant bit (Resolution).
        return type is "float".
        """
        return math.pow(10, -self.digit)

    @property
    def lsb_str(self) -> str:
        """
        Least significant bit (Resolution).
        return type is "string".
        """
        return format(self.lsb_value, "." + str(self.digit) + "f")


class GranParam(Enum):
    """
    Granularity parameter.
    """
    M1 = (Gran.GRAN_M1, dt.timedelta(minutes=1))
    M2 = (Gran.GRAN_M2, dt.timedelta(minutes=2))
    M3 = (Gran.GRAN_M3, dt.timedelta(minutes=3))
    M4 = (Gran.GRAN_M4, dt.timedelta(minutes=4))
    M5 = (Gran.GRAN_M5, dt.timedelta(minutes=5))
    M10 = (Gran.GRAN_M10, dt.timedelta(minutes=10))
    M15 = (Gran.GRAN_M15, dt.timedelta(minutes=15))
    M30 = (Gran.GRAN_M30, dt.timedelta(minutes=30))
    H1 = (Gran.GRAN_H1, dt.timedelta(hours=1))
    H2 = (Gran.GRAN_H2, dt.timedelta(hours=2))
    H3 = (Gran.GRAN_H3, dt.timedelta(hours=3))
    H4 = (Gran.GRAN_H4, dt.timedelta(hours=4))
    H6 = (Gran.GRAN_H6, dt.timedelta(hours=6))
    H8 = (Gran.GRAN_H8, dt.timedelta(hours=8))
    H12 = (Gran.GRAN_H12, dt.timedelta(hours=12))
    D = (Gran.GRAN_D, dt.timedelta(days=1))
    W = (Gran.GRAN_W, dt.timedelta(weeks=1))

    def __init__(self,
                 msg_id: int,               # ROS message ID
                 timedelta: dt.timedelta    # Time delta
                 ) -> None:
        self.msg_id = msg_id
        self.timedelta = timedelta

    @classmethod
    def get_member_by_msgid(cls, msg_id: int):
        for m in cls:
            if msg_id == m.msg_id:
                return m
        return None
