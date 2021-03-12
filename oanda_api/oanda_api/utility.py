from dataclasses import dataclass
import datetime as dt
from typing import Dict
from oanda_api.constant import FMT_YMDHMSF

_JST_OFS = dt.timedelta(hours=9)


@dataclass
class RosParam():
    """
    Chart tag.
    """
    name: str = None
    value = None


def convert_datetime_jst(oanda_dt: str,
                         fmt: str = FMT_YMDHMSF
                         ) -> str:

    time_list = oanda_dt.split(".")
    dt_str = time_list[0] + "." + time_list[1][:6]
    dt_ = dt.datetime.strptime(dt_str, FMT_YMDHMSF)
    return (dt_ + _JST_OFS).strftime(fmt)


def convert_from_utc_to_jst(utc_dt: dt.datetime,
                            ) -> dt.datetime:
    return utc_dt + _JST_OFS


def convert_from_jst_to_utc(jst_dt: dt.datetime,
                            ) -> dt.datetime:
    return jst_dt - _JST_OFS


def inverse_dict(d: Dict[int, str]) -> Dict[str, int]:
    return {v: k for k, v in d.items()}


def roundi(val: float) -> int:
    return int((val * 2 + 1) // 2)


def roundf(val: float, digit: int=0) -> float:
    p = 10 ** digit
    return (val * p * 2 + 1) // 2 / p
