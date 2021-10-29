from dataclasses import dataclass
import datetime as dt
import pandas as pd
from trade_manager.constant import SATURDAY, SUNDAY, MONDAY


@dataclass
class RosParam():
    """
    Chart tag.
    """
    name: str = None
    value = None


def is_market_close(dt_: dt.datetime) -> bool:

    is_close = False
    if dt_.weekday() == SATURDAY:
        close_time = get_market_close_time(dt_.date())
        if close_time <= dt_.time():
            is_close = True
    elif dt_.weekday() == SUNDAY:
        is_close = True
    elif dt_.weekday() == MONDAY:
        open_time = get_market_open_time(dt_.date())
        if dt_.time() < open_time:
            is_close = True
    else:
        is_close = False

    return is_close


def is_summer_time(dt_: dt.date) -> bool:

    pddt = pd.Timestamp(dt_.isoformat() + " 03:00:00",
                        tz="America/New_York")
    if 3600 <= pddt.dst().seconds:
        return True
    else:
        return False


def get_market_open_time(dt_: dt.date) -> dt.time:

    if is_summer_time(dt_):
        open_time = dt.time(6, 0)
    else:
        open_time = dt.time(7, 0)

    return open_time


def get_market_close_time(dt_: dt.date) -> dt.time:

    if is_summer_time(dt_):
        close_time = dt.time(6, 0)
    else:
        close_time = dt.time(7, 0)

    return close_time


def limit(val, min_val, max_val):

    if val < min_val:
        ret_val = min_val
    elif max_val < val:
        ret_val = max_val
    else:
        ret_val = val

    return ret_val


def roundi(val: float) -> int:
    return int((val * 2 + 1) // 2)
