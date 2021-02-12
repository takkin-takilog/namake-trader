from dataclasses import dataclass
import datetime as dt
import pandas as pd


@dataclass
class RosParam():
    """
    Chart tag.
    """
    name: str = None
    value = None


def is_summer_time(dt_: dt.date) -> bool:

    pddt = pd.Timestamp(dt_.isoformat() + " 03:00:00",
                        tz="America/New_York")
    if 3600 <= pddt.dst().seconds:
        return True
    else:
        return False
