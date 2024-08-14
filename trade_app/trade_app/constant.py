from typing import Any

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_TIME_HM = "%H:%M"
FMT_TIME_HMS = "%H:%M:%S"

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


class EntryDir(ConstantGroup):
    """
    Entry direction.
    """

    LONG = 1
    SHORT = -1


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
