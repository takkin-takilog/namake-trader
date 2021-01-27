from enum import Enum

FMT_DTTM_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_DTTM_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"


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
