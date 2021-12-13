from enum import Enum


class SpreadTyp(Enum):
    """
    Spread type.
    """
    MID = "Mid"
    ASK = "Ask"
    BID = "Bid"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]

    @classmethod
    def get_index(cls, mem):
        idx = 0
        for m in cls:
            if m == mem:
                return idx
            else:
                idx += 1
        return -1
