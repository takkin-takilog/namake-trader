import math


def roundi(number: float) -> int:
    """
    Rounds a number to the nearest integer

    :param number: Any real number that you want rounded nearest integer.
    :return: Rounded integer number.
    """
    return int((number * 2 + 1) // 2)


def rounddown(number: float, digits: int) -> float:
    """
    Rounds a number down, toward zero.

    :param number: Any real number that you want rounded down.
    :param digits: The number of digits to which you want to round number.
    :return: Rounded down number.
    """
    return float(int(number * 10**digits) / (10**digits))


def roundup(number: float, digits: int) -> float:
    """
    Rounds a number up, away from zero.

    :param number: Any real number that you want rounded up.
    :param digits: The number of digits to which you want to round number.
    :return: Rounded up number.
    """
    r_num: float = 0.0
    if 0 < number:
        r_num = math.ceil(number * 10**digits) / (10**digits)
    else:
        r_num = math.floor(number * 10**digits) / (10**digits)
    return r_num
