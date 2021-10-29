import datetime as dt

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"
FMT_YMDHMS = "%Y-%m-%dT%H:%M:%S"
FMT_TIME_HMS = "%H:%M:%S"

MIN_TIME = dt.time(0, 0, 0)
MAX_TIME = dt.time(23, 59, 59)

MONDAY = 0  # Monday
TUESDAY = 1  # Tuesday
WEDNESDAY = 2  # Wednesday
THURSDAY = 3  # Thursday
FRIDAY = 4  # Friday
SATURDAY = 5  # Saturday
SUNDAY = 6  # Sunday
