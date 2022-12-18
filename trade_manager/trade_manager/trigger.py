import datetime as dt


class TimeTrigger:
    """
    Time trigger class.
    """

    def __init__(
        self,
        second: int = 0,
        minute: int | None = None,
        hour: int | None = None,
    ) -> None:
        self._enable_trigger = False

        self._second = self._check_second_fields(second)
        self._enable_hour = False
        self._enable_minute = False

        self._enable_hour = False if hour is None else True
        self._enable_minute = False if minute is None else True

        now = dt.datetime.now().replace(microsecond=0)

        if self._enable_hour:
            self._enable_minute = True
            self._hour = self._check_hour_fields(hour)
            if minute is None:
                self._minute = 0
            else:
                self._minute = self._check_minute_fields(minute)
            self._add_timedelta = dt.timedelta(days=1)
            self._th_datetime = now.replace(
                hour=self._hour, minute=self._minute, second=self._second
            )
        elif self._enable_minute:
            self._minute = self._check_minute_fields(minute)
            self._add_timedelta = dt.timedelta(hours=1)
            self._th_datetime = now.replace(minute=self._minute, second=self._second)
        else:
            self._add_timedelta = dt.timedelta(minutes=1)
            self._th_datetime = now.replace(second=self._second)

        self.start()

    def start(self) -> None:
        now_time = dt.datetime.now()
        if self._th_datetime < now_time:
            while self._th_datetime < now_time:
                self._th_datetime += self._add_timedelta
        self._enable_trigger = True

    def stop(self) -> None:
        self._enable_trigger = False

    def triggered(self, now_time: dt.datetime | None = None) -> bool:
        if not self._enable_trigger:
            return False
        if now_time is None:
            now_time = dt.datetime.now()
        trig = False
        if self._th_datetime < now_time:
            trig = True
            while self._th_datetime < now_time:
                self._th_datetime += self._add_timedelta
        return trig

    def next_trigger_time(self) -> dt.datetime:
        return self._th_datetime

    def _check_hour_fields(self, hour):
        hour = self._check_int_field(hour)
        if not 0 <= hour <= 23:
            raise ValueError("hour must be in 0..23", hour)
        return hour

    def _check_minute_fields(self, minute):
        minute = self._check_int_field(minute)
        if not 0 <= minute <= 59:
            raise ValueError("minute must be in 0..59", minute)
        return minute

    def _check_second_fields(self, second):
        second = self._check_int_field(second)
        if not 0 <= second <= 59:
            raise ValueError("second must be in 0..59", second)
        return second

    def _check_int_field(self, value):
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            raise TypeError("integer argument expected, got float")
        try:
            value = value.__index__()
        except AttributeError:
            pass
        else:
            if not isinstance(value, int):
                raise TypeError(
                    "__index__ returned non-int (type %s)" % type(value).__name__
                )
            return value
        orig = value
        try:
            value = value.__int__()
        except AttributeError:
            pass
        else:
            if not isinstance(value, int):
                raise TypeError(
                    "__int__ returned non-int (type %s)" % type(value).__name__
                )
            import warnings

            warnings.warn(
                "an integer is required (got type %s)" % type(orig).__name__,
                DeprecationWarning,
                stacklevel=2,
            )
            return value
        raise TypeError("an integer is required (got type %s)" % type(value).__name__)
