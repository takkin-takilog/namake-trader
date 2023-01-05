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

        now = dt.datetime.now().replace(microsecond=0)

        if hour is not None:
            self._hour = self._check_hour_fields(hour)
            if minute is None:
                self._minute = 0
            else:
                self._minute = self._check_minute_fields(minute)
            self._add_timedelta = dt.timedelta(days=1)
            self._th_datetime = now.replace(
                hour=self._hour, minute=self._minute, second=self._second
            )
        elif minute is not None:
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

    def _check_hour_fields(self, hour: int) -> int:
        if not 0 <= hour <= 23:
            raise ValueError("hour must be in 0..23", hour)
        return hour

    def _check_minute_fields(self, minute: int) -> int:
        if not 0 <= minute <= 59:
            raise ValueError("minute must be in 0..59", minute)
        return minute

    def _check_second_fields(self, second: int) -> int:
        if not 0 <= second <= 59:
            raise ValueError("second must be in 0..59", second)
        return second
