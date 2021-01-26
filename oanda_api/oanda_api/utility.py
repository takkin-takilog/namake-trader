import datetime as dt

FMT_YMDHMSF = "%Y-%m-%dT%H:%M:%S.%f"


def convert_datetime_jst(oanda_dt: str,
                         fmt: str = FMT_YMDHMSF
                         ) -> str:

    time_list = oanda_dt.split(".")
    dt_str = time_list[0] + "." + time_list[1][:6]
    dt_ = dt.datetime.strptime(dt_str, "%Y-%m-%dT%H:%M:%S.%f")
    return (dt_ + dt.timedelta(hours=9)).strftime(fmt)
