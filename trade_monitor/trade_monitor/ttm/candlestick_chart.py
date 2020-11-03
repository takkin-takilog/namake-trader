from PySide2.QtCore import Qt, QDateTime, QDate, QTime
from trade_monitor.base import BaseCandlestickChart


class CandlestickChartTtm(BaseCandlestickChart):

    def __init__(self, widget):
        super().__init__(widget)

    def update(self, df, decimal_digit):
        super().update(df, decimal_digit)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(60 * 10)
        max_x = QDateTime(qd, qt)

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        min_x = QDateTime(qd, qt)
        dtstr = dt_.strftime("%Y/%m/%d (Fri)")

        self.chart().axisX().setTitleText(dtstr)
        self.chart().axisX().setRange(min_x, max_x)


