from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtWidgets import QGraphicsLineItem
from trade_monitor.base import BaseCandlestickChart


class CandlestickChartGapFillPrev(BaseCandlestickChart):

    def __init__(self, widget):
        super().__init__(widget)

    def update(self,
               df,
               gap_close_price,
               gap_open_price,
               decimal_digit):
        super().update(df, gap_close_price, gap_open_price, decimal_digit)

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


class CandlestickChartGapFillCurr(BaseCandlestickChart):

    def __init__(self, widget):
        super().__init__(widget)

        # ---------- Add EndVerticalLine on scene ----------
        self._vl_endhour = QGraphicsLineItem()
        pen = self._vl_endhour.pen()
        pen.setColor(Qt.cyan)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._vl_endhour.setPen(pen)
        self.scene().addItem(self._vl_endhour)

        self._end_point = QPointF(0, 0)

    def update(self,
               df,
               gap_close_price,
               gap_open_price,
               decimal_digit,
               end_time):
        super().update(df, gap_close_price, gap_open_price, decimal_digit)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        max_x = QDateTime(qd, qt)

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(-60 * 10)
        min_x = QDateTime(qd, qt)
        dtstr = dt_.strftime("%Y/%m/%d (Mon)")

        self.chart().axisX().setTitleText(dtstr)
        self.chart().axisX().setRange(min_x, max_x)

        h = end_time.hour
        m = end_time.minute
        qdttm = QDateTime(dt_.date(), QTime(h, m))
        self._end_point.setX(qdttm.toMSecsSinceEpoch())
        self._update_end_hour()

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_end_hour()

    def _update_end_hour(self):

        m2p = self.chart().mapToPosition(self._end_point)
        plotAreaRect = self.chart().plotArea()
        self._vl_endhour.setLine(QLineF(m2p.x(),
                                        plotAreaRect.top(),
                                        m2p.x(),
                                        plotAreaRect.bottom()))
        self._vl_endhour.show()
