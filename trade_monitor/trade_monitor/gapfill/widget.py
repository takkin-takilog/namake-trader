from PySide2.QtCore import Qt, QDateTime, QTime, QPointF, QLineF
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtCharts import QtCharts
from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis
from trade_monitor import utility as utl


class BaseCandlestickChartViewGapFill(CandlestickChartViewDateTimeAxis):

    def __init__(self, parent):
        super().__init__(parent)

        # ---------- Add PreviousClosePriceLine on scene ----------
        self._hl_prev_cls = QGraphicsLineItem()
        pen = self._hl_prev_cls.pen()
        pen.setColor(Qt.magenta)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_prev_cls.setPen(pen)
        self.scene().addItem(self._hl_prev_cls)

        # ---------- Add CurrentOpenPriceLine on scene ----------
        self._hl_curr_opn = QGraphicsLineItem()
        pen = self._hl_curr_opn.pen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_curr_opn.setPen(pen)
        self.scene().addItem(self._hl_curr_opn)

        self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self.set_callout_dt_format("hh:mm")

        self._gap_close_price = 0
        self._gap_open_price = 0
        self._is_update = False

    def update(self, df, gran_id, gap_close_price, gap_open_price, digit):
        super().update(df, gran_id, digit)

        chart = self.chart()
        point = QPointF(0, gap_close_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._hl_prev_cls.setLine(QLineF(plotAreaRect.left(),
                                         m2p.y(),
                                         plotAreaRect.right(),
                                         m2p.y()))

        point = QPointF(0, gap_open_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._hl_curr_opn.setLine(QLineF(plotAreaRect.left(),
                                         m2p.y(),
                                         plotAreaRect.right(),
                                         m2p.y()))

        self._hl_prev_cls.show()
        self._hl_curr_opn.show()

        self._gap_close_price = gap_close_price
        self._gap_open_price = gap_open_price
        self._is_update = True

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            chart = self.chart()
            point = QPointF(0, self._gap_close_price)
            m2p = chart.mapToPosition(point)
            plotAreaRect = chart.plotArea()
            self._hl_prev_cls.setLine(QLineF(plotAreaRect.left(),
                                             m2p.y(),
                                             plotAreaRect.right(),
                                             m2p.y()))

            point = QPointF(0, self._gap_open_price)
            m2p = chart.mapToPosition(point)
            plotAreaRect = chart.plotArea()
            self._hl_curr_opn.setLine(QLineF(plotAreaRect.left(),
                                             m2p.y(),
                                             plotAreaRect.right(),
                                             m2p.y()))

            self._hl_prev_cls.show()
            self._hl_curr_opn.show()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        self._hl_prev_cls.show()
        self._hl_curr_opn.show()


class CandlestickChartViewPrev(BaseCandlestickChartViewGapFill):

    def __init__(self, parent):
        super().__init__(parent)

    def update(self,
               df,
               gran_id,
               gap_close_price,
               gap_open_price,
               digit):
        super().update(df, gran_id, gap_close_price, gap_open_price, digit)

        qdttm_x = utl.convert_to_qdatetime(df.index[-1])
        max_x = qdttm_x.addSecs(60 * 10)

        dt_ = df.index[0]
        min_x = utl.convert_to_qdatetime(dt_)

        dtstr = dt_.strftime("%Y/%m/%d (Fri)")

        self.chart().axisX().setTitleText(dtstr)
        self.chart().axisX().setRange(min_x, max_x)


class CandlestickChartViewCurr(BaseCandlestickChartViewGapFill):

    def __init__(self, parent):
        super().__init__(parent)

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
               gran_id,
               gap_close_price,
               gap_open_price,
               digit,
               end_time):
        super().update(df, gran_id, gap_close_price, gap_open_price, digit)

        max_x = utl.convert_to_qdatetime(df.index[-1])
        dt_ = df.index[0]
        qdttm = utl.convert_to_qdatetime(dt_)
        min_x = qdttm.addSecs(-60 * 10)
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
