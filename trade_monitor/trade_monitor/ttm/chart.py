from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QGraphicsLineItem
from trade_monitor.base import BaseCandlestickChart
from trade_monitor.base import CalloutDataTime
from trade_monitor.base import BaseLineChart
from trade_monitor import utilities as utl
from trade_monitor.utilities import FMT_QT_TIME
from trade_monitor.ttm.ttm_common import (DATA_TYP_HO_MEAN,
                                          DATA_TYP_HO_STD,
                                          DATA_TYP_LO_MEAN,
                                          DATA_TYP_LO_STD,
                                          DATA_TYP_CO_MEAN,
                                          DATA_TYP_CO_STD,
                                          DATA_TYP_CO_CSUM
                                          )


class CandlestickChartTtm(BaseCandlestickChart):

    def __init__(self, widget=None):
        super().__init__(widget)

        color = QColor(Qt.blue)

        # ---------- Add CurrentOpenPriceLine on scene ----------
        self._vl_ttm = QGraphicsLineItem()
        pen = self._vl_ttm.pen()
        pen.setColor(color)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._vl_ttm.setPen(pen)
        self._vl_ttm.setZValue(1)
        self.scene().addItem(self._vl_ttm)

        # ---------- Add CalloutDataTime on scene ----------
        self._callout_ttm_dt = CalloutDataTime(self.chart())
        self._callout_ttm_dt.setBackgroundColor(color)
        self._callout_ttm_dt.setZValue(0)
        self.scene().addItem(self._callout_ttm_dt)

        self._is_update = False

    def update(self, df, gran_id, decimal_digit):
        super().update(df, gran_id, decimal_digit)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        max_x = QDateTime(qd, qt)

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        min_x = QDateTime(qd, qt)

        dtstr = dt_.strftime("%Y/%m/%d")

        chart = self.chart()
        chart.axisX().setTitleText(dtstr)
        chart.axisX().setRange(min_x, max_x)

        qdttm = QDateTime(dt_.date(), QTime(9, 55))

        self._update_callout_ttm(qdttm)

        self._is_update = True
        self._qdttm = qdttm

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout_ttm(self._qdttm)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        self._vl_ttm.show()
        self._callout_ttm_dt.show()

    def _update_callout_ttm(self, qdttm):

        chart = self.chart()

        # drow Vertical TTM Line
        x = qdttm.toMSecsSinceEpoch()
        ttm_point = QPointF(x, 0)
        m2p = chart.mapToPosition(ttm_point)
        plotAreaRect = chart.plotArea()
        self._vl_ttm.setLine(QLineF(m2p.x(),
                                    plotAreaRect.top(),
                                    m2p.x(),
                                    plotAreaRect.bottom()))
        self._vl_ttm.show()

        # drow Callout TTM
        dtstr = qdttm.toString("hh:mm")
        self._callout_ttm_dt.updateGeometry(dtstr, m2p)
        self._callout_ttm_dt.show()


class LineChartTtm(BaseLineChart):

    def __init__(self, widget=None):
        super().__init__(widget)

        self._CALLOUT_DT_FMT = "hh:mm"

        color = QColor(Qt.blue)

        # ---------- Add CurrentOpenPriceLine on scene ----------
        self._vl_ttm = QGraphicsLineItem()
        pen = self._vl_ttm.pen()
        pen.setColor(color)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._vl_ttm.setPen(pen)
        self._vl_ttm.setZValue(1)
        self.scene().addItem(self._vl_ttm)

        # ---------- Add CalloutDataTime on scene ----------
        self._callout_ttm_dt = CalloutDataTime(self.chart())
        self._callout_ttm_dt.setBackgroundColor(color)
        self._callout_ttm_dt.setZValue(0)
        self.scene().addItem(self._callout_ttm_dt)

        self._QDT_BASE = QDate(2010, 1, 1)
        self._QDTTM_TTM = QDateTime(self._QDT_BASE, QTime(9, 55))

        self._logger = utl.get_logger()
        self._is_update = False

    def update(self, gran_id, decimal_digit):
        super().update(gran_id, decimal_digit)

        self._update_callout_ttm(self._QDTTM_TTM)
        self._is_update = True

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout_ttm(self._QDTTM_TTM)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        self._vl_ttm.show()
        self._callout_ttm_dt.show()

    def _update_callout_ttm(self, qdttm):

        chart = self.chart()

        # drow Vertical TTM Line
        x = qdttm.toMSecsSinceEpoch()
        ttm_point = QPointF(x, 0)
        m2p = chart.mapToPosition(ttm_point)
        plotAreaRect = chart.plotArea()
        self._vl_ttm.setLine(QLineF(m2p.x(),
                                    plotAreaRect.top(),
                                    m2p.x(),
                                    plotAreaRect.bottom()))
        self._vl_ttm.show()

        # drow Callout TTM
        dtstr = qdttm.toString("hh:mm")
        self._callout_ttm_dt.updateGeometry(dtstr, m2p)
        self._callout_ttm_dt.show()


class LineChartTtmStatistics(LineChartTtm):

    def __init__(self, widget=None):
        super().__init__(widget)

    def update(self, df, gran_id, decimal_digit):
        super().update(gran_id, decimal_digit)

        self._ser_line.clear()

        sr = df.loc[DATA_TYP_HO_MEAN]

        for idx in sr.index:
            qtm = QTime.fromString(idx, FMT_QT_TIME)
            qdttm = QDateTime(self._QDT_BASE, qtm)
            self._ser_line.append(qdttm.toMSecsSinceEpoch(), sr[idx])

        tm_ = sr.index[-1]
        qtm = QTime.fromString(tm_, FMT_QT_TIME)
        max_x = QDateTime(self._QDT_BASE, qtm)

        tm_ = sr.index[0]
        qtm = QTime.fromString(tm_, FMT_QT_TIME)
        min_x = QDateTime(self._QDT_BASE, qtm)

        self.chart().axisX().setRange(min_x, max_x)


class LineChartTtmCumsum(LineChartTtm):

    def __init__(self, widget=None):
        super().__init__(widget)

    def update(self, df, gran_id, decimal_digit):
        super().update(gran_id, decimal_digit)

        self._ser_line.clear()

        sr = df.loc[DATA_TYP_CO_CSUM]

        for idx in sr.index:
            qtm = QTime.fromString(idx, FMT_QT_TIME)
            qdttm = QDateTime(self._QDT_BASE, qtm)
            self._ser_line.append(qdttm.toMSecsSinceEpoch(), sr[idx])

        tm_ = sr.index[-1]
        qtm = QTime.fromString(tm_, FMT_QT_TIME)
        max_x = QDateTime(self._QDT_BASE, qtm)

        tm_ = sr.index[0]
        qtm = QTime.fromString(tm_, FMT_QT_TIME)
        min_x = QDateTime(self._QDT_BASE, qtm)

        self.chart().axisX().setRange(min_x, max_x)
