from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QGraphicsLineItem
from trade_monitor.base import BaseCandlestickChart
from trade_monitor.base import CalloutDataTime
from trade_monitor.base import BaseLineChart
from trade_monitor import utilities as utl
from trade_monitor.ttm.ttm_common import (DATA_TYP_HO_MEAN,
                                          DATA_TYP_HO_STD,
                                          DATA_TYP_LO_MEAN,
                                          DATA_TYP_LO_STD,
                                          DATA_TYP_CO_MEAN,
                                          DATA_TYP_CO_STD,
                                          DATA_TYP_CO_CSUM
                                          )


class CandlestickChartTtm(BaseCandlestickChart):

    def __init__(self, widget):
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

    def __init__(self, widget):
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

        self._logger = utl.get_logger()
        self._is_update = False

    def update(self, df, gran_id, decimal_digit):

        self._logger.debug("--------------------------")
        self._logger.debug("{}".format(df))

        super().update(gran_id, decimal_digit)

        self._ser_line.clear()

        sr = df.loc[DATA_TYP_HO_MEAN]

        self._logger.debug("============================")
        self._logger.debug("{}".format(type(sr)))
        self._logger.debug("{}".format(sr))

        self._logger.debug("--- index ----------")
        self._logger.debug("{}".format(sr.index.to_list()))
        self._logger.debug("--- values ----------")
        self._logger.debug("{}".format(sr.values))

        for idx in sr.index:
            print("{}:{}".format(idx, sr[idx]))
            self._ser_line.append(idx, sr[idx])

        """
        for dt_, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            qd = QDate(dt_.year, dt_.month, dt_.day)
            qt = QTime(dt_.hour, dt_.minute)
            qdt = QDateTime(qd, qt)
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_,
                                           qdt.toMSecsSinceEpoch())
            self._ser_line.append(cnd)

        """


        """

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
        """

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
