import os
import pandas as pd
import datetime as dt
from enum import Enum, IntEnum, auto
from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtGui import QColor, QPen
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCharts import QtCharts
from trade_monitor import ros_common as ros_com
from trade_monitor import utility as utl
from trade_monitor.constant import FMT_QT_TIME, FMT_TIME_HM
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.widget_base import CandlestickChartViewBarCategoryAxis
from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis
from trade_monitor.widget_base import CalloutDataTime
from trade_monitor.widget_base import BaseLineChartView
from trade_monitor.tech.constant import ColNameOhlc
from trade_monitor.tech.constant import ColNameLine


class CandlestickChartView(CandlestickChartViewBarCategoryAxis):

    def __init__(self, parent=None):
        super().__init__(parent)

        color = QColor(Qt.blue)
        # ---------- Add CalloutDataTime on scene ----------
        self._callout_trg_dt = CalloutDataTime(self.chart())
        self._callout_trg_dt.setBackgroundColor(color)
        self._callout_trg_dt.setZValue(0)
        self.scene().addItem(self._callout_trg_dt)

        # ---------- Add vertical line of CalloutDataTime on scene ----------
        self._trg_vl = QGraphicsLineItem()
        pen = self._trg_vl.pen()
        pen.setColor(color)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._trg_vl.setPen(pen)
        self._trg_vl.setZValue(1)
        self.scene().addItem(self._trg_vl)


        data_list = []

        # ==================== SMA ====================
        # --------------- Long ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.SMA_L.value, pen, QtCharts.QLineSeries()])

        # --------------- Middle ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.SMA_M.value, pen, QtCharts.QLineSeries()])

        # --------------- Short ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.SMA_S.value, pen, QtCharts.QLineSeries()])


        # ==================== Ichimoku Kinko ====================
        # --------------- Base ---------------
        pen = QPen()
        pen.setColor(Qt.black)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.ICHMK_BASE.value, pen, QtCharts.QLineSeries()])

        # --------------- Conv ---------------
        pen = QPen()
        pen.setColor(Qt.black)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.ICHMK_CONV.value, pen, QtCharts.QLineSeries()])

        # --------------- Span A ---------------
        pen = QPen()
        pen.setColor(Qt.black)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.ICHMK_SPNA.value, pen, QtCharts.QLineSeries()])

        # --------------- Span B ---------------
        pen = QPen()
        pen.setColor(Qt.black)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.ICHMK_SPNB.value, pen, QtCharts.QLineSeries()])

        # --------------- Lag ---------------
        pen = QPen()
        pen.setColor(Qt.black)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        data_list.append([ColNameOhlc.ICHMK_LAG.value, pen, QtCharts.QLineSeries()])

        # self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self._init_chart(data_list)
        self._trg_loc = None
        self.logger = ros_com.get_logger()

    def update(self,
               df: pd.DataFrame,
               trg_loc: int,
               inst_param: InstParam):
        super().update(df, inst_param)

        for data_type, row in self._df_chart.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.clear()
            pdsr = df[data_type]

            for idx, value in enumerate(pdsr):
                series.append(idx, value)

        self._trg_loc = trg_loc
        self._update_target_callout(self._trg_loc)

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._trg_loc is not None:
            self._update_target_callout(self._trg_loc)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

    def clear_line_chart(self):

        for _, row in self._df_chart.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.clear()

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

    def _init_chart(self, data_list):

        df_chart = pd.DataFrame(data_list,
                                 columns=ColNameLine.to_list())
        df_chart.set_index(ColNameLine.DATA_TYP.value, inplace=True)

        # ---------- Attach X/Y Axis to series ----------
        axis_x = self.chart().axes(Qt.Horizontal)[0]
        axis_y = self.chart().axes(Qt.Vertical)[0]

        for _, row in df_chart.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.setPen(row[ColNameLine.PEN.value])
            self.chart().addSeries(series)
            series.attachAxis(axis_x)
            series.attachAxis(axis_y)

        self._df_chart = df_chart

    def _update_target_callout(self, trg_loc: int):

        chart = self.chart()

        new_pos = QPointF(trg_loc, 0)
        m2p = chart.mapToPosition(new_pos)

        # drow Target Vertical Line
        plotAreaRect = chart.plotArea()
        self._trg_vl.setLine(QLineF(m2p.x(),
                                    plotAreaRect.top(),
                                    m2p.x(),
                                    plotAreaRect.bottom()))
        self._trg_vl.show()

        # drow Target Callout Datetime
        x_label_list = chart.axisX().categories()
        dtstr = x_label_list[trg_loc]
        self._callout_trg_dt.updateGeometry(dtstr, m2p)
        self._callout_trg_dt.show()
