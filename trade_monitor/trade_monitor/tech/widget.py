import os
import pandas as pd
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

        # self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self._init_chart(data_list)

        self.logger = ros_com.get_logger()

    def update(self,
               df: pd.DataFrame,
               inst_param: InstParam):
        super().update(df, inst_param)

        for data_type, row in self._df_chart.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.clear()
            pdsr = df[data_type]

            for idx, value in enumerate(pdsr):
                series.append(idx, value)

    def resizeEvent(self, event):
        super().resizeEvent(event)

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
