import os
import pandas as pd
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


class CandlestickChartView(CandlestickChartViewBarCategoryAxis):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self.logger = ros_com.get_logger()

    def update(self,
               df: pd.DataFrame,
               inst_param: InstParam):
        super().update(df, inst_param)

        min_x = utl.convert_to_qdatetime(df.index[0])
        max_x = utl.convert_to_qdatetime(df.index[-1])

        chart = self.chart()
        # chart.axisX().setTitleText(dtstr)
        # chart.axisX().setRange(min_x, max_x)


    def resizeEvent(self, event):
        super().resizeEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

