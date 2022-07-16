import os
import pandas as pd
from dataclasses import dataclass
from enum import Enum
from PySide2.QtWidgets import QMainWindow
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCore import Qt, QPointF, QLineF
from PySide2.QtGui import QColor, QPen
from PySide2.QtCharts import QtCharts
from PySide2.QtWidgets import QGraphicsLineItem
from ...constant import QtColor
from .constant import ColOhlcChart
from ...parameter import GranParam, InstParam
from ... import ros_common as ros_com
from ...widget_base import CandlestickChartViewBarCategoryAxis
from ...widget_base import CalloutDataTime, CallouPrice


@dataclass
class ChartInfo():
    """
    View chart info.
    """
    df_ohlc: pd.DataFrame
    entry_time_str: str
    entry_time_loc: int
    entry_price: float
    exit_time_str: str
    exit_time_loc: int
    exit_price: float


class ColNameLine(Enum):
    """
    Line Chart dataframe column name.
    """
    TARGET_LABEL = "target_label"
    PEN = "pen"
    SERIES = "series"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class BaseUi(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

    def _load_ui(self, parent, ui_name: str):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), ui_name)
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui


class CandlestickChartView(CandlestickChartViewBarCategoryAxis):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.logger = ros_com.get_logger()

        color_entry = QColor(Qt.blue)
        color_exit = QColor(Qt.red)

        # ---------- Add VerticalLine "EntryTime" on scene ----------
        self._vl_entry_time = QGraphicsLineItem()
        pen = self._vl_entry_time.pen()
        pen.setColor(color_entry)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._vl_entry_time.setPen(pen)
        self._vl_entry_time.setZValue(1)
        self.scene().addItem(self._vl_entry_time)

        # ---------- Add Callout "EntryTime" on scene ----------
        self._co_entry_time = CalloutDataTime(self.chart())
        self._co_entry_time.setBackgroundColor(color_entry)
        self._co_entry_time.setZValue(0)
        self.scene().addItem(self._co_entry_time)

        # ---------- Add HorizontalLine "EntryPrice" on scene ----------
        self._hl_entry_price = QGraphicsLineItem()
        pen = self._hl_entry_price.pen()
        pen.setColor(color_entry)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_entry_price.setPen(pen)
        self._hl_entry_price.setZValue(1)
        self.scene().addItem(self._hl_entry_price)

        # ---------- Add Callout "EntryPrice" on scene ----------
        self._co_entry_price = CallouPrice(self.chart())
        self._co_entry_price.setBackgroundColor(color_entry)
        self._co_entry_price.setZValue(0)
        self.scene().addItem(self._co_entry_price)

        # ---------- Add VerticalLine "ExitTime" on scene ----------
        self._vl_exit_time = QGraphicsLineItem()
        pen = self._vl_exit_time.pen()
        pen.setColor(color_exit)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._vl_exit_time.setPen(pen)
        self._vl_exit_time.setZValue(1)
        self.scene().addItem(self._vl_exit_time)

        # ---------- Add Callout "ExitTime" on scene ----------
        self._co_exit_time = CalloutDataTime(self.chart())
        self._co_exit_time.setBackgroundColor(color_exit)
        self._co_exit_time.setZValue(0)
        self.scene().addItem(self._co_exit_time)

        # ---------- Add HorizontalLine "ExitPrice" on scene ----------
        self._hl_exit_price = QGraphicsLineItem()
        pen = self._hl_exit_price.pen()
        pen.setColor(color_exit)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_exit_price.setPen(pen)
        self._hl_exit_price.setZValue(1)
        self.scene().addItem(self._hl_exit_price)

        # ---------- Add Callout "ExitPrice" on scene ----------
        self._co_exit_price = CallouPrice(self.chart())
        self._co_exit_price.setBackgroundColor(color_exit)
        self._co_exit_price.setZValue(0)
        self.scene().addItem(self._co_exit_price)

        # ==================== Parabolic ====================
        config_tbl = []
        # ---------- Add SAR(L) Line on scene ----------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(2)
        pen.setStyle(Qt.NoPen)
        series = QtCharts.QLineSeries()
        series.setPointsVisible(True)
        config_tbl.append([ColOhlcChart.SAR_L.value, pen, series])

        # ---------- Add SAR(S) Line on scene ----------
        pen = QPen()
        pen.setColor(QColor(QtColor.ROYALBLUE.value))
        pen.setWidth(2)
        pen.setStyle(Qt.NoPen)
        series = QtCharts.QLineSeries()
        series.setPointsVisible(True)
        config_tbl.append([ColOhlcChart.SAR_S.value, pen, series])

        df_conf = pd.DataFrame(config_tbl,
                               columns=ColNameLine.to_list())
        df_conf.set_index(ColNameLine.TARGET_LABEL.value, inplace=True)

        # ---------- Attach X/Y Axis to series ----------
        axis_x = self.chart().axes(Qt.Horizontal)[0]
        axis_y = self.chart().axes(Qt.Vertical)[0]

        for _, row in df_conf.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.setPen(row[ColNameLine.PEN.value])
            self.chart().addSeries(series)
            series.attachAxis(axis_x)
            series.attachAxis(axis_y)

        self._df_conf = df_conf

        self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)
        self._is_update = False

    def clear_line_chart(self):

        for _, row in self._df_conf.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.clear()

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

    def update(self,
               df: pd.DataFrame,
               chart_info: ChartInfo,
               gran_param: GranParam,
               inst_param: InstParam):
        super().update(df, inst_param)

        # max_x = utl.convert_to_qdatetime(df.index[-1])
        # min_x = utl.convert_to_qdatetime(df.index[0])

        # dtstr = dt_.strftime("%Y/%m/%d")
        # chart = self.chart()
        # chart.axisX().setTitleText(dtstr)
        # chart.axisX().setRange(min_x, max_x)

        # ---------- update Parabolic(SAR) Line ----------
        for target_label, row in self._df_conf.iterrows():
            series = row[ColNameLine.SERIES.value]
            series.clear()
            pdsr = df[target_label]
            for idx, value in enumerate(pdsr):
                series.append(idx, value)

        # ---------- update Callout ----------
        self._chart_info = chart_info
        self._update_callout_target_datetime()

        self._is_update = True

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout_target_datetime()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        if self._is_update:
            self._vl_entry_time.show()
            self._co_entry_time.show()
            self._hl_entry_price.show()
            self._co_entry_price.show()
            self._vl_exit_time.show()
            self._co_exit_time.show()
            self._hl_exit_price.show()
            self._co_exit_price.show()

    def _update_callout_target_datetime(self):

        chart = self.chart()

        # ---------- drow VerticalLine "EntryTime" ----------
        point = QPointF(self._chart_info.entry_time_loc, 0)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._vl_entry_time.setLine(QLineF(m2p.x(),
                                           plotAreaRect.top(),
                                           m2p.x(),
                                           plotAreaRect.bottom()))
        self._vl_entry_time.show()

        # ---------- drow Callout "EntryTime" ----------
        self._co_entry_time.updateGeometry(self._chart_info.entry_time_str, m2p)
        self._co_entry_time.show()

        # ---------- drow HorizontalLine "EntryPrice" ----------
        point = QPointF(0, self._chart_info.entry_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._hl_entry_price.setLine(QLineF(plotAreaRect.left(),
                                            m2p.y(),
                                            plotAreaRect.right(),
                                            m2p.y()))
        self._hl_entry_price.show()

        # ---------- drow Callout "EntryPrice" ----------
        self._co_entry_price.updateGeometry(str(self._chart_info.entry_price), m2p)
        self._co_entry_price.show()

        # ---------- drow VerticalLine "ExitTime" ----------
        point = QPointF(self._chart_info.exit_time_loc, 0)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._vl_exit_time.setLine(QLineF(m2p.x(),
                                          plotAreaRect.top(),
                                          m2p.x(),
                                          plotAreaRect.bottom()))
        self._vl_exit_time.show()

        # ---------- drow Callout "ExitTime" ----------
        self._co_exit_time.updateGeometry(self._chart_info.exit_time_str, m2p)
        self._co_exit_time.show()

        # ---------- drow HorizontalLine "ExitPrice" ----------
        point = QPointF(0, self._chart_info.exit_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._hl_exit_price.setLine(QLineF(plotAreaRect.left(),
                                           m2p.y(),
                                           plotAreaRect.right(),
                                           m2p.y()))
        self._hl_exit_price.show()

        # ---------- drow Callout "ExitPrice" ----------
        self._co_exit_price.updateGeometry(str(self._chart_info.exit_price), m2p)
        self._co_exit_price.show()
