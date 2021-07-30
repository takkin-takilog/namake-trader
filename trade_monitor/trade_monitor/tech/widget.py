import os
import pandas as pd
from typing import List
from PySide2.QtCore import Qt, QPointF, QLineF, QRectF
from PySide2.QtGui import QColor, QPen, QBrush
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCharts import QtCharts
from trade_monitor import ros_common as ros_com
from trade_monitor.constant import InstParam
from trade_monitor.widget_base import CandlestickChartViewBarCategoryAxis
from trade_monitor.widget_base import CalloutDataTime
from trade_monitor.widget_base import LineChartViewBarCategoryAxis
from trade_monitor.tech.constant import ColTrnd
from trade_monitor.tech.constant import ColLine
from trade_monitor.constant import QtColor


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

        config_tbl = []

        # ==================== SMA ====================
        # --------------- Long ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.SMA_L.value, pen, QtCharts.QLineSeries()])

        # --------------- Middle ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.SMA_M.value, pen, QtCharts.QLineSeries()])

        # --------------- Short ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.SMA_S.value, pen, QtCharts.QLineSeries()])

        # ==================== EMA ====================
        # --------------- Long ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(2)
        pen.setStyle(Qt.DashLine)
        config_tbl.append([ColTrnd.EMA_L.value, pen, QtCharts.QLineSeries()])

        # --------------- Middle ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(2)
        pen.setStyle(Qt.DashLine)
        config_tbl.append([ColTrnd.EMA_M.value, pen, QtCharts.QLineSeries()])

        # --------------- Short ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(2)
        pen.setStyle(Qt.DashLine)
        config_tbl.append([ColTrnd.EMA_S.value, pen, QtCharts.QLineSeries()])

        # ==================== WMA ====================
        # --------------- Long ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(2)
        pen.setStyle(Qt.DashDotLine)
        config_tbl.append([ColTrnd.WMA_L.value, pen, QtCharts.QLineSeries()])

        # --------------- Middle ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(2)
        pen.setStyle(Qt.DashDotLine)
        config_tbl.append([ColTrnd.WMA_M.value, pen, QtCharts.QLineSeries()])

        # --------------- Short ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(2)
        pen.setStyle(Qt.DashDotLine)
        config_tbl.append([ColTrnd.WMA_S.value, pen, QtCharts.QLineSeries()])

        # ==================== Ichimoku Kinko ====================
        # --------------- Base ---------------
        pen = QPen()
        pen.setColor(QtColor.DARKVIOLET.value)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.ICHMK_BASE.value, pen, QtCharts.QLineSeries()])

        # --------------- Conv ---------------
        pen = QPen()
        pen.setColor(Qt.green)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.ICHMK_CONV.value, pen, QtCharts.QLineSeries()])

        # --------------- Span A ---------------
        pen = QPen()
        pen.setColor(QtColor.ORANGERED.value)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.ICHMK_SPNA.value, pen, QtCharts.QLineSeries()])

        # --------------- Span B ---------------
        pen = QPen()
        pen.setColor(QtColor.CRIMSON.value)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.ICHMK_SPNB.value, pen, QtCharts.QLineSeries()])

        # --------------- Lag ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.ICHMK_LAG.value, pen, QtCharts.QLineSeries()])

        # ==================== Bollinger bands ====================
        # --------------- Base ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_BASE.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * +1 ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_PS1.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * +2 ---------------
        pen = QPen()
        pen.setColor(QtColor.ROYALBLUE.value)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_PS2.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * +3 ---------------
        pen = QPen()
        pen.setColor(QtColor.DARKVIOLET.value)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_PS3.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * -1 ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_NS1.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * -2 ---------------
        pen = QPen()
        pen.setColor(QtColor.ROYALBLUE.value)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_NS2.value, pen, QtCharts.QLineSeries()])

        # --------------- sigma * -3 ---------------
        pen = QPen()
        pen.setColor(QtColor.DARKVIOLET.value)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        config_tbl.append([ColTrnd.BLNGR_NS3.value, pen, QtCharts.QLineSeries()])

        # self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self._init_config(config_tbl)
        self._trg_loc = None
        self.logger = ros_com.get_logger()

    def update(self,
               df: pd.DataFrame,
               trg_loc: int,
               inst_param: InstParam):
        super().update(df, inst_param)

        for data_type, row in self._df_chart.iterrows():
            series = row[ColLine.SERIES.value]
            series.clear()

            if data_type in df:
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
            series = row[ColLine.SERIES.value]
            series.clear()

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

    def _init_config(self, config_tbl):

        df_chart = pd.DataFrame(config_tbl,
                                columns=ColLine.to_list())
        df_chart.set_index(ColLine.DATA_TYP.value, inplace=True)

        # ---------- Attach X/Y Axis to series ----------
        axis_x = self.chart().axes(Qt.Horizontal)[0]
        axis_y = self.chart().axes(Qt.Vertical)[0]

        for _, row in df_chart.iterrows():
            series = row[ColLine.SERIES.value]
            series.setPen(row[ColLine.PEN.value])
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


class LineChartViewTech(LineChartViewBarCategoryAxis):

    def __init__(self, config_tbl: List, parent=None):
        super().__init__(config_tbl, parent)

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

        # ---------- Set Legend on chart ----------
        self.chart().legend().setVisible(True)

        self.chart().legend().detachFromChart()
        self.chart().legend().setBackgroundVisible(True)
        self.chart().legend().setBrush(QBrush(QColor(0, 0, 0, 0)))
        self.chart().legend().layout().setContentsMargins(10, 0, 0, 0)

        self._trg_loc = None

    def update(self,
               df: pd.DataFrame,
               trg_loc: int,
               inst_param: InstParam):
        super().update(df, inst_param)

        area = self.chart().plotArea()
        m2v = area.topLeft()
        self.chart().legend().setGeometry(QRectF(m2v.x(),
                                                 m2v.y(),
                                                 area.width(),
                                                 area.height()))
        self.chart().legend().update()

        self._trg_loc = trg_loc
        self._update_target_callout(self._trg_loc)

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

    def resizeEvent(self, event):
        super().resizeEvent(event)

        area = self.chart().plotArea()
        m2v = area.topLeft()
        self.chart().legend().setGeometry(QRectF(m2v.x(),
                                                 m2v.y(),
                                                 area.width(),
                                                 area.height()))
        self.chart().legend().update()

        if self._trg_loc is not None:
            self._update_target_callout(self._trg_loc)
