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
from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis
from trade_monitor.widget_base import CalloutDataTime
from trade_monitor.widget_base import BaseLineChartView
from trade_monitor.constant import FMT_QT_TIME, FMT_TIME_HM
from trade_monitor.ttm.constant import ColumnName, GapType, DataType


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

    def _make_statistics_dataframe(self,
                                   df_base: pd.DataFrame,
                                   level: list
                                   ) -> pd.DataFrame:

        # ----- make DataFrame "Mean" -----
        df_mean = df_base.mean(level=level).sort_index()
        df_mean.reset_index(ColumnName.GAP_TYP.value, inplace=True)

        df_mean[ColumnName.DATA_TYP.value] = 0
        cond = df_mean[ColumnName.GAP_TYP.value] == GapType.HO.value
        df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.HO_MEAN.value
        cond = df_mean[ColumnName.GAP_TYP.value] == GapType.LO.value
        df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.LO_MEAN.value
        cond = df_mean[ColumnName.GAP_TYP.value] == GapType.CO.value
        df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.CO_MEAN.value

        df_mean.drop(columns=ColumnName.GAP_TYP.value, inplace=True)
        index = ColumnName.DATA_TYP.value
        df_mean.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Std" -----
        df_std = df_base.std(level=level).sort_index()
        df_std.reset_index(ColumnName.GAP_TYP.value, inplace=True)

        df_std[ColumnName.DATA_TYP.value] = 0
        cond = df_std[ColumnName.GAP_TYP.value] == GapType.HO.value
        df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.HO_STD.value
        cond = df_std[ColumnName.GAP_TYP.value] == GapType.LO.value
        df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.LO_STD.value
        cond = df_std[ColumnName.GAP_TYP.value] == GapType.CO.value
        df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.CO_STD.value

        df_std.drop(columns=ColumnName.GAP_TYP.value, inplace=True)
        index = ColumnName.DATA_TYP.value
        df_std.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Cumulative Sum" -----
        cond = df_mean.index.get_level_values(ColumnName.DATA_TYP.value) == DataType.CO_MEAN.value
        df_csum = df_mean[cond].rename(index={DataType.CO_MEAN.value: DataType.CO_CSUM.value},
                                       level=ColumnName.DATA_TYP.value)
        df_csum = df_csum.cumsum(axis=1)

        # concat "df_mean" and "df_std" and "df_csum"
        return pd.concat([df_mean, df_std, df_csum]).sort_index()


class CandlestickChartView(CandlestickChartViewDateTimeAxis):

    def __init__(self, parent=None):
        super().__init__(parent)

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

        self._is_update = False

    def update(self, df, gran_id, digit):
        super().update(df, gran_id, digit)

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


class BaseLineChartViewTtm(BaseLineChartView):

    _COL_DATA_TYP = "DataType"
    _COL_PEN = "Pen"
    _COL_SERIES = "Series"

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

        # ---------- Add Horizon Zero Line on scene ----------
        self._hl_zero = QGraphicsLineItem()
        pen = self._hl_zero.pen()
        pen.setColor(QColor(Qt.black))
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_zero.setPen(pen)
        self._hl_zero.setZValue(1)
        self.scene().addItem(self._hl_zero)

        self._QDT_BASE = QDate(2010, 1, 1)
        self._QDTTM_TTM = QDateTime(self._QDT_BASE, QTime(9, 55))

        self._df_date = pd.DataFrame()
        self._is_update = False
        self._logger = ros_com.get_logger()

    def _init_chart(self, data_list):

        chart_tbl = pd.DataFrame(data_list,
                                 columns=[self._COL_DATA_TYP,
                                          self._COL_PEN,
                                          self._COL_SERIES])
        chart_tbl.set_index(self._COL_DATA_TYP, inplace=True)

        # ---------- Attach X/Y Axis to series ----------
        axis_x = self.chart().axes(Qt.Horizontal)[0]
        axis_y = self.chart().axes(Qt.Vertical)[0]

        for _, row in chart_tbl.iterrows():
            series = row[self._COL_SERIES]
            series.setPen(row[self._COL_PEN])
            self.chart().addSeries(series)
            series.attachAxis(axis_x)
            series.attachAxis(axis_y)

        self._chart_tbl = chart_tbl

    def clear(self):

        for _, row in self._chart_tbl.iterrows():
            series = row[self._COL_SERIES]
            series.clear()

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

    def update(self, df, gran_id, digit):
        super().update(gran_id, digit)

        for data_type, row in self._chart_tbl.iterrows():
            series = row[self._COL_SERIES]
            series.clear()
            pdsr = df.loc[data_type]

            if not pdsr.isnull().any():
                for idx in pdsr.index:
                    qtm = QTime.fromString(idx, FMT_QT_TIME)
                    qdttm = QDateTime(self._QDT_BASE, qtm)
                    series.append(qdttm.toMSecsSinceEpoch(), pdsr[idx])

        qtm = QTime.fromString(df.columns[-1], FMT_QT_TIME)
        max_x = QDateTime(self._QDT_BASE, qtm)

        qtm = QTime.fromString(df.columns[0], FMT_QT_TIME)
        min_x = QDateTime(self._QDT_BASE, qtm)

        self.chart().axisX().setRange(min_x, max_x)

        self._update_callout_ttm(self._QDTTM_TTM)
        self._is_update = True

    def set_dataframe_date(self, df_date):
        self._df_date = df_date

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout_ttm(self._QDTTM_TTM)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        self._vl_ttm.show()
        self._callout_ttm_dt.show()
        self._hl_zero.show()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

        chart = self.chart()
        flag = chart.plotArea().contains(event.pos())
        if flag and (event.buttons() == Qt.LeftButton):
            m2v = chart.mapToValue(event.pos())
            pdt = QDateTime.fromMSecsSinceEpoch(round(m2v.x())).toPython()
            pdt = pd.to_datetime(pdt).round(self._freq)
            stm = pdt.strftime(FMT_TIME_HM)

            self._logger.debug("============================================================")
            self._logger.debug("buttons:{}".format(event.buttons()))
            self._logger.debug("pdt:{}".format(pdt))
            self._logger.debug("type(pdt):{}".format(type(pdt)))
            self._logger.debug("stm:{}".format(stm))
            self._logger.debug("df:{}".format(self._df_date))
            self._logger.debug("df[{}]:{}".format(stm, self._df_date[stm]))

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

        # drow Horizontal Zreo Line
        x = qdttm.toMSecsSinceEpoch()
        ttm_point = QPointF(0, 0)
        m2p = chart.mapToPosition(ttm_point)
        plotAreaRect = chart.plotArea()
        self._hl_zero.setLine(QLineF(plotAreaRect.left(),
                                     m2p.y(),
                                     plotAreaRect.right(),
                                     m2p.y()))
        self._hl_zero.show()


class LineChartViewStats(BaseLineChartViewTtm):

    def __init__(self, parent=None):
        super().__init__(parent)

        pen_hl_m = QPen()
        pen_hl_m.setColor(Qt.green)
        pen_hl_m.setWidth(2)
        pen_hl_m.setStyle(Qt.SolidLine)

        pen_c_m = QPen()
        pen_c_m.setColor(Qt.magenta)
        pen_c_m.setWidth(3)
        pen_c_m.setStyle(Qt.SolidLine)

        pen_c_s = QPen()
        pen_c_s.setColor(Qt.red)
        pen_c_s.setWidth(1)
        pen_c_s.setStyle(Qt.DashLine)

        data_list = [
            [DataType.HO_MEAN.value, pen_hl_m, QtCharts.QLineSeries()],
            # [DataType.HO_STD.value, Qt.blue, QtCharts.QLineSeries()],
            [DataType.LO_MEAN.value, pen_hl_m, QtCharts.QLineSeries()],
            # [DataType.LO_STD.value, Qt.green, QtCharts.QLineSeries()],
            [DataType.CO_MEAN.value, pen_c_m, QtCharts.QLineSeries()],
            [DataType.CO_STD.value, pen_c_s, QtCharts.QLineSeries()]
        ]

        self._init_chart(data_list)


class LineChartViewCumsum(BaseLineChartViewTtm):

    def __init__(self, parent=None):
        super().__init__(parent)

        pen = QPen()
        pen.setColor(Qt.cyan)
        pen.setWidth(3)
        pen.setStyle(Qt.SolidLine)

        data_list = [
            [DataType.CO_CSUM.value, pen, QtCharts.QLineSeries()]
        ]

        self._init_chart(data_list)
