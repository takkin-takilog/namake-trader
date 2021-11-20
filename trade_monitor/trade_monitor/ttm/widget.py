import os
import pandas as pd
from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtGui import QColor, QPen
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCharts import QtCharts
from trade_monitor import utility as utl
from trade_monitor.constant import FMT_QT_TIME, FMT_TIME_HM
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis
from trade_monitor.widget_base import CalloutDataTime
from trade_monitor.widget_base import LineChartViewDateTimeAxis
from trade_monitor.ttm.constant import ColumnName, GapType, DataType
from trade_monitor.ttm.constant import ChartTag
from trade_monitor.ttm.histogram_ui import ColumnName as HistColumnName
from trade_monitor.ttm.histogram_ui import HistogramUi


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

    def _generate_statistics_dataframe(self,
                                       df_base: pd.DataFrame,
                                       level: list
                                       ) -> pd.DataFrame:

        # ----- generate DataFrame "Mean" -----
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

        # ----- generate DataFrame "Std" -----
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

        # ----- generate DataFrame "Cumulative Sum" -----
        cond = df_mean.index.get_level_values(ColumnName.DATA_TYP.value) == DataType.CO_MEAN.value
        df_csum = df_mean[cond].rename(index={DataType.CO_MEAN.value: DataType.CO_CSUM.value},
                                       level=ColumnName.DATA_TYP.value)
        df_csum = df_csum.cumsum(axis=1)

        # concat "df_mean" and "df_std" and "df_csum"
        return pd.concat([df_mean, df_std, df_csum]).sort_index()


class CandlestickChartView(CandlestickChartViewDateTimeAxis):

    def __init__(self, parent=None):
        super().__init__(parent)

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

        self.chart().setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        self.set_callout_dt_format("hh:mm")

        self._is_update = False

    def update(self,
               df: pd.DataFrame,
               gran_param: GranParam,
               inst_param: InstParam):
        super().update(df, gran_param, inst_param)

        max_x = utl.convert_to_qdatetime(df.index[-1])

        dt_ = df.index[0]
        min_x = utl.convert_to_qdatetime(dt_)

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

        if self._is_update:
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


class BaseLineChartViewTtm(LineChartViewDateTimeAxis):

    def __init__(self, config_tbl, parent=None):
        super().__init__(config_tbl, parent)

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

        self.set_callout_dt_format("hh:mm")

        self._QDT_BASE = QDate(2010, 1, 1)
        self._QDTTM_TTM = QDateTime(self._QDT_BASE, QTime(9, 55))

        self._is_update = False

    def clear(self):

        for _, row in self._config_df.iterrows():
            series = row[self._COL_SERIES]
            series.clear()

        self.chart().axisY().setRange(self._min_y, self._max_y)

    def update(self,
               df: pd.DataFrame,
               inst_param: InstParam,
               gran_param: GranParam):
        super().update(df, inst_param, gran_param)

        qtm = QTime.fromString(df.index[-1], FMT_QT_TIME)
        max_x = QDateTime(self._QDT_BASE, qtm)

        qtm = QTime.fromString(df.index[0], FMT_QT_TIME)
        min_x = QDateTime(self._QDT_BASE, qtm)

        self.chart().axisX().setRange(min_x, max_x)

        self._update_callout_ttm(self._QDTTM_TTM)
        self._df_date = pd.DataFrame()
        self._is_update = True

    def set_dataframe_date(self, df_date):
        self._df_date = df_date

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout_ttm(self._QDTTM_TTM)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        if self._is_update:
            self._vl_ttm.show()
            self._callout_ttm_dt.show()
            self._hl_zero.show()

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

    def __init__(self, tag: ChartTag, parent=None):

        pen_ho_m = QPen()
        pen_ho_m.setColor(Qt.magenta)
        pen_ho_m.setWidth(2)
        pen_ho_m.setStyle(Qt.SolidLine)

        pen_lo_m = QPen()
        pen_lo_m.setColor(Qt.cyan)
        pen_lo_m.setWidth(2)
        pen_lo_m.setStyle(Qt.SolidLine)

        pen_co_m = QPen()
        pen_co_m.setColor(Qt.green)
        pen_co_m.setWidth(2)
        pen_co_m.setStyle(Qt.SolidLine)

        pen_co_s = QPen()
        pen_co_s.setColor(Qt.green)
        pen_co_s.setWidth(1)
        pen_co_s.setStyle(Qt.DashLine)

        config_tbl = [
            [DataType.HO_MEAN.value, pen_ho_m, "mean_high-open"],
            # [DataType.HO_STD.value, Qt.blue],
            [DataType.LO_MEAN.value, pen_lo_m, "mean_low-open"],
            # [DataType.LO_STD.value, Qt.green],
            [DataType.CO_MEAN.value, pen_co_m, "mean_close-open"],
            [DataType.CO_STD.value, pen_co_s, "std_close-open"]
        ]

        super().__init__(config_tbl, parent)

        self._hist_ui = HistogramUi(tag)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

        if self._is_update:
            chart = self.chart()
            flag = chart.plotArea().contains(event.pos())
            if flag and (event.buttons() == Qt.LeftButton):
                m2v = chart.mapToValue(event.pos())
                xpos = utl.roundi(m2v.x())
                pdt = QDateTime.fromMSecsSinceEpoch(xpos).toPython()
                pdt = pd.to_datetime(pdt).round(self._gran_param.freq)
                stm = pdt.strftime(FMT_TIME_HM)
                df = self._df_date[stm]
                sr_ho = df.xs(GapType.HO.value, level=ColumnName.GAP_TYP.value)
                sr_ho.rename(HistColumnName.PRICE_HIOP.value, inplace=True)
                sr_lo = df.xs(GapType.LO.value, level=ColumnName.GAP_TYP.value)
                sr_lo.rename(HistColumnName.PRICE_LOOP.value, inplace=True)
                sr_co = df.xs(GapType.CO.value, level=ColumnName.GAP_TYP.value)
                sr_co.rename(HistColumnName.PRICE_CLOP.value, inplace=True)
                df_hcl = pd.concat([sr_ho, sr_co, sr_lo], axis=1)

                self._hist_ui.set_data(df_hcl, self._inst_param)
                self._hist_ui.setWindowTitle("Time Axis Analysis".format(stm))
                self._hist_ui.set_tag_text(stm)
                self._hist_ui.show()


class LineChartViewCumsum(BaseLineChartViewTtm):

    def __init__(self, parent=None):

        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(2)
        pen.setStyle(Qt.SolidLine)

        config_tbl = [
            [DataType.CO_CSUM.value, pen, "cumsum_close-open"]
        ]

        super().__init__(config_tbl, parent)
