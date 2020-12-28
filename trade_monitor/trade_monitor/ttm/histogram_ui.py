import os
from enum import Enum, IntEnum
import pandas as pd
from PySide2.QtCore import QPointF, QLineF, QFile, Qt
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtUiTools import QUiLoader
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.widget_base import CandlestickChartViewBarCategoryAxis
from PySide2.QtCharts import QtCharts


class ColumnName(Enum):
    DATE = "date"
    PRICE_HIOP = "price hi-op"
    PRICE_LOOP = "price lo-op"
    PRICE_CLOP = "price cl-op"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ChartView(CandlestickChartViewBarCategoryAxis):

    def __init__(self, parent=None):
        super().__init__(parent)

        axis_x = self.chart().axes(Qt.Horizontal)[0]
        axis_x.setTitleVisible(False)

        # ---------- Set Candlestick color ----------
        self._series.setDecreasingColor(Qt.blue)
        self._series.setIncreasingColor(Qt.red)

        # ---------- Add Horizon Zero Line on scene ----------
        self._hl_zero = QGraphicsLineItem()
        pen = self._hl_zero.pen()
        pen.setColor(QColor(Qt.black))
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._hl_zero.setPen(pen)
        self._hl_zero.setZValue(1)
        self.scene().addItem(self._hl_zero)

    def update(self, df, digit):

        x_axis_label = []
        self._series.clear()
        for idx, sr in df.iterrows():
            op = 0.0
            hi = sr[ColumnName.PRICE_HIOP.value]
            lo = sr[ColumnName.PRICE_LOOP.value]
            cl = sr[ColumnName.PRICE_CLOP.value]
            x_axis_label.append(idx)
            cnd = QtCharts.QCandlestickSet(op, hi, lo, cl)
            self._series.append(cnd)

        chart = self.chart()
        chart.axisX(self._series).setCategories(x_axis_label)
        chart.axisY().setRange(self._min_y, self._max_y)
        # gran_info = GranInfo.get_member_by_msgid(gran_id)

        self.setRubberBand(QtCharts.QChartView.HorizontalRubberBand)

        self._digit = digit
        self._is_update = True
        # self._freq = gran_info.freq

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_callout()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        self._hl_zero.show()

    def _update_callout(self):

        chart = self.chart()

        # drow Horizontal Zreo Line
        zero_point = QPointF(0, 0)
        m2p = chart.mapToPosition(zero_point)
        plotAreaRect = chart.plotArea()
        self._hl_zero.setLine(QLineF(plotAreaRect.left(),
                                     m2p.y(),
                                     plotAreaRect.right(),
                                     m2p.y()))
        self._hl_zero.show()


class HistogramUi(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent, "histogram.ui")
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())
        self.setWindowTitle("TTM Histogram")

        # set TreeView
        pdtreeview = PandasTreeView(ui.widget_TreeView)
        header = pdtreeview.header()
        callback = self._on_view_header_sectionClicked
        header.sectionClicked.connect(callback)

        # set ChartView
        chartview = ChartView(ui.widget_ChartView)

        # self._logger = ros_com.get_logger()
        self._ui = ui
        self._pdtreeview = pdtreeview
        self._chartview = chartview

    def set_data(self, df: pd.DataFrame, digit):
        print("========== df ==========")
        print(df)

        print("========== df_csum ==========")
        """
        df_csum = df.cumsum()
        print(df_csum)
        """

        # set TreeView
        self._pdtreeview.set_dataframe(df)

        # set ChartView
        max_val = df[ColumnName.PRICE_HIOP.value].max()
        min_val = df[ColumnName.PRICE_LOOP.value].min()

        max_y = max(abs(max_val), abs(min_val))
        min_y = -max_y

        dif = max_y * 0.05
        self._chartview.set_max_y(max_y + dif)
        self._chartview.set_min_y(min_y - dif)

        self._chartview.update(df, digit)

    def _on_view_header_sectionClicked(self, logical_index):
        self._pdtreeview.show_header_menu(logical_index)

    def _load_ui(self, parent, ui_name: str):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), ui_name)
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui


if __name__ == "__main__":
    import sys
    from PySide2.QtCore import QCoreApplication
    from PySide2.QtWidgets import QApplication

    data = [["2020-10-12", 0.021, 0.000, 0.015],
            ["2020-10-19", 0.004, -0.021, -0.019],
            ["2020-10-26", 0.009, -0.008, -0.007],
            ["2020-11-02", 0.003, -0.012, 0.003],
            ["2020-11-09", 0.003, -0.015, -0.006],
            ["2020-11-16", 0.003, -0.014, -0.010],
            ["2020-11-23", 0.001, -0.004, -0.004],
            ["2020-12-07", 0.008, 0.000, 0.002],
            ["2020-12-14", 0.020, -0.006, 0.018],
            ["2020-12-21", 0.026, 0.000, 0.026]]
    df = pd.DataFrame(data,
                      columns=ColumnName.to_list())
    df.set_index(ColumnName.DATE.value, inplace=True)

    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = HistogramUi()
    widget.set_data(df, digit=3)
    widget.show()
    sys.exit(app.exec_())
