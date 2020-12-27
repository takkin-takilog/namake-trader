import os
from enum import Enum, IntEnum
import pandas as pd
from PySide2.QtWidgets import QMainWindow, QSizePolicy
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile, Qt
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.widget_base import BaseCandlestickChartView2
from PySide2.QtCharts import QtCharts


class ColumnName(Enum):
    DATE = "date"
    PRICE_HIOP = "price hi-op"
    PRICE_LOOP = "price lo-op"
    PRICE_CLOP = "price cl-op"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ChartView(BaseCandlestickChartView2):

    def __init__(self, parent=None):
        super().__init__(parent)

    def update(self, df, digit):

        x_axis_label = []
        self._ser_cdl.clear()
        for idx, sr in df.iterrows():
            op = 0.0
            hi = sr[ColumnName.PRICE_HIOP.value]
            lo = sr[ColumnName.PRICE_LOOP.value]
            cl = sr[ColumnName.PRICE_CLOP.value]
            x_axis_label.append(idx)
            cnd = QtCharts.QCandlestickSet(op, hi, lo, cl)
            self._ser_cdl.append(cnd)

        chart = self.chart()
        chart.axisX(self._ser_cdl).setCategories(x_axis_label)
        chart.axisY().setRange(self._min_y, self._max_y)
        # gran_info = GranInfo.get_member_by_msgid(gran_id)

        self.setRubberBand(QtCharts.QChartView.HorizontalRubberBand)

        self._digit = digit
        # self._freq = gran_info.freq


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
        max_y = df[ColumnName.PRICE_HIOP.value].max()
        min_y = df[ColumnName.PRICE_LOOP.value].min()

        dif = (max_y - min_y) * 0.05
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
