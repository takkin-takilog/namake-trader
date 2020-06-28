import sys
import os
import pandas as pd

from abc import ABCMeta, abstractmethod

from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QSizePolicy
from PySide2.QtWidgets import QGraphicsRectItem
from PySide2.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget
from PySide2.QtCore import Qt, QFile, QSizeF, QPointF, QRectF
from PySide2.QtUiTools import QUiLoader
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QVector3D, QGuiApplication
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient, QGradient
from PySide2.QtCharts import QtCharts


class AreaSeries(QtCharts.QAreaSeries):

    def __init__(self,
                 upperSeries: QtCharts.QLineSeries,
                 lowerSeries: QtCharts.QLineSeries,
                 height: float):
        super().__init__(upperSeries, lowerSeries)

        pen = QPen(0x059605)
        pen.setWidth(3)
        self.setPen(pen)

        gradient = QLinearGradient(QPointF(0, 0), QPointF(0, 1))
        gradient.setColorAt(0.0, 0x3cc63c)
        gradient.setColorAt(1.0, 0x26f626)
        gradient.setCoordinateMode(QGradient.ObjectBoundingMode)
        self.setBrush(gradient)

        self.__height = height


class HeatMapChartViewAbs(QtCharts.QChartView):
    __metaclass__ = ABCMeta

    def __init__(self, parent):
        super().__init__()
        self.setParent(parent)

        # Create Chart and set General Chart setting
        chart = QtCharts.QChart()
        chart.layout().setContentsMargins(0, 0, 0, 0)
        chart.setBackgroundRoundness(0)

        # itle Font size
        """
        font = QFont("Sans Serif", )
        font.setPixelSize(18)
        chart.setTitleFont(font)
        """

        # chart.setTitle("Temperature in Celcius For Device:")
        #chart.setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        """
        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)
        chart.setPalette(palette)
        """

        # Chart Background
        """
        backgroundGradient = QLinearGradient(0, 0, 0, 400)
        backgroundGradient.setColorAt(0.0, QColor('#50a1dc'))
        backgroundGradient.setColorAt(1.0, QColor('#00a1de'))
        chart.setBackgroundBrush(backgroundGradient)
        """

        # Plot area background
        plotAreaGradient = QLinearGradient(0, 100, 0, 400)
        plotAreaGradient.setColorAt(0.0, QColor('#f1f1f1'))
        plotAreaGradient.setColorAt(1.0, QColor('#ffffff'))
        chart.setPlotAreaBackgroundBrush(plotAreaGradient)
        chart.setPlotAreaBackgroundVisible(True)

        chart.legend().hide()
        chart.legend().setVisible(False)

        self.setChart(chart)

        """
        self.setChart(chart)
        self.setParent(widget)
        self.resize(widget.frameSize())

        self._chart = chart
        self._widget = widget
        """

    """
    @abstractmethod
    def update(self):
        raise NotImplementedError()
    """


class HeatMapChartView(HeatMapChartViewAbs):

    def __init__(self, parent):
        super().__init__(parent)

        self.chart().setTitle('Simple Area Chart')


        """
        # X Axis Settings
        axis_x = QtCharts.QValueAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Value")
        # axis_x.setFormat("h:mm")
        # axis_x.setLabelsAngle(0)
        axis_x.setRange(0.0, 1.0)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(2)
        axis_y.setTitleText("Value")
        # axis_y.setFormat("h:mm")
        # axis_y.setLabelsAngle(0)
        axis_y.setRange(0.0, 1.0)

        #self._chart.addAxis(axis_x, Qt.AlignBottom)
        #self._series.attachAxis(axis_x)
        #self._chart.addAxis(axis_y, Qt.AlignLeft)
        #self._series.attachAxis(axis_y)
        """

    def test(self):
        upperSeries = QtCharts.QLineSeries()
        lowerSeries = QtCharts.QLineSeries()

        x = [x for x in range(1, 3, 1)]
        y1 = [10, 5]
        y2 = [1, 5]

        for i in range(len(x)):
            upperSeries.append(x[i], y1[i])
            lowerSeries.append(x[i], y2[i])

        series = QtCharts.QAreaSeries(upperSeries, lowerSeries)

        pen = QPen(Qt.red)
        pen.setWidth(3)
        series.setPen(pen)

        gradient = QLinearGradient(QPointF(0, 0), QPointF(0, 1))
        gradient.setColorAt(0.0, QColor(255, 255, 255))
        gradient.setColorAt(1.0, QColor(0, 255, 0))
        gradient.setCoordinateMode(QGradient.ObjectBoundingMode)
        series.setBrush(gradient)

        chart = QtCharts.QChart()
        chart.removeAllSeries()
        chart.addSeries(series)
        chart.createDefaultAxes()

        self.setChart(chart)

        """
        self.chart().removeAllSeries()
        self.chart().addSeries(series)
        self.chart().createDefaultAxes()
        """
        #self.chart().axes(Qt.Horizontal)[0].setRange(0, 3)
        #self.chart().axes(Qt.Vertical)[0].setRange(0, 10)

    def set_data(self, df: pd.DataFrame):

        df_s = df.sort_values(df.index.name, ascending=True)

        print("aaaaaaaaaaa")

        delta_y = df_s.index[1] - df_s.index[0]
        delta_x = df_s.columns[1] - df_s.columns[0]

        self.chart().series().clear()
        for idx, row in df_s.iterrows():
            idx_pre = idx - delta_y
            print("idx: " + str(idx))
            for x in row:
                point_pre = QPointF(x - delta_x, idx_pre)
                point_crr = QPointF(x, idx_pre)
                lower_series = QtCharts.QLineSeries()
                lower_series.append(point_pre)
                lower_series.append(point_crr)

                point_pre = QPointF(x - delta_x, idx)
                point_crr = QPointF(x, idx)
                upper_series = QtCharts.QLineSeries()
                upper_series.append(point_pre)
                upper_series.append(point_crr)

                #are_ser = AreaSeries(upper_series, lower_series, x)
                are_ser = QtCharts.QAreaSeries(upper_series, lower_series)
                #self.chart().addSeries(are_ser)

        upperSeries = QtCharts.QLineSeries()
        lowerSeries = QtCharts.QLineSeries()

        x = [x for x in range(1, 3, 1)]
        y1 = [5, 7]
        y2 = [3, 4]

        for i in range(len(x)):
            upperSeries.append(x[i], y1[i])
            lowerSeries.append(x[i], y2[i])

        series = QtCharts.QAreaSeries(upperSeries, lowerSeries)

        pen = QPen(Qt.red)
        pen.setWidth(3)
        series.setPen(pen)

        gradient = QLinearGradient(QPointF(0, 0), QPointF(0, 1))
        gradient.setColorAt(0.0, QColor(255, 255, 255))
        gradient.setColorAt(1.0, QColor(0, 255, 0))
        gradient.setCoordinateMode(QGradient.ObjectBoundingMode)
        series.setBrush(gradient)

        self._chart.addSeries(series)

        #series.attachAxis(self.__axis_x)
        #series.attachAxis(self.__axis_y)

        self._chart.setTitle('Simple Area Chart')
        self._chart.legend().hide()
        self._chart.createDefaultAxes()
        #self._chart.axes(Qt.Horizontal)[0].setRange(0, 3)
        #self._chart.axes(Qt.Vertical)[0].setRange(0, 10)

        #self.chart().addSeries(series)
        #self.chart().createDefaultAxes()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        print("---------- mouseMoveEvent ----------")
        self.test()
        print("--------------------------------------------------------")
    """
    def update(self):
        super().update()
    """


class GapFillHeatMap(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle('Qt DataVisualization 3D Bars')

        chart_view = HeatMapChartView(ui.widget)

        self.__ui = ui
        self.__chart_view = chart_view

    """
    def set_data(self, df: pd.DataFrame):
        self.__hmap_chart.set_data(df)
    """

    def __load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gapfill_heatmap.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        fs = self.__ui.widget.frameSize()
        #self.__chart_view.resize(fs)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        print("------------------ resizeEvent --------------------")
        fs = self.__ui.widget.frameSize()
        #self.__chart_view.resize(fs)

    def test(self):
        self.__chart_view.test()

def gen_sample_dataframe():

    import numpy as np

    xlist = [i / 10 for i in range(1, 11, 1)]
    ylist = [i / 10 for i in range(3, 11, 1)]

    map_ = []
    for y in ylist:
        row = []
        row.append(y)
        ran = np.random.randint(-100, 100, len(xlist)) / 100
        map_.append(row + list(ran))

    idx = "Y"
    columns = [idx] + xlist

    df = pd.DataFrame(map_, columns=columns)
    df.set_index(idx, inplace=True)

    df_s = df.sort_values(df.index.name, ascending=False)

    delta_y = df_s.index[1] - df_s.index[0]
    delta_x = df_s.columns[1] - df_s.columns[0]

    for idx, row in df_s.iterrows():
        idx_pre = idx - delta_y
        for x in row:
            point_pre = QPointF(x - delta_x, idx_pre)
            point_crr = QPointF(x, idx)
            series = QtCharts.QLineSeries()
            series.append(point_pre)
            series.append(point_crr)

    return df



if __name__ == "__main__":

    df = gen_sample_dataframe()

    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])

    widget = GapFillHeatMap()
    #widget.set_data(df)


    widget.show()
    widget.init_resize()

    sys.exit(app.exec_())
