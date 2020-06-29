import sys
import os
import pandas as pd

from abc import ABCMeta, abstractmethod

from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QSizePolicy
from PySide2.QtWidgets import QGraphicsRectItem
from PySide2.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget
from PySide2.QtCore import Qt, QFile, QSizeF, QPointF, QRectF, QSize, QMargins
from PySide2.QtUiTools import QUiLoader
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QVector3D, QGuiApplication, QPixmap, QBrush, QIcon
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient, QGradient, QImage
from PySide2.QtCharts import QtCharts

from trade_monitor.util import GradientManager

gradMng = GradientManager()


class ColorScaleChartView(QtCharts.QChartView):

    def __init__(self, parent):
        super().__init__(parent)

        # Create Chart and set General Chart setting
        chart = QtCharts.QChart()
        chart.layout().setContentsMargins(0, 0, 0, 0)
        chart.setBackgroundRoundness(0)
        margin = chart.margins()
        margin.setLeft(0)
        margin.setRight(0)
        chart.setMargins(margin)

        # Chart Background
        """
        backgroundGradient = QLinearGradient(0, 0, 0, 400)
        backgroundGradient.setColorAt(0.0, QColor('#50a1dc'))
        backgroundGradient.setColorAt(1.0, QColor('#00a1de'))
        chart.setBackgroundBrush(backgroundGradient)
        """

        chart.setPlotAreaBackgroundVisible(True)
        chart.legend().setVisible(False)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(3)
        #axis_y.setTitleText("Value")
        # axis_y.setFormat("h:mm")
        axis_y.setLabelsAngle(0)
        axis_y.setRange(-1.0, 1.0)

        chart.addAxis(axis_y, Qt.AlignRight)

        self.setChart(chart)

    def update_intensity_range(self):
        max_abs = gradMng.intensityMax
        self.chart().axes(Qt.Vertical)[0].setRange(-max_abs, max_abs)

    def update_color_scale(self):
        rect = self.chart().plotArea()
        gradMng.setRect(rect)
        grad = gradMng.getGradient()
        """
        grad.setStart(rect.topLeft())
        grad.setFinalStop(0, rect.bottom())
        """
        self.chart().setPlotAreaBackgroundBrush(grad)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        rect = self.chart().plotArea()
        #rect.setWidth(50)
        gradMng.setRect(rect)
        grad = gradMng.getGradient()
        self.chart().setPlotAreaBackgroundBrush(grad)
        #self.chart().setPlotArea(rect)


def gen_sample_dataframe():

    import numpy as np

    xlist = [i / 10 for i in range(1, 101, 1)]
    ylist = [i / 10 for i in range(3, 101, 1)]

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

    df_s = df.sort_values(df.index.name, ascending=True)
    print(df.sort_values(df.index.name, ascending=False))

    return df

class HeatBlockSeries(QtCharts.QAreaSeries):

    def __init__(self,
                 upperSeries: QtCharts.QLineSeries,
                 lowerSeries: QtCharts.QLineSeries,
                 intensity: float):
        super().__init__(upperSeries, lowerSeries)

        pen = QPen(0x059605)
        pen.setWidth(3)
        self.setPen(pen)

        color = gradMng.convertValueToColor(intensity)
        self.setColor(color)

        self.__upperSeries = upperSeries
        self.__lowerSeries = lowerSeries
        self.__intensity = intensity

    def update_heat_value(self, intensity: float) -> None:
        self.__intensity = intensity

        # brush = QBrush()
        # self.setBrush(brush)

    def update_color(self):
        color = gradMng.convertValueToColor(self.__intensity)
        self.setColor(color)

        """
        pmp = QPainter(QPixmap(1, 256))
        pmp.setBrush(QBrush(grad))
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(0, 0, 1, 256)
        """

        # brush = QBrush()
        # self.setBrush(brush)


class HeatMapChartViewAbs(QtCharts.QChartView):
    __metaclass__ = ABCMeta

    def __init__(self, parent):
        super().__init__(parent)

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
        """
        plotAreaGradient = QLinearGradient(0, 100, 0, 400)
        plotAreaGradient.setColorAt(0.0, QColor('#f1f1f1'))
        plotAreaGradient.setColorAt(1.0, QColor('#ffffff'))
        chart.setPlotAreaBackgroundBrush(plotAreaGradient)
        chart.setPlotAreaBackgroundVisible(True)
        """

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

        # X Axis Settings
        axis_x = QtCharts.QValueAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Value")
        # axis_x.setFormat("h:mm")
        axis_x.setLabelsAngle(0)
        axis_x.setRange(-10.0, 10.0)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(2)
        axis_y.setTitleText("Value")
        # axis_y.setFormat("h:mm")
        axis_y.setLabelsAngle(0)
        axis_y.setRange(-10.0, 10.0)

        self.chart().addAxis(axis_x, Qt.AlignBottom)
        #self._series.attachAxis(axis_x)
        self.chart().addAxis(axis_y, Qt.AlignLeft)
        #self._series.attachAxis(axis_y)
        self.__block_list = []


    def reset_map(self, df: pd.DataFrame):

        df_s = df.sort_values(df.index.name, ascending=True)

        max_val = df_s.max().max()
        min_val = df_s.min().min()
        intensity_max_abs = max(abs(max_val), abs(min_val))

        print("--------------- updateColorTable start --------------------")
        gradMng.updateColorTable(intensity_max_abs)
        print("--------------- updateColorTable end --------------------")

        delta_y = df_s.index[1] - df_s.index[0]
        delta_x = df_s.columns[1] - df_s.columns[0]

        print("--------------- removeAllSeries start --------------------")
        self.chart().removeAllSeries()
        print("--------------- removeAllSeries end --------------------")

        self.__block_list = []
        self.chart().series().clear()
        for idx_y, row in df_s.iterrows():
            idx_y_pre = idx_y - delta_y
            for idx_num, x in enumerate(row):
                idx_x = df.columns[idx_num]
                idx_x_pre = idx_x - delta_x
                upper_series = QtCharts.QLineSeries()
                lower_series = QtCharts.QLineSeries()
                upper_series.append(idx_x_pre, idx_y)
                upper_series.append(idx_x, idx_y)
                lower_series.append(idx_x_pre, idx_y_pre)
                lower_series.append(idx_x, idx_y_pre)
                block = HeatBlockSeries(upper_series, lower_series, x)
                self.chart().addSeries(block)
                self.__block_list.append(block)
                print("x:{}, y:{}" .format(idx_num, idx_y))

        self.chart().createDefaultAxes()
        #self.chart().axes(Qt.Horizontal)[0].setRange(0, 1)
        #self.chart().axes(Qt.Vertical)[0].setRange(0, 1)
        #chart.createDefaultAxes()
        #self.setChart(chart)

    def update_color(self):
        for block in self.__block_list:
            block.update_color()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        print("---------- mouseMoveEvent ----------")

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

        chart_view = HeatMapChartView(ui.widget_HeatMap)

        # Color
        grGtoR = QLinearGradient()
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        icon = GradientManager.generateIcon(grGtoR, 24, 100)
        ui.pushButton_gradientGtoRPB.setIcon(icon)
        ui.pushButton_gradientGtoRPB.setIconSize(QSize(24, 100))

        grBtoY = QLinearGradient()
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        icon = GradientManager.generateIcon(grBtoY, 24, 100)
        ui.pushButton_gradientBtoYPB.setIcon(icon)
        ui.pushButton_gradientBtoYPB.setIconSize(QSize(24, 100))

        callback = self.__on_gradientBtoYPB_clicked
        ui.pushButton_gradientBtoYPB.clicked.connect(callback)

        callback = self.__on_gradientGtoRPB_clicked
        ui.pushButton_gradientGtoRPB.clicked.connect(callback)

        gradMng.setGradient(grBtoY)
        color_view = ColorScaleChartView(ui.widget_ColorScale)

        callback = self.__on_pushButton_clicked
        ui.pushButton.clicked.connect(callback)

        self.__ui = ui
        self.__chart_view = chart_view
        self.__color_view = color_view

    def __on_gradientBtoYPB_clicked(self):
        grBtoY = QLinearGradient(0, 0, 0, 100)
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        gradMng.setGradient(grBtoY)

        self.__chart_view.update_color()
        self.__color_view.update_color_scale()

    def __on_gradientGtoRPB_clicked(self):
        grGtoR = QLinearGradient(0, 0, 0, 100)
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        gradMng.setGradient(grGtoR)

        self.__chart_view.update_color()
        self.__color_view.update_color_scale()

    def __on_pushButton_clicked(self):
        # 以下はテストコード
        print("--------------- start --------------------")
        df = gen_sample_dataframe()
        print("--------------- df comp --------------------")
        self.reset_map(df)

    """
    def set_data(self, df: pd.DataFrame):
        self.__hmap_chart.set_data(df)
    """

    def reset_map(self, df: pd.DataFrame):
        self.__chart_view.reset_map(df)
        print("--------------- reset_map comp --------------------")
        self.__color_view.update_intensity_range()

    def __load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gapfill_heatmap.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        fs = self.__ui.widget_HeatMap.frameSize()
        self.__chart_view.resize(fs)
        fs = self.__ui.widget_ColorScale.frameSize()
        self.__color_view.resize(fs)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        print("------------------ resizeEvent --------------------")
        fs = self.__ui.widget_HeatMap.frameSize()
        self.__chart_view.resize(fs)
        fs = self.__ui.widget_ColorScale.frameSize()
        self.__color_view.resize(fs)


if __name__ == "__main__":

    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])

    widget = GapFillHeatMap()
    #widget.set_data(df)


    widget.show()
    widget.init_resize()

    sys.exit(app.exec_())
