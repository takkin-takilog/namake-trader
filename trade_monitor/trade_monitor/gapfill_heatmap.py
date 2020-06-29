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


class ColorScaleChartView(QtCharts.QChartView):

    def __init__(self, parent, init_grad):
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

        self.__grad = init_grad

    def change_intensity_max_abs(self, intensity_max_abs: float):
          self.chart().axes(Qt.Vertical)[0].setRange(-intensity_max_abs, intensity_max_abs)

    def change_color_scale(self, grad: QLinearGradient):
        rect = self.chart().plotArea()
        grad.setStart(rect.topLeft())
        grad.setFinalStop(0, rect.bottom())
        self.chart().setPlotAreaBackgroundBrush(grad)
        self.__grad = grad

    def resizeEvent(self, event):
        super().resizeEvent(event)
        rect = self.chart().plotArea()
        #rect.setWidth(50)
        self.__grad.setStart(rect.topLeft())
        self.__grad.setFinalStop(0, rect.bottom())
        self.chart().setPlotAreaBackgroundBrush(self.__grad)
        print("6666666666666666666666666")
        print(rect)
        print(event.size().width())
        #self.chart().setPlotArea(rect)


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


def print_gradient(grad: QLinearGradient):

    grad.setSStart(0, 25.6)
    grad.setFinalStop(0, 25.6)
    # create image and fill it with gradient
    image = QImage(1, 256, QImage.Format_RGB32)
    painter = QPainter(image)
    painter.fillRect(image.rect(), grad)
    painter.end()

    print("w:{}, h:{}" .format(image.width(), image.height()))
    for i in range(256):
        print("[{}]:{}" .format(i, image.pixelColor(0, i)))


    rgblist = image.colorTable()
    print(rgblist)


class HeatBlockSeries(QtCharts.QAreaSeries):

    def __init__(self,
                 upperSeries: QtCharts.QLineSeries,
                 lowerSeries: QtCharts.QLineSeries,
                 heat_value: float):
        super().__init__(upperSeries, lowerSeries)

        pen = QPen(0x059605)
        pen.setWidth(3)
        self.setPen(pen)

        gradient = QLinearGradient(QPointF(0, 0), QPointF(0, 1))
        gradient.setColorAt(0.0, 0x3cc63c)
        gradient.setColorAt(1.0, 0x26f626)
        gradient.setCoordinateMode(QGradient.ObjectBoundingMode)
        self.setBrush(gradient)

        self.__upperSeries = upperSeries
        self.__lowerSeries = lowerSeries
        self.__heat_value = heat_value

    def update_heat_value(self, heat_value: float) -> None:
        self.__heat_value = heat_value

        # brush = QBrush()
        # self.setBrush(brush)

    def update_gradient(self, grad: QLinearGradient):

        """
        pmp = QPainter(QPixmap(1, 256))
        pmp.setBrush(QBrush(grad))
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(0, 0, 1, 256)
        """

        # create image and fill it with gradient
        image = QImage(1, 256, QImage.Format_RGB32)
        painter = QPainter(image)
        painter.fillRect(image.rect(), grad)
        rgblist = image.colorTable()
        print(rgblist)

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

        self.__intensity_max_abs = 0

    def set_gradient(self, gradient: QLinearGradient):
        pass

    def reset_map(self, df: pd.DataFrame):

        df_s = df.sort_values(df.index.name, ascending=True)

        max_val = df_s.max().max()
        min_val = df_s.min().min()
        self.__intensity_max_abs = max(abs(max_val), abs(min_val))

        print(df_s)
        print(df.columns)

        delta_y = df_s.index[1] - df_s.index[0]
        delta_x = df_s.columns[1] - df_s.columns[0]

        self.chart().removeAllSeries()

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

        self.chart().createDefaultAxes()
        #self.chart().axes(Qt.Horizontal)[0].setRange(0, 1)
        #self.chart().axes(Qt.Vertical)[0].setRange(0, 1)
        #chart.createDefaultAxes()
        #self.setChart(chart)

        return self.__intensity_max_abs

    def change_color_scale(self, grad: QLinearGradient):
        pass

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
        grBtoY = QLinearGradient(0, 0, 0, 100)
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        pm1 = QPixmap(24, 100)
        pmp = QPainter(pm1)
        pmp.setBrush(QBrush(grBtoY))
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(0, 0, 24, 100)
        pmp.end()
        ui.pushButton_gradientBtoYPB.setIcon(QIcon(pm1))
        ui.pushButton_gradientBtoYPB.setIconSize(QSize(24, 100))

        print_gradient(grBtoY)

        grGtoR = QLinearGradient(0, 0, 0, 100)
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        pm2 = QPixmap(24, 100)
        pmp = QPainter(pm2)
        pmp.setBrush(QBrush(grGtoR))
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(0, 0, 24, 100)
        pmp.end()
        ui.pushButton_gradientGtoRPB.setIcon(QIcon(pm2))
        ui.pushButton_gradientGtoRPB.setIconSize(QSize(24, 100))

        callback = self.__on_gradientBtoYPB_clicked
        ui.pushButton_gradientBtoYPB.clicked.connect(callback)

        callback = self.__on_gradientGtoRPB_clicked
        ui.pushButton_gradientGtoRPB.clicked.connect(callback)

        color_view = ColorScaleChartView(ui.widget_ColorScale, grBtoY)

        self.__ui = ui
        self.__chart_view = chart_view
        self.__color_view = color_view
        self.__grBtoY = grBtoY
        self.__grGtoR = grGtoR

    def __on_gradientBtoYPB_clicked(self):
        self.__chart_view.change_color_scale(self.__grBtoY)
        self.__color_view.change_color_scale(self.__grBtoY)

    def __on_gradientGtoRPB_clicked(self):
        self.__chart_view.change_color_scale(self.__grGtoR)
        self.__color_view.change_color_scale(self.__grGtoR)

    """
    def set_data(self, df: pd.DataFrame):
        self.__hmap_chart.set_data(df)
    """

    def reset_map(self, df: pd.DataFrame):

        intensity_max_abs = self.__chart_view.reset_map(df)
        self.__color_view.change_intensity_max_abs(intensity_max_abs)


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

        df = gen_sample_dataframe()
        self.reset_map(df)


if __name__ == "__main__":

    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])

    widget = GapFillHeatMap()
    #widget.set_data(df)


    widget.show()
    widget.init_resize()

    sys.exit(app.exec_())
