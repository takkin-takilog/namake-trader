import sys
import os
import math
import pandas as pd
import numpy as np
from decimal import Decimal, ROUND_HALF_UP

from abc import ABCMeta, abstractmethod

from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QSizePolicy
from PySide2.QtWidgets import QGraphicsRectItem
from PySide2.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget
from PySide2.QtWidgets import QLabel, QProgressBar, QStatusBar
from PySide2.QtCore import Qt, QFile, QSizeF, QPointF, QRectF, QSize, QMargins
from PySide2.QtUiTools import QUiLoader
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QVector3D, QGuiApplication, QPixmap, QBrush, QIcon
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient, QGradient, QImage
from PySide2.QtCharts import QtCharts

from trade_monitor import util as utl
from trade_monitor.util import GradientManager
from trade_monitor.util import INST_MSG_LIST

gradMng = GradientManager()


def gen_sample_dataframe():

    xlist = [i for i in range(1, 50+1, 1)]
    ylist = [i for i in range(4, 50+1, 1)]

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
    #print(df.sort_values(df.index.name, ascending=False))

    inst_id = 0

    return df, inst_id


class StatusBar():

    def __init__(self, parent):

        sts_label = QLabel()
        sts_label.setVisible(False)

        prog_bar = QProgressBar()
        prog_bar.setVisible(False)
        prog_bar.setTextVisible(True)

        parent.addPermanentWidget(sts_label)
        parent.addPermanentWidget(prog_bar, 1)

        self.__sts_label = sts_label
        self.__prog_bar = prog_bar

    def set_label_text(self, text):
        self.__sts_label.setText(text)
        self.__sts_label.setVisible(True)

    def set_bar_range(self, minimum, maximum):
        self.__prog_bar.setVisible(True)
        self.__prog_bar.setRange(minimum, maximum)

    def set_bar_value(self, value):
        self.__prog_bar.setValue(value)


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


class HeatBlockSeries(QtCharts.QAreaSeries):

    def __init__(self):

        upper_series = QtCharts.QLineSeries()
        lower_series = QtCharts.QLineSeries()
        upper_series.append(0, 0)
        upper_series.append(0, 0)
        lower_series.append(0, 0)
        lower_series.append(0, 0)

        super().__init__(upper_series, lower_series)
        self.setPen(Qt.NoPen)

        self.__upper_series = upper_series
        self.__lower_series = lower_series
        self.__intensity = 0

    def update(self,
               left_x: float,
               right_x: float,
               upper_y: float,
               lower_y: float,
               intensity: float) -> None:

        self.upperSeries().replace(0, left_x, upper_y)
        self.upperSeries().replace(1, right_x, upper_y)
        self.lowerSeries().replace(0, left_x, lower_y)
        self.lowerSeries().replace(1, right_x, lower_y)

        color = gradMng.convertValueToColor(intensity)
        self.setColor(color)

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

    def __init__(self, parent, sts_bar):
        super().__init__(parent)

        self.chart().setTitle('Simple Area Chart')

        # X Axis Settings
        axis_x = QtCharts.QValueAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Value")
        # axis_x.setFormat("h:mm")
        axis_x.setLabelsAngle(0)
        #axis_x.setRange(-10.0, 10.0)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(2)
        axis_y.setTitleText("Value")
        # axis_y.setFormat("h:mm")
        axis_y.setLabelsAngle(0)
        #axis_y.setRange(-10.0, 10.0)

        self.chart().addAxis(axis_x, Qt.AlignBottom)
        self.chart().addAxis(axis_y, Qt.AlignLeft)

        self.__sts_bar = sts_bar

    def reset_map(self, df: pd.DataFrame, inst_id: int, thin_rang: int):

        df_s = df.sort_values(df.index.name, ascending=True)

        df_t = self.__thin_out_map(df_s, thin_rang)
        self.__draw_map(df_t, inst_id)

    def update_color(self):
        for block in self.chart().series():
            block.update_color()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        print("---------- mouseMoveEvent ----------")

    def __thin_out_map(self, df: pd.DataFrame, thin_rang: int):

        print("---------- thin_out_map ----------")
        print(df)
        print(thin_rang)

        if thin_rang < 2:
            return df

        col_min = df.columns[0]
        col_max = df.columns[-1]
        row_min = df.index[0]
        row_max = df.index[-1]

        rng_y = list(range(((row_min - 1) // thin_rang) * thin_rang + thin_rang,
                           row_max + thin_rang,
                           thin_rang))

        rng_x = list(range(((col_min - 1) // thin_rang) * thin_rang + thin_rang,
                           col_max + thin_rang,
                           thin_rang))

        print("rng_x: {}" .format(rng_x))
        print("rng_y: {}" .format(rng_y))

        self.__sts_bar.set_label_text("[1/3]")
        self.__sts_bar.set_bar_range(0, len(rng_y) * len(rng_x))

        new_y_map = []
        cnt = 0
        for y in rng_y:
            str_y = y - thin_rang + 1
            str_y = utl.limit(str_y, row_min, row_max)
            end_y = utl.limit(y, row_min, row_max) + 1
            y_rng = range(str_y,  end_y, 1)
            new_x_list = [y]
            for x in rng_x:
                str_x = x - thin_rang + 1
                str_x = utl.limit(str_x, col_min, col_max)
                end_x = utl.limit(x, col_min, col_max) + 1
                x_rng = range(str_x,  end_x, 1)
                new_x_list.append(df.loc[y_rng][x_rng].max().max())
                cnt = cnt + 1
                self.__sts_bar.set_bar_value(cnt)
            new_y_map.append(new_x_list)

        idx = "Y"
        columns = [idx] + list(rng_x)
        df_new = pd.DataFrame(new_y_map, columns=columns)
        df_new.set_index(idx, inplace=True)
        print(df_new)

        return df_new

    def __draw_map(self, df: pd.DataFrame, inst_id: int):

        print("---------- draw_map ----------")
        print(df)

        max_val = df.max().max()
        min_val = df.min().min()
        intensity_max_abs = max(abs(max_val), abs(min_val))

        gradMng.updateColorTable(intensity_max_abs)

        decimal_digit = INST_MSG_LIST[inst_id].decimal_digit
        lsb = math.pow(10, -decimal_digit)

        rows_list = [n * lsb for n in df.index.to_list()]
        cols_list = [n * lsb for n in df.columns.to_list()]

        print("rows_list: {}" .format(rows_list))
        print("cols_list: {}" .format(cols_list))

        delta_y = rows_list[1] - rows_list[0]
        delta_x = cols_list[1] - cols_list[0]

        diff = df.size - len(self.chart().series())

        self.__sts_bar.set_label_text("[2/3]")
        if 0 < diff:
            print("===== 0 < diff =====")
            self.__sts_bar.set_bar_range(0, diff)
            for i in range(diff):
                block = HeatBlockSeries()
                self.chart().addSeries(block)
                block.attachAxis(self.chart().axes(Qt.Horizontal)[0])
                block.attachAxis(self.chart().axes(Qt.Vertical)[0])
                self.__sts_bar.set_bar_value(i+1)
        elif diff < 0:
            print("===== diff < 0 =====")
            self.__sts_bar.set_bar_range(0, -diff)
            for i in range(-diff):
                sr = self.chart().series()[-1]
                self.chart().removeSeries(sr)
                self.__sts_bar.set_bar_value(i+1)

        self.__sts_bar.set_label_text("[3/3]")
        self.__sts_bar.set_bar_range(0, df.size)
        itr = 0
        for y, row in df.iterrows():
            upper_y = y * lsb
            lower_y = upper_y - delta_y
            for idx_num, x in enumerate(row):
                right_x = cols_list[idx_num]
                left_x = right_x - delta_x
                block = self.chart().series()[itr]
                block.update(left_x, right_x, upper_y, lower_y, x)
                itr = itr + 1
                self.__sts_bar.set_bar_value(itr)

        print("length: {}" .format(len(self.chart().series())))

        #self.chart().createDefaultAxes()
        self.chart().axes(Qt.Horizontal)[0].setRange(cols_list[0]-delta_x, cols_list[-1])
        self.chart().axes(Qt.Vertical)[0].setRange(rows_list[0]-delta_y, rows_list[-1])

    """
    def update(self):
        super().update()
    """


class GapFillHeatMap(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)

        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle('Qt DataVisualization 3D Bars')

        """
        # set status bar
        # sts_label = QLabel()
        sts_prog_bar = QProgressBar()
        # sts_label.setText("Status Label")
        sts_prog_bar.setTextVisible(True)
        # ui.statusbar.addPermanentWidget(sts_label)
        ui.statusbar.addPermanentWidget(sts_prog_bar, 1)
        """

        sts_bar = StatusBar(ui.statusbar)

        chart_view = HeatMapChartView(ui.widget_HeatMap,
                                      sts_bar)

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
        val = self.__ui.spinBox_ThinOut.value()
        print("----- {} ----" .format(val))
        df, inst_id = gen_sample_dataframe()
        print("--------------- df comp --------------------")
        self.reset_map(df, inst_id)

    """
    def set_data(self, df: pd.DataFrame):
        self.__hmap_chart.set_data(df)
    """

    def reset_map(self, df: pd.DataFrame, inst_id: int):

        thin_rang = self.__ui.spinBox_ThinOut.value()
        self.__chart_view.reset_map(df, inst_id, thin_rang)
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
