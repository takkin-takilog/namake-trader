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
from PySide2.QtWidgets import QGraphicsDropShadowEffect
from PySide2.QtCore import Qt, QFile, QSizeF, QPointF, QRectF, QRect, QSize, QMargins
from PySide2.QtUiTools import QUiLoader
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QVector3D, QGuiApplication, QPixmap, QBrush, QIcon
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient, QGradient, QImage, QFontMetrics
from PySide2.QtCharts import QtCharts

from trade_monitor.abstract import CalloutChartAbs
from trade_monitor import util as utl
from trade_monitor.util import GradientManager
from trade_monitor.util import INST_MSG_LIST

import time

COL_NAME_DATE = "date"
COL_NAME_GPA_DIR = "gap dir"
COL_NAME_GPA_CLOSE_PRICE = "gap close price"
COL_NAME_GPA_OPEN_PRICE = "gap open price"
COL_NAME_GPA_PRICE_MID = "gap price(mid)"
COL_NAME_GPA_PRICE_REAL = "gap price(real)"
COL_NAME_VALID_FLAG = "valid flag"
COL_NAME_SUCCESS_FLAG = "success flag"
COL_NAME_GAP_FILLED_TIME = "gap filled time"
COL_NAME_MAX_OPEN_RANGE = "max open range"
COL_NAME_END_CLOSE_PRICE = "end close price"

COL_GPA_PRICE_TH = "gap price thresh"
pd.set_option('display.max_columns', 1000)
# pd.set_option('display.max_rows', 1000)

gradMng = GradientManager()


def gen_sample_gapdata():

    datalist = [
        ["2020-04-06", 0, 108.528, 108.382004, 0.145996, 0.096001, True, True, "7:00:00", 0.112999, 108.976997],
        ["2020-04-13", 0, 108.464996, 108.403, 0.061996, 0.011993, True, True, "6:50:00", 0.100006, 108.167],
        ["2020-04-27", 1, 107.503006, 107.574005, 0.070999, 0.020996, True, True, "7:00:00", 0.125999, 107.556999],
        ["2020-05-11", 0, 106.639999, 106.539993, 0.100006, 0.050003, True, True, "6:40:00", 0.133995, 106.950996],
        ["2020-05-18", 1, 107.074501, 107.114502, 0.040001, 0.002502, True, False, "", 0.228996, 107.178001],
        ["2020-06-01", 0, 107.820999, 107.720001, 0.100998, 0.087997, True, True, "9:10:00", 0.094002, 107.744003],
        ["2020-06-22", 0, 106.891006, 106.772995, 0.118011, 0.068008, True, True, "9:20:00", 0.099998, 106.864998],
        ["2020-06-29", 0, 107.218994, 107.160004, 0.05899, 0.008995, True, True, "8:50:00", 0.148003, 107.266998],
    ]
    columns = [
        COL_NAME_DATE,
        COL_NAME_GPA_DIR,
        COL_NAME_GPA_CLOSE_PRICE,
        COL_NAME_GPA_OPEN_PRICE,
        COL_NAME_GPA_PRICE_MID,
        COL_NAME_GPA_PRICE_REAL,
        COL_NAME_VALID_FLAG,
        COL_NAME_SUCCESS_FLAG,
        COL_NAME_GAP_FILLED_TIME,
        COL_NAME_MAX_OPEN_RANGE,
        COL_NAME_END_CLOSE_PRICE
    ]

    df = pd.DataFrame(datalist, columns=columns)
    df.set_index("date", inplace=True)

    inst_id = 0

    return df, inst_id


class Callout(CalloutChartAbs):

    def __init__(self, parent: QtCharts.QChart = None):
        super().__init__(parent)

    def setChart(self, parent: QtCharts.QChart):
        self._chart = parent

    def setText(self, text: str):

        metrics = QFontMetrics(self._chart.font())
        self._textRect = QRectF(metrics.boundingRect(QRect(0, 0, 0, 0),
                                                     Qt.AlignLeft,
                                                     text))
        dx = 5
        dy = 5
        self._textRect.translate(dx, dy)
        self.prepareGeometryChange()
        self._rect = self._textRect.adjusted(-dx, -dy, dx, dy)
        self._text = text

    def updateGeometry(self, point: QPointF):
        #print("--- updateGeometry ---")
        self.prepareGeometryChange()

        center = self._chart.plotArea().center()
        anchor = self._chart.mapToPosition(point)
        pos_x = anchor.x() - self._rect.width() / 2
        pos_y = anchor.y() - self._rect.height() / 2

        ofs_x = 100
        ofs_y = 100
        if center.x() < anchor.x():
            if center.y() < anchor.y():
                anchor.setX(pos_x - ofs_x)
                anchor.setY(pos_y - ofs_y)
            else:
                anchor.setX(pos_x - ofs_x)
                anchor.setY(pos_y + ofs_y)
        else:
            if center.y() < anchor.y():
                anchor.setX(pos_x + ofs_x)
                anchor.setY(pos_y - ofs_y)
            else:
                anchor.setX(pos_x + ofs_x)
                anchor.setY(pos_y + ofs_y)

        self.setPos(anchor)

        self._anchor = anchor

    def paint(self,
              painter: QPainter,
              option: QStyleOptionGraphicsItem,
              widget: QWidget):
        #print("--- paint ---")
        path = QPainterPath()
        path.addRoundedRect(self._rect, 1, 1)   # 丸みを帯びた長方形の角を規定

        # 枠を描写
        painter.setBrush(Qt.white)    # 図形の塗りつぶし
        painter.setPen(QPen(Qt.black))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(QColor(Qt.blue)))
        painter.drawText(self._textRect, self._text)


callout = Callout()
callout.setZValue(0)


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


class ColorMapLabel(QLabel):

    def __init__(self, parent):
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.__margin_topbottom = 0.05
        self.__margin_leftright = 0.3

        self.__max_abs = 100
        self.__div = 2

        self.__frame_size = QSize()

    def update_intensity_range(self):
        self.__max_abs = gradMng.intensityMax
        self.__update()

    def update_color_scale(self):
        self.__update()

    def resize(self, frame_size):
        super().resize(frame_size)
        self.__frame_size = frame_size
        self.__update()

    def __update(self):

        frame_size = self.__frame_size

        tmp = frame_size.height() * self.__margin_topbottom
        tb_margin = utl.roundi(tmp)

        tmp = frame_size.width() * self.__margin_leftright
        lr_margin = utl.roundi(tmp)

        height = frame_size.height() - (tb_margin * 2)
        width = frame_size.width() - (lr_margin * 2)
        # print("height: {}" .format(height))

        rect = QRectF(0, tb_margin, width, height)

        gradMng.setRect(rect)
        grad = gradMng.getGradient()

        pm = QPixmap(frame_size)
        pm.fill(Qt.transparent)
        pmp = QPainter(pm)
        pmp.setBrush(grad)
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(rect)

        stpl = utl.roundi(height / self.__div)
        stpi = utl.roundi(self.__max_abs * 2 / self.__div)
        for i in range(0, self.__div + 1, 1):
            y_pos = i * stpl + tb_margin
            y_val = -i * stpi + self.__max_abs
            # print(y_pos)
            pmp.setPen(Qt.lightGray)
            pmp.drawLine(0, y_pos, width, y_pos)
            pmp.setPen(Qt.black)
            pmp.drawText(width + 5, y_pos + 5, str(y_val))
        pmp.end()

        self.setPixmap(pm)


class HeatBlockSeries(QtCharts.QAreaSeries):

    RGB8_MAX = 255

    def __init__(self):

        upper_series = QtCharts.QLineSeries()
        lower_series = QtCharts.QLineSeries()
        upper_series.append(0, 0)
        upper_series.append(0, 0)
        lower_series.append(0, 0)
        lower_series.append(0, 0)

        super().__init__(upper_series, lower_series)

        callback = self.__on_hovered
        self.hovered.connect(callback)

        self.__upper_series = upper_series
        self.__lower_series = lower_series
        self.__intensity = 0

    def set_block(self,
                  left_x: int,
                  right_x: int,
                  upper_y: int,
                  lower_y: int,
                  intensity: int,
                  frame_color: QColor = None
                  ) -> None:

        self.upperSeries().replace(0, left_x, upper_y)
        self.upperSeries().replace(1, right_x, upper_y)
        self.lowerSeries().replace(0, left_x, lower_y)
        self.lowerSeries().replace(1, right_x, lower_y)

        color = gradMng.convertValueToColor(intensity)
        self.setColor(color)

        inv_r = self.RGB8_MAX - color.red()
        inv_g = self.RGB8_MAX - color.green()
        inv_b = self.RGB8_MAX - color.blue()

        if frame_color is not None:
            frame_width = 2
            pen = QPen(frame_color)
            pen.setWidth(frame_width)
            self.setPen(pen)
        else:
            pen = QPen(Qt.black)
            pen.setWidth(1)
            self.setPen(pen)

        self.__intensity = intensity
        self.__brush_color_off = color
        self.__brush_color_on = QColor(inv_r, inv_g, inv_b)
        self.__center = QPointF((right_x + left_x) / 2,
                                (upper_y + lower_y) / 2)
        self.__left_x = left_x
        self.__right_x = right_x
        self.__lower_y = lower_y
        self.__upper_y = upper_y

    def set_intensity(self, intensity: int):
        color = gradMng.convertValueToColor(intensity)
        self.setColor(color)
        self.__intensity = intensity

    def update_color(self):
        color = gradMng.convertValueToColor(self.__intensity)
        self.setColor(color)
        inv_r = self.RGB8_MAX - color.red()
        inv_g = self.RGB8_MAX - color.green()
        inv_b = self.RGB8_MAX - color.blue()

        self.__brush_color_off = color
        self.__brush_color_on = QColor(inv_r, inv_g, inv_b)

    def __on_hovered(self, point: QPointF, state: bool):
        if state:
            self.setColor(self.__brush_color_on)

            text = f"・Profit or Loss: {self.__intensity}\n"\
                f"・Gap Range th: {self.__left_x} - {self.__right_x}\n"\
                f"・Max Open Range Th: {self.__lower_y} - {self.__upper_y}"

            callout.setText(text)
            callout.updateGeometry(self.__center)
        else:
            self.setColor(self.__brush_color_off)


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

        self.chart().setTitle('Profit and Loss Heat Map')

        # X Axis Settings
        axis_x = QtCharts.QValueAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Loss cut Range Threshold [pips]")
        # axis_x.setFormat("h:mm")
        axis_x.setLabelsAngle(0)

        #axis_x.setRange(-10.0, 10.0)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(2)
        axis_y.setTitleText("Gap Range Threshold [pips]")
        # axis_y.setFormat("h:mm")
        axis_y.setLabelsAngle(0)
        #axis_y.setRange(-10.0, 10.0)

        self.chart().addAxis(axis_x, Qt.AlignBottom)
        self.chart().addAxis(axis_y, Qt.AlignLeft)

        self.__callout = Callout(self.chart())
        callout.setChart(self.chart())
        self.scene().addItem(callout)
        shadow = QGraphicsDropShadowEffect()
        shadow.setOffset(0, 4)
        shadow.setBlurRadius(8)
        callout.setGraphicsEffect(shadow)

        self.__sts_bar = sts_bar
        self.__thin_num = 0

    def rebuild_hmap(self, df: pd.DataFrame, thin_num: int):

        df_s = df.sort_values(df.index.name, ascending=True)

        df_t = self.__thin_out_map(df_s, thin_num)
        self.__draw_map(df_t)

        self.__thin_num = thin_num

    def update_hmap(self, df: pd.DataFrame):

        df_s = df.sort_values(df.index.name, ascending=True)

        df_t = self.__thin_out_map(df_s, self.__thin_num)
        self.__update_hmap(df_t)

    def update_color(self):
        for block in self.chart().series():
            block.update_color()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        # print("---------- HeatMapChartView:mouseMoveEvent ----------")
        # print("frameSize(this): {}" .format(self.frameSize()))
        # print("frameSize(parent): {}" .format(self.parent().frameSize()))
        flag1 = self.chart().plotArea().contains(event.pos())
        flag2 = 0 < len(self.chart().series())
        if (flag1 and flag2):
            callout.show()
        else:
            callout.hide()


    """
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        print("----- HeatMapChartView:mousePressEvent ------")
        if self.__is_zoom:
            self.__is_zoom = False
            fs = self.parent().frameSize()
        else:
            self.__is_zoom = True
            fs = self.parent().frameSize() / 5.0
        self.resize(fs)
    """

    def __thin_out_map(self, df: pd.DataFrame, thin_num: int):

        # print("---------- thin_out_map ----------")
        #print(df)
        #print(thin_num)

        if thin_num < 2:
            return df

        col_min = df.columns[0]
        col_max = df.columns[-1]
        row_min = df.index[0]
        row_max = df.index[-1]

        rng_y = list(range(((row_min - 1) // thin_num) * thin_num + thin_num,
                           row_max + thin_num,
                           thin_num))

        rng_x = list(range(((col_min - 1) // thin_num) * thin_num + thin_num,
                           col_max + thin_num,
                           thin_num))

        #print("rng_x: {}" .format(rng_x))
        #print("rng_y: {}" .format(rng_y))

        self.__sts_bar.set_label_text("[1/3]")
        self.__sts_bar.set_bar_range(0, len(rng_y) * len(rng_x))

        new_y_map = []
        cnt = 0
        for y in rng_y:
            str_y = y - thin_num + 1
            str_y = utl.limit(str_y, row_min, row_max)
            end_y = utl.limit(y, row_min, row_max) + 1
            y_rng = range(str_y,  end_y, 1)
            new_x_list = [y]
            for x in rng_x:
                str_x = x - thin_num + 1
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
        # print(df_new)

        return df_new

    def __draw_map(self, df: pd.DataFrame):

        # print("---------- draw_map ----------")
        # print(df)

        max_val = df.max().max()
        min_val = df.min().min()
        intensity_max_abs = max(abs(max_val), abs(min_val))

        gradMng.updateColorTable(intensity_max_abs)

        # decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        # lsb = math.pow(10, -decimal_digit)

        rows_list = [n for n in df.index.to_list()]
        cols_list = [n for n in df.columns.to_list()]

        # print("rows_list: {}" .format(rows_list))
        # print("cols_list: {}" .format(cols_list))

        delta_y = rows_list[1] - rows_list[0]
        delta_x = cols_list[1] - cols_list[0]

        chart = self.chart()
        diff = df.size - len(chart.series())
        # print("-------------- diff:{}" .format(diff))

        self.__sts_bar.set_label_text("[2/3]")
        time_start = time.time()
        if 0 < diff:
            # print("===== 0 < diff =====")
            self.__sts_bar.set_bar_range(0, diff)
            ax_h = chart.axes(Qt.Horizontal)[0]
            ax_v = chart.axes(Qt.Vertical)[0]
            for i in range(diff):
                block = HeatBlockSeries()
                chart.addSeries(block)
                block.attachAxis(ax_h)
                block.attachAxis(ax_v)
                self.__sts_bar.set_bar_value(i + 1)
        elif diff < 0:
            # print("===== diff < 0 =====")
            self.__sts_bar.set_bar_range(0, -diff)
            for i in range(-diff):
                srlist = chart.series()
                chart.removeSeries(srlist[-1])
                self.__sts_bar.set_bar_value(i + 1)

        elapsed_time = time.time() - time_start
        print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

        self.__sts_bar.set_label_text("[3/3]")
        self.__sts_bar.set_bar_range(0, df.size)
        itr = 0
        # mark_list = []
        blklist = chart.series()
        for upper_y, row in df.iterrows():
            lower_y = upper_y - delta_y
            for idx_num, x in enumerate(row):
                right_x = cols_list[idx_num]
                left_x = right_x - delta_x
                """
                if max_val <= x:
                    mark_list.append([left_x, right_x, upper_y, lower_y, x])
                else:
                """
                blklist[itr].set_block(left_x, right_x, upper_y, lower_y, x)
                itr += 1
                self.__sts_bar.set_bar_value(itr)

        """
        for m in mark_list:
            block = self.chart().series()[itr]
            block.set_block(m[0], m[1], m[2], m[3], m[4], Qt.white)
            itr += 1
            self.__sts_bar.set_bar_value(itr)
        """

        ax_h = chart.axes(Qt.Horizontal)[0]
        ax_h.setRange(cols_list[0] - delta_x, cols_list[-1])
        ax_v = chart.axes(Qt.Vertical)[0]
        ax_v.setRange(rows_list[0] - delta_y, rows_list[-1])

    def __update_hmap(self, df: pd.DataFrame):

        self.__sts_bar.set_label_text("[3/3]")
        self.__sts_bar.set_bar_range(0, df.size)
        itr = 0
        srlist = self.chart().series()
        for _, row in df.iterrows():
            for x in row:
                srlist[itr].set_intensity(x)
                itr += 1
                self.__sts_bar.set_bar_value(itr)


class GapFillHeatMap(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)

        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("Gap-Fill Heat Map")

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
        icon_w = 24
        icon_h = 100
        grGtoR = QLinearGradient()
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        icon = GradientManager.generateIcon(grGtoR, icon_w, icon_h)
        ui.pushButton_gradientGtoRPB.setIcon(icon)
        ui.pushButton_gradientGtoRPB.setIconSize(QSize(icon_w, icon_h))

        grBtoY = QLinearGradient()
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        icon = GradientManager.generateIcon(grBtoY, icon_w, icon_h)
        ui.pushButton_gradientBtoYPB.setIcon(icon)
        ui.pushButton_gradientBtoYPB.setIconSize(QSize(icon_w, icon_h))

        callback = self.__on_gradientBtoYPB_clicked
        ui.pushButton_gradientBtoYPB.clicked.connect(callback)

        callback = self.__on_gradientGtoRPB_clicked
        ui.pushButton_gradientGtoRPB.clicked.connect(callback)

        gradMng.setGradient(grBtoY)
        color_map = ColorMapLabel(ui.widget_ColorMap)

        callback = self.__on_pushButtonGenHMap_clicked
        ui.pushButton_genHMap.clicked.connect(callback)

        callback = self.__on_spinBoxThinOut_changed
        ui.spinBox_ThinOut.valueChanged.connect(callback)

        # ----- test -----
        callback = self.__on_pushButton_test_clicked
        ui.pushButton_test.clicked.connect(callback)

        callback = self.__on_spinBoxDateStep_changed
        ui.spinBox_DateStep.valueChanged.connect(callback)

        callback = self.__on_scrollBarDate_changed
        ui.scrollBar_Date.valueChanged.connect(callback)

        self.__ui = ui
        self.__chart_view = chart_view
        self.__color_map = color_map
        self.__inst_idx = 0

        self.__df_param = pd.DataFrame()
        self.__df_htbl = pd.DataFrame()

    def __on_spinBoxDateStep_changed(self, value):
        print("spinBoxDateStep_value: {}" .format(value))
        maxval = self.__ui.spinBox_DateStep.maximum()
        self.__ui.scrollBar_Date.setMaximum(maxval - value)
        self.__ui.scrollBar_Date.setValue(maxval - value)

    def __on_scrollBarDate_changed(self, value):
        print("scrollBarDate_value: {}" .format(value))
        if self.__ui.pushButton_AutoUpdate.isChecked():
            self.__update_hmap(self.__df_hmap)

    def __update_hmap(self, df_hmap: pd.DataFrame):
        df_hmap_new = self.__remake_df_hmap(df_hmap)
        self.__chart_view.update_hmap(df_hmap_new)

    def __remake_df_hmap(self, df_hmap: pd.DataFrame):

        df_hmap_new = df_hmap

        return df_hmap_new

    def __on_pushButtonGenHMap_clicked(self):
        # 以下はテストコード
        """
        print("--------------- start --------------------")
        val = self.__ui.spinBox_ThinOut.value()
        print("----- {} ----" .format(val))
        df, inst_idx = gen_sample_dataframe()
        print("--------------- df comp --------------------")
        """
        self.__ui.pushButton_AutoUpdate.setChecked(False)
        self.rebuild_hmap(self.__df_hmap)

        step_val = len(self.__df_param)
        self.__ui.spinBox_DateStep.setValue(step_val)
        self.__ui.spinBox_DateStep.setMaximum(step_val)
        self.__ui.scrollBar_Date.setMaximum(0)

    def __on_pushButton_test_clicked(self):
        df_param, inst_idx = gen_sample_gapdata()
        self.set_gapfill_param(df_param, inst_idx)

    def set_gapfill_param(self, df_param: pd.DataFrame, inst_idx):
        df_valid = df_param[df_param[COL_NAME_VALID_FLAG]]

        # print("---------- df_valid ----------")
        # print(df_valid)
        # print("---------- df_valid ----------")
        max_open_price_max = df_valid[COL_NAME_MAX_OPEN_RANGE].max()
        # print("---------- max_open_price_max: {}" .format(max_open_price_max))
        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        margin = 5
        max_open_pips_max = utl.roundi(max_open_price_max * lsb) + margin
        # print("---------- max_open_pips_max: {}" .format(max_open_pips_max))
        hmap_col = [COL_NAME_DATE] + list(range(1, max_open_pips_max + 1))
        # print("---------- hmap_col: {}" .format(hmap_col))

        roslist = []
        for date, is_succ, mop, gpr in zip(df_valid.index,
                                           df_valid[COL_NAME_SUCCESS_FLAG],
                                           df_valid[COL_NAME_MAX_OPEN_RANGE],
                                           df_valid[COL_NAME_GPA_PRICE_REAL]):
            if is_succ:
                max_open_pips = utl.roundi(mop * lsb)
            else:
                max_open_pips = max_open_pips_max

            row_left = list(range(-1, -max_open_pips-1, -1))
            gap_pips = utl.roundi(gpr * lsb)

            row_right = [gap_pips] * (max_open_pips_max - max_open_pips)
            roslist.append([date] + row_left + row_right)

        df_htbl = pd.DataFrame(roslist, columns=hmap_col)
        df_htbl.set_index(COL_NAME_DATE, inplace=True)
        df_hmap = self.__make_hmap(df_param, df_htbl, inst_idx)

        lenmax = max(df_hmap.shape)
        thinout = (lenmax // 100) + 1

        self.__df_param = df_param
        self.__df_htbl = df_htbl
        self.__inst_idx = inst_idx
        self.__df_hmap = df_hmap

        self.__ui.spinBox_ThinOut.setValue(thinout)

    def __make_hmap(self,
                    df_param: pd.DataFrame,
                    df_htbl: pd.DataFrame,
                    inst_idx: int
                    ):

        gap_price_real_max = df_param[COL_NAME_GPA_PRICE_REAL].max()
        # print("---------- gap_price_real_max: {}" .format(gap_price_real_max))
        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        margin = 5
        gap_pips_max = utl.roundi(gap_price_real_max * lsb) + margin
        # print("---------- gap_pips_max: {}" .format(gap_pips_max))

        # print("---------- column: {}" .format(df_htbl.columns))

        df_mst = pd.DataFrame()
        for date, htbl in df_htbl.iterrows():
            gap_price = df_param.loc[date][COL_NAME_GPA_PRICE_REAL]
            gap_pips = utl.roundi(gap_price * lsb)
            collist = [htbl.to_list()] * (gap_pips_max - gap_pips)
            # print("---------- collist: {}" .format(len(collist)))
            gpt_col = list(range(gap_pips + 1, gap_pips_max + 1))

            df = pd.DataFrame(collist,
                              columns=df_htbl.columns)
            df[COL_GPA_PRICE_TH] = gpt_col
            df[COL_NAME_DATE] = date
            df.set_index([COL_NAME_DATE, COL_GPA_PRICE_TH], inplace=True)
            # print("===== df =====")
            # print(df)
            df_mst = pd.concat([df_mst, df])
        # df_mst.sort_index(inplace=True)
        # print(df_mst)

        return df_mst.sum(level=COL_GPA_PRICE_TH).sort_index()

    def __on_gradientBtoYPB_clicked(self):
        grBtoY = QLinearGradient(0, 0, 0, 100)
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        gradMng.setGradient(grBtoY)

        self.__chart_view.update_color()
        self.__color_map.update_color_scale()

    def __on_gradientGtoRPB_clicked(self):
        grGtoR = QLinearGradient(0, 0, 0, 100)
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        gradMng.setGradient(grGtoR)

        self.__chart_view.update_color()
        self.__color_map.update_color_scale()

    def __on_spinBoxThinOut_changed(self, i):
        # print("========= __on_spinBoxThinOut_changed =========")
        self.__update_hmap_size_txt(self.__df_hmap, i)

    def __update_hmap_size_txt(self, df_hmap, thinout):
        row_size = df_hmap.shape[0] // thinout
        col_size = df_hmap.shape[1] // thinout
        txt = "行数：" + str(row_size) + "\n列数：" + str(col_size)
        self.__ui.label_Roughness.setText(txt)

    def rebuild_hmap(self, df: pd.DataFrame):

        thin_num = self.__ui.spinBox_ThinOut.value()
        self.__chart_view.rebuild_hmap(df, thin_num)
        self.__color_map.update_intensity_range()

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
        fs = self.__ui.widget_ColorMap.frameSize()
        self.__color_map.resize(fs)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        print("----- GapFillHeatMap:resizeEvent ------")
        fs = self.__ui.widget_HeatMap.frameSize()
        self.__chart_view.resize(fs)
        fs = self.__ui.widget_ColorMap.frameSize()
        self.__color_map.resize(fs)


if __name__ == "__main__":

    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])

    widget = GapFillHeatMap()
    widget.show()
    widget.init_resize()

    sys.exit(app.exec_())
