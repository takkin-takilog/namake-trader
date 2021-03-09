import sys
import os
import math
import pandas as pd

from PySide2.QtWidgets import QApplication, QMainWindow, QSizePolicy
from PySide2.QtWidgets import QStyleOptionGraphicsItem
from PySide2.QtWidgets import QLabel, QProgressBar, QWidget
from PySide2.QtWidgets import QGraphicsDropShadowEffect
from PySide2.QtCore import Qt, QFile, QPointF, QRectF, QRect, QSize
from PySide2.QtUiTools import QUiLoader
from PySide2.QtGui import QColor, QPen, QPainter, QPainterPath, QPixmap
from PySide2.QtGui import QLinearGradient, QFontMetrics
from PySide2.QtCharts import QtCharts

from trade_monitor.widget_base import BaseCalloutChart
from trade_monitor import utility as utl
from trade_monitor.utility import GradientManager
from trade_monitor.gapfill.heatmap import HeatMap

_g_gradMng = GradientManager()


class Callout(BaseCalloutChart):

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
        path = QPainterPath()
        path.addRoundedRect(self._rect, 1, 1)

        painter.setBrush(Qt.white)
        painter.setPen(QPen(Qt.black))
        painter.drawPath(path)

        painter.setPen(QPen(QColor(Qt.blue)))
        painter.drawText(self._textRect, self._text)


_g_callout = Callout()
_g_callout.setZValue(0)


class StatusBar():

    def __init__(self, parent):

        sts_label = QLabel()
        sts_label.setVisible(False)

        prog_bar = QProgressBar()
        prog_bar.setVisible(False)
        prog_bar.setTextVisible(True)

        parent.addPermanentWidget(sts_label)
        parent.addPermanentWidget(prog_bar, 1)

        self._sts_label = sts_label
        self._prog_bar = prog_bar

    def set_label_text(self, text):
        self._sts_label.setText(text)
        self._sts_label.setVisible(True)

    def set_bar_range(self, minimum, maximum):
        self._prog_bar.setVisible(True)
        self._prog_bar.setRange(minimum, maximum)

    def set_bar_value(self, value):
        self._prog_bar.setValue(value)


class ColorMapLabel(QLabel):

    def __init__(self, parent):
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._margin_topbottom = 0.05
        self._margin_leftright = 0.3

        self._max_abs = 100
        self._div = 2

        self._frame_size = QSize()

    def update_intensity_range(self):
        self._max_abs = _g_gradMng.intensityMax
        self._update()

    def update_color_scale(self):
        self._update()

    def resize(self, frame_size):
        super().resize(frame_size)
        self._frame_size = frame_size
        self._update()

    def _update(self):

        frame_size = self._frame_size

        tmp = frame_size.height() * self._margin_topbottom
        tb_margin = utl.roundi(tmp)

        tmp = frame_size.width() * self._margin_leftright
        lr_margin = utl.roundi(tmp)

        height = frame_size.height() - (tb_margin * 2)
        width = frame_size.width() - (lr_margin * 2)

        rect = QRectF(0, tb_margin, width, height)

        _g_gradMng.setRect(rect)
        grad = _g_gradMng.getGradient()

        pm = QPixmap(frame_size)
        pm.fill(Qt.transparent)
        pmp = QPainter(pm)
        pmp.setBrush(grad)
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(rect)

        stpl = utl.roundi(height / self._div)
        stpi = utl.roundi(self._max_abs * 2 / self._div)
        for i in range(0, self._div + 1, 1):
            y_pos = i * stpl + tb_margin
            y_val = -i * stpi + self._max_abs
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

        callback = self._on_hovered
        self.hovered.connect(callback)

        self._upper_series = upper_series
        self._lower_series = lower_series
        self._intensity = 0

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

        color = _g_gradMng.convertValueToColor(intensity)
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

        self._intensity = intensity
        self._brush_color_off = color
        self._brush_color_on = QColor(inv_r, inv_g, inv_b)
        self._center = QPointF((right_x + left_x) / 2,
                               (upper_y + lower_y) / 2)
        self._left_x = left_x
        self._right_x = right_x
        self._lower_y = lower_y
        self._upper_y = upper_y

    def set_intensity(self, intensity: int):
        self._update_color(intensity)
        self._intensity = intensity

    def update_color(self):
        self._update_color(self._intensity)

    def _update_color(self, intensity: int):
        color = _g_gradMng.convertValueToColor(intensity)
        self.setColor(color)
        inv_r = self.RGB8_MAX - color.red()
        inv_g = self.RGB8_MAX - color.green()
        inv_b = self.RGB8_MAX - color.blue()

        self._brush_color_off = color
        self._brush_color_on = QColor(inv_r, inv_g, inv_b)

    def _on_hovered(self, point: QPointF, state: bool):
        if state:
            self.setColor(self._brush_color_on)

            text = f"・Profit or Loss: {self._intensity}\n"\
                f"・Loss cut Range th: {self._left_x} - {self._right_x}\n"\
                f"・Gap Range Th: {self._lower_y} - {self._upper_y}"

            _g_callout.setText(text)
            _g_callout.updateGeometry(self._center)
        else:
            self.setColor(self._brush_color_off)


class HeatMapChartViewAbs(QtCharts.QChartView):

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


class HeatMapChartView(HeatMapChartViewAbs):

    def __init__(self, parent, sts_bar):
        super().__init__(parent)

        self.chart().setTitle('Profit and Loss Heat Map')

        # X Axis Settings
        axis_x = QtCharts.QValueAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Loss cut Range Threshold [pips]")
        axis_x.setLabelsAngle(0)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTickCount(2)
        axis_y.setTitleText("Gap Range Threshold [pips]")
        axis_y.setLabelsAngle(0)

        self.chart().addAxis(axis_x, Qt.AlignBottom)
        self.chart().addAxis(axis_y, Qt.AlignLeft)

        _g_callout.setChart(self.chart())
        self.scene().addItem(_g_callout)
        shadow = QGraphicsDropShadowEffect()
        shadow.setOffset(0, 4)
        shadow.setBlurRadius(8)
        _g_callout.setGraphicsEffect(shadow)

        self._sts_bar = sts_bar

    def rebuild_hmap(self, df_hmap: pd.DataFrame):

        self._draw_map(df_hmap)

    def update_hmap(self, df_hmap: pd.DataFrame):

        self._update_hmap(df_hmap)

    def update_color(self):
        for block in self.chart().series():
            block.update_color()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        flag1 = self.chart().plotArea().contains(event.pos())
        flag2 = 0 < len(self.chart().series())
        if (flag1 and flag2):
            _g_callout.show()
        else:
            _g_callout.hide()

    def _draw_map(self, df: pd.DataFrame):

        """
        min_val = df.min().min()
        max_val = df.max().max()
        intensity_max_abs = max(abs(max_val), abs(min_val))
        """
        intensity_max = df.max().max()
        if intensity_max < 1:
            intensity_max = abs(df.min().min())

        _g_gradMng.updateColorTable(intensity_max)

        rows_list = [n for n in df.index.to_list()]
        cols_list = [n for n in df.columns.to_list()]

        delta_y = rows_list[1] - rows_list[0]
        delta_x = cols_list[1] - cols_list[0]

        chart = self.chart()
        diff = df.size - len(chart.series())

        self._sts_bar.set_label_text("Generating Heat Map : [2/3]")
        if 0 < diff:
            self._sts_bar.set_bar_range(0, diff)
            ax_h = chart.axes(Qt.Horizontal)[0]
            ax_v = chart.axes(Qt.Vertical)[0]
            for i in range(diff):
                block = HeatBlockSeries()
                chart.addSeries(block)
                block.attachAxis(ax_h)
                block.attachAxis(ax_v)
                self._sts_bar.set_bar_value(i + 1)
        elif diff < 0:
            self._sts_bar.set_bar_range(0, -diff)
            for i in range(-diff):
                srlist = chart.series()
                chart.removeSeries(srlist[-1])
                self._sts_bar.set_bar_value(i + 1)

        self._sts_bar.set_label_text("Generating Heat Map : [3/3]")
        self._sts_bar.set_bar_range(0, df.size)
        itr = 0
        blklist = chart.series()
        for upper_y, row in df.iterrows():
            lower_y = upper_y - delta_y
            for idx_num, x in enumerate(row):
                right_x = cols_list[idx_num]
                left_x = right_x - delta_x
                blklist[itr].set_block(left_x, right_x, upper_y, lower_y, x)
                itr += 1
                self._sts_bar.set_bar_value(itr)

        ax_h = chart.axes(Qt.Horizontal)[0]
        ax_h.setRange(cols_list[0] - delta_x, cols_list[-1])
        ax_v = chart.axes(Qt.Vertical)[0]
        ax_v.setRange(rows_list[0] - delta_y, rows_list[-1])

    def _update_hmap(self, df: pd.DataFrame):

        intensity_max = df.max().max()
        if intensity_max < 1:
            intensity_max = abs(df.min().min())

        _g_gradMng.updateColorTable(intensity_max)

        self._sts_bar.set_label_text("Generating Heat Map : [3/3]")
        self._sts_bar.set_bar_range(0, df.size)
        itr = 0
        srlist = self.chart().series()
        for _, row in df.iterrows():
            for x in row:
                srlist[itr].set_intensity(x)
                itr += 1
                self._sts_bar.set_bar_value(itr)


class HeatMapUi(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("Gap-Fill Heat Map")

        sts_bar = StatusBar(ui.statusbar)
        chart_view = HeatMapChartView(ui.widget_HeatMap, sts_bar)

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

        callback = self._on_gradientBtoYPB_clicked
        ui.pushButton_gradientBtoYPB.clicked.connect(callback)

        callback = self._on_gradientGtoRPB_clicked
        ui.pushButton_gradientGtoRPB.clicked.connect(callback)

        _g_gradMng.setGradient(grBtoY)
        color_map = ColorMapLabel(ui.widget_ColorMap)

        callback = self._on_pushButtonGenHMap_clicked
        ui.pushButton_genHMap.clicked.connect(callback)

        callback = self._on_spinBoxDecim_changed
        ui.spinBox_Decim.valueChanged.connect(callback)

        callback = self._on_pushButtonAutoUpdate_toggled
        ui.pushButton_AutoUpdate.toggled.connect(callback)

        # ---------- Date Range ----------
        callback = self._on_spinBoxDateStep_changed
        ui.spinBox_DateStep.valueChanged.connect(callback)

        callback = self._on_scrollBarDate_changed
        ui.scrollBar_Date.valueChanged.connect(callback)
        # ---------- Gap Direction ----------
        callback = self._on_radioButtonGapDirAll_clicked
        ui.radioButton_GapDirAll.clicked.connect(callback)
        callback = self._on_radioButtonGapDirUp_clicked
        ui.radioButton_GpaDirUp.clicked.connect(callback)
        callback = self._on_radioButtonGapDirDown_clicked
        ui.radioButton_GapDirLo.clicked.connect(callback)

        self._hmap = HeatMap(sts_bar)

        self._ui = ui
        self._chart_view = chart_view
        self._color_map = color_map

        self._is_chart_updatable = True

    def set_param(self,
                  inst_idx: int,
                  df_gap: pd.DataFrame):

        self._hmap.set_param(df_gap, inst_idx)

        shape = self._hmap.shape
        lenmax = max(shape)
        val = math.ceil(lenmax / 100)

        self._ui.spinBox_Decim.setValue(val)

    def _on_pushButtonAutoUpdate_toggled(self, checked: bool):
        if checked:
            self._update_hmap()

    def _on_spinBoxDateStep_changed(self, value):
        if self._is_chart_updatable:
            self._hmap.date_step = value
            maxval = self._ui.spinBox_DateStep.maximum()
            self._ui.scrollBar_Date.setMaximum(maxval - value)
            if self._ui.pushButton_AutoUpdate.isChecked():
                self._update_hmap()

    def _on_scrollBarDate_changed(self, value):
        if self._is_chart_updatable:
            self._hmap.date_pos = value
            if self._ui.pushButton_AutoUpdate.isChecked():
                self._update_hmap()

    def _on_radioButtonGapDirAll_clicked(self):
        self._hmap.switch_dir_all()
        self._update_status()
        if self._ui.pushButton_AutoUpdate.isChecked():
            self._update_hmap()

    def _on_radioButtonGapDirUp_clicked(self):
        self._hmap.switch_dir_up()
        self._update_status()
        if self._ui.pushButton_AutoUpdate.isChecked():
            self._update_hmap()

    def _on_radioButtonGapDirDown_clicked(self):
        self._hmap.switch_dir_down()
        self._update_status()
        if self._ui.pushButton_AutoUpdate.isChecked():
            self._update_hmap()

    def _update_status(self):
        self._is_chart_updatable = False
        date_pos = self._hmap.date_pos
        date_step = self._hmap.date_step
        data_len = self._hmap.data_len
        self._ui.spinBox_DateStep.setMaximum(data_len)
        self._ui.spinBox_DateStep.setValue(date_step)
        self._ui.scrollBar_Date.setMaximum(data_len - date_step)
        self._ui.scrollBar_Date.setValue(date_pos)
        self._update_date_list()
        self._is_chart_updatable = True

    def _update_hmap(self):
        df = self._hmap.tuned_hmap()
        self._chart_view.update_hmap(df)
        self._update_date_list()
        self._color_map.update_intensity_range()

    def _on_pushButtonGenHMap_clicked(self):

        self._ui.pushButton_AutoUpdate.setChecked(False)

        deci = self._ui.spinBox_Decim.value()
        df = self._hmap.reset_hmap(deci)

        self._chart_view.rebuild_hmap(df)
        self._color_map.update_intensity_range()

        self._update_status()

        self._ui.radioButton_GapDirAll.setChecked(True)

    def _update_date_list(self):
        text = ""
        for date in self._hmap.date_list:
            text += date + "\n"
        self._ui.plainTextEdit_DateList.setPlainText(text)

    def _on_gradientBtoYPB_clicked(self):
        grBtoY = QLinearGradient(0, 0, 0, 100)
        grBtoY.setColorAt(1.0, Qt.black)
        grBtoY.setColorAt(0.67, Qt.blue)
        grBtoY.setColorAt(0.33, Qt.red)
        grBtoY.setColorAt(0.0, Qt.yellow)
        _g_gradMng.setGradient(grBtoY)

        self._chart_view.update_color()
        self._color_map.update_color_scale()

    def _on_gradientGtoRPB_clicked(self):
        grGtoR = QLinearGradient(0, 0, 0, 100)
        grGtoR.setColorAt(1.0, Qt.darkGreen)
        grGtoR.setColorAt(0.5, Qt.yellow)
        grGtoR.setColorAt(0.2, Qt.red)
        grGtoR.setColorAt(0.0, Qt.darkRed)
        _g_gradMng.setGradient(grGtoR)

        self._chart_view.update_color()
        self._color_map.update_color_scale()

    def _on_spinBoxDecim_changed(self, decim):
        shape = self._hmap.shape

        row_len = math.ceil(shape[0] / decim)
        col_len = math.ceil(shape[1] / decim)
        txt = "rows len: " + str(row_len) + "\ncols len: " + str(col_len)
        self._ui.label_Roughness.setText(txt)

    def _load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "heatmap.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        fs = self._ui.widget_HeatMap.frameSize()
        self._chart_view.resize(fs)
        fs = self._ui.widget_ColorMap.frameSize()
        self._color_map.resize(fs)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        fs = self._ui.widget_HeatMap.frameSize()
        self._chart_view.resize(fs)
        fs = self._ui.widget_ColorMap.frameSize()
        self._color_map.resize(fs)


if __name__ == "__main__":

    def gen_sample_gapdata():

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

        datalist = [
            ["2020-04-06", 0, 108.528, 108.382004, 0.145996, 0.096001,
                True, True, "7:00:00", 0.112999, 108.976997],
            ["2020-04-13", 0, 108.464996, 108.403, 0.061996,
                0.011993, True, True, "6:50:00", 0.100006, 108.167],
            ["2020-04-27", 1, 107.503006, 107.574005, 0.070999,
                0.020996, False, True, "7:00:00", 0.125999, 107.556999],
            ["2020-05-11", 0, 106.639999, 106.539993, 0.100006,
                0.050003, True, True, "6:40:00", 0.133995, 106.950996],
            ["2020-05-18", 1, 107.074501, 107.114502, 0.040001,
                0.002502, True, False, "", 0.228996, 107.178001],
            ["2020-06-01", 0, 107.820999, 107.720001, 0.100998,
                0.087997, True, True, "9:10:00", 0.094002, 107.744003],
            ["2020-06-22", 0, 106.891006, 106.772995, 0.118011,
                0.068008, False, True, "9:20:00", 0.099998, 106.864998],
            ["2020-06-29", 0, 107.218994, 107.160004, 0.05899,
                0.008995, True, True, "8:50:00", 0.148003, 107.266998],
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

    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])

    df_param, inst_idx = gen_sample_gapdata()

    widget = HeatMap()
    widget.set_param(inst_idx, df_param)
    widget.show()
    widget.init_resize()

    sys.exit(app.exec_())
