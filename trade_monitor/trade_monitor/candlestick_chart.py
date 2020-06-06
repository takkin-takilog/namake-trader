import datetime as dt
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QDateTime, QDate
from PySide2.QtGui import QPalette, QLinearGradient, QColor, QFont
from trade_manager_msgs.msg import Granularity as Gran


class CandlestickChart(object):

    COL_NAME_OP = "open"
    COL_NAME_HI = "high"
    COL_NAME_LO = "low"
    COL_NAME_CL = "close"

    def __init__(self, widget):

        # Chart Type
        series = QtCharts.QCandlestickSeries()
        series.setDecreasingColor(Qt.red)
        series.setIncreasingColor(Qt.green)

        # Create Chart and set General Chart setting
        chart = QtCharts.QChart()
        chart.createDefaultAxes()
        chart.addSeries(series)

        #Title Font size
        """
        font = QFont("Sans Serif", )
        font.setPixelSize(18)
        chart.setTitleFont(font)
        """

        #chart.setTitle("Temperature in Celcius For Device:")
        chart.setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)
        chart.setPalette(palette)

        # Chart Background
        """
        backgroundGradient = QLinearGradient(0, 0, 0, 400)
        backgroundGradient.setColorAt(0.0, QColor('#50a1dc'))
        backgroundGradient.setColorAt(1.0, QColor('#00a1de'))
        chart.setBackgroundBrush(backgroundGradient)
        """

        #Plot area background
        plotAreaGradient = QLinearGradient(0, 100, 0, 400)
        plotAreaGradient.setColorAt(0.0, QColor('#f1f1f1'))
        plotAreaGradient.setColorAt(1.0, QColor('#ffffff'))
        chart.setPlotAreaBackgroundBrush(plotAreaGradient)
        chart.setPlotAreaBackgroundVisible(True)

        # X Axis Settings
        #axis_x = QtCharts.QDateTimeAxis()
        axis_x = QtCharts.QBarCategoryAxis()
        #axis_x.setFormat("yyyy-MM-dd")
        axis_x.setTitleText("Date")
        axis_x.setLabelsAngle(-90)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        axis_y.setTitleText("Ratio")

        # Customize axis label font
        #Lfont = QFont("Sans Serif")
        #Lfont.setPixelSize(16)
        #axis_x.setLabelsFont(Lfont)
        #axis_y.setLabelsFont(Lfont)

        """
        # Customize axis colors
        axisPen = QPen(QColor('#151512'))
        axisPen.setWidth(2)
        axisX.setLinePen(axisPen)
        axisY.setLinePen(axisPen)

        # Customize axis label colors
        axisBrush = QBrush(QColor('#ffffff'))
        axisX.setLabelsBrush(axisBrush)
        axisY.setLabelsBrush(axisBrush)

        # Customize grid lines and shades
        axisX.setGridLineVisible(True)
        axisY.setGridLineVisible(True)
        axisY.setShadesBrush(QBrush(QColor('#ffffff')))
        # axisX.setShadesBrush(QBrush(QColor('#ffffff')))
        axisY.setShadesVisible(True)
        """

        # add Axis
        chart.addAxis(axis_x, Qt.AlignBottom)
        series.attachAxis(axis_x)
        chart.addAxis(axis_y, Qt.AlignLeft)
        series.attachAxis(axis_y)


        """
        既存
        chart.addSeries(series)
        chart.setAxisX(axis_x, series)
        chart.setAxisY(axis_y, series)

        """
        chart.legend().hide()

        chartview = QtCharts.QChartView(chart)
        chartview.setParent(widget)
        chartview.resize(widget.frameSize())

        chart.legend().setVisible(False)

        self.__series = series
        self.__chart = chart
        self.__chartview = chartview

    def update(self, df, gran_id):

        max_y = df[self.COL_NAME_HI].max()
        min_y = df[self.COL_NAME_LO].min()

        max_x = df.index.max()
        min_x = df.index.min()

        cnt = 0
        x_axis_label = []
        self.__series.clear()
        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            t_ = QDateTime(time)
            cnt = cnt + 1
            x_axis_label.append(str(cnt))
            """
            cnd = QtCharts.QCandlestickSet(
                o_, h_, l_, c_, t_.toMSecsSinceEpoch())
            """
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_)
            self.__series.append(cnd)
        #self.__chart.createDefaultAxes()

        self.__chart.axisX(self.__series).setCategories(x_axis_label)

        self.__chart.axisX().setRange(0, cnt)
        self.__chart.axisY().setRange(min_y, max_y)

    def resize(self, frame_size):
        self.__chartview.resize(frame_size)
