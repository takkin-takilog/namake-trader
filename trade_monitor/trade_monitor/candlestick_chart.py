import datetime as dt
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QDateTime, QDate, QTime
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
        axis_x = QtCharts.QDateTimeAxis()
        axis_x.setTitleText("Date")
        axis_x.setFormat("h:mm")
        #axis_x.setLabelsAngle(0)

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

        chart.legend().hide()

        chartview = QtCharts.QChartView(chart)
        chartview.setParent(widget)
        chartview.resize(widget.frameSize())

        chart.legend().setVisible(False)

        self.__series = series
        self.__chart = chart
        self.__chartview = chartview

    def update(self, df, gran_id):

        min_y = df[self.COL_NAME_LO].min()
        max_y = df[self.COL_NAME_HI].max()

        if gran_id < Gran.GRAN_D:
            fmt = "%Y/%m/%d %H:%M"
        else:
            fmt = "%Y/%m/%d"

        self.__series.clear()
        qdatetimelist = []
        for dt_, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            qd = QDate(dt_.year, dt_.month, dt_.day)
            qt = QTime(dt_.hour, dt_.minute)
            qdt = QDateTime(qd, qt).toMSecsSinceEpoch()
            qdatetimelist.append(qdt)
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_, qdt)
            self.__series.append(cnd)

        min_x = qdatetimelist[0]
        max_x = qdatetimelist[-1]

        print("--------------------")
        print(min_x)
        print(max_x)

        self.__chart.axisX().setRange(min_x, max_x)
        self.__chart.axisY().setRange(min_y, max_y)

    def resize(self, frame_size):
        self.__chartview.resize(frame_size)
