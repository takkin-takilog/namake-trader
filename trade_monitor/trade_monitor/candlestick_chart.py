import datetime as dt
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QDateTime


class CandlestickChart(object):

    COL_NAME_OP = "open"
    COL_NAME_HI = "high"
    COL_NAME_LO = "low"
    COL_NAME_CL = "close"

    def __init__(self, widget, frame_size):

        series = QtCharts.QCandlestickSeries()
        series.setDecreasingColor(Qt.red)
        series.setIncreasingColor(Qt.green)

        # axises
        axis_x = QtCharts.QDateTimeAxis()
        axis_x.setFormat("yyyy-MM-dd hh:mm:ss")
        axis_x.setTitleText("Date")
        axis_x.setLabelsAngle(-90)

        axis_y = QtCharts.QValueAxis()
        axis_y.setTitleText("Ratio")

        chart = QtCharts.QChart()
        chart.addAxis(axis_x, Qt.AlignBottom)
        chart.addAxis(axis_y, Qt.AlignRight)
        chart.addSeries(series)
        chart.setAxisX(axis_x, series)
        chart.setAxisY(axis_y, series)

        chart.createDefaultAxes()
        chart.legend().hide()

        chartview = QtCharts.QChartView(chart)
        chartview.setParent(widget)
        chartview.resize(frame_size)

        self.__series = series
        self.__chart = chart
        self.__chartview = chartview

    def update(self, df):

        max_ = df[self.COL_NAME_HI].max()
        min_ = df[self.COL_NAME_LO].min()

        self.__series.clear()
        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            t_ = QDateTime(dt.datetime.date(time))

            cnd = QtCharts.QCandlestickSet(
                o_, h_, l_, c_, t_.toMSecsSinceEpoch())
            self.__series.append(cnd)

        self.__chart.axisY().setRange(min_, max_)
        self.__chart.createDefaultAxes()

    def resize(self, frame_size):
        self.__chartview.resize(frame_size)
