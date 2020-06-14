from abc import ABCMeta, abstractmethod
import pandas as pd
import datetime as dt
from PySide2.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QRectF, QRect, QLineF
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QFontMetrics, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient
from PySide2.QtWidgets import QGraphicsLineItem
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
        axis_x = QtCharts.QBarCategoryAxis()
        axis_x.setTitleText("Date")
        axis_x.setLabelsAngle(45)

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

        max_y = df[self.COL_NAME_HI].max()
        min_y = df[self.COL_NAME_LO].min()

        if gran_id < Gran.GRAN_D:
            fmt = "%Y/%m/%d %H:%M"
        else:
            fmt = "%Y/%m/%d"

        x_axis_label = []
        self.__series.clear()
        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            x_axis_label.append(time.strftime(fmt))
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_)
            self.__series.append(cnd)

        self.__chart.axisX(self.__series).setCategories(x_axis_label)
        self.__chart.axisX().setRange(x_axis_label[0], x_axis_label[-1])
        self.__chart.axisY().setRange(min_y, max_y)

        self.__chartview.setRubberBand(QtCharts.QChartView.HorizontalRubberBand)

    def resize(self, frame_size):
        self.__chartview.resize(frame_size)


class CalloutAbs(QGraphicsItem):
    __metaclass__ = ABCMeta

    def __init__(self, parent: QtCharts.QChart):
        super().__init__()
        self._chart = parent
        self._text = ""
        self._anchor = QPointF()
        self._font = QFont()
        self._textRect = QRectF()
        self._rect = QRectF()

    def setText(self, text: str):
        self._text = text

        metrics = QFontMetrics(self._chart.font())
        self._textRect = QRectF(metrics.boundingRect(QRect(0, 0, 150, 150),
                                                     Qt.AlignLeft,
                                                     self._text))
        self._textRect.translate(5, 5)
        self.prepareGeometryChange()
        self._rect = self._textRect.adjusted(-5, -5, 5, 5)

    def setAnchor(self, point):
        self._anchor = point

    def updateGeometry(self):
        self.prepareGeometryChange()
        anchor = QPointF()
        anchor.setX(self._anchor.x() - self._rect.width() / 2)
        anchor.setY(self._chart.plotArea().bottom())
        self.setPos(anchor)

    def boundingRect(self) -> QRectF:
        from_parent = self.mapFromParent(self._anchor)
        anchor = QPointF(from_parent)
        rect = QRectF()
        rect.setLeft(min(self._rect.left(), anchor.x()))
        rect.setRight(max(self._rect.right(), anchor.x()))
        rect.setTop(min(self._rect.top(), anchor.y()))
        rect.setBottom(max(self._rect.bottom(), anchor.y()))
        return rect

    def paint(self,
              painter: QPainter,
              option: QStyleOptionGraphicsItem,
              widget: QWidget):
        print("---------- paint ----------")
        path = QPainterPath()
        mr = self._rect
        print("mr: {}" .format(mr))
        path.addRoundedRect(mr, 5, 5)   # 丸みを帯びた長方形の角を規定

        # 枠を描写
        painter.setBrush(QColor(93, 93, 93, 90))    # 図形の塗りつぶし
        painter.setPen(QPen(QColor(93, 93, 93, 75)))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(QColor("white")))
        painter.drawText(self._textRect, self._text)

        #print("text: " .format(self._text))
        #print("text_rect: " .format(self._textRect))


class CalloutDataTime(CalloutAbs):

    def __init__(self, parent: QtCharts.QChart):
        super().__init__(parent)

    def setText(self, text: str):
        self._text = text

        metrics = QFontMetrics(self._chart.font())
        self._textRect = QRectF(metrics.boundingRect(QRect(0, 0, 0, 0),
                                                     Qt.AlignLeft,
                                                     self._text))

        print("------------------ metrics ---------------------")
        print(self._textRect)
        dx = 5
        dy = 5
        self._textRect.translate(dx, dy)
        self.prepareGeometryChange()
        self._rect = self._textRect.adjusted(-dx, -dy, dx, dy)

    def setAnchor(self, point):
        self._anchor = point

    def updateGeometry(self):
        self.prepareGeometryChange()
        anchor = QPointF()
        anchor.setX(self._anchor.x() - self._rect.width() / 2)
        anchor.setY(self._chart.plotArea().bottom())
        self.setPos(anchor)

    def boundingRect(self) -> QRectF:
        from_parent = self.mapFromParent(self._anchor)
        anchor = QPointF(from_parent)
        rect = QRectF()
        rect.setLeft(min(self._rect.left(), anchor.x()))
        rect.setRight(max(self._rect.right(), anchor.x()))
        rect.setTop(min(self._rect.top(), anchor.y()))
        rect.setBottom(max(self._rect.bottom(), anchor.y()))
        return rect

    def paint(self,
              painter: QPainter,
              option: QStyleOptionGraphicsItem,
              widget: QWidget):
        print("---------- paint ----------")
        path = QPainterPath()
        mr = self._rect
        print("mr: {}" .format(mr))
        path.addRoundedRect(mr, 5, 5)   # 丸みを帯びた長方形の角を規定

        # 枠を描写
        painter.setBrush(QColor(93, 93, 93, 90))    # 図形の塗りつぶし
        painter.setPen(QPen(QColor(93, 93, 93, 75)))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(QColor("white")))
        painter.drawText(self._textRect, self._text)

        #print("text: " .format(self._text))
        #print("text_rect: " .format(self._textRect))


class CandlestickChartAbs(QtCharts.QChartView):
    __metaclass__ = ABCMeta

    COL_NAME_OP = "open"
    COL_NAME_HI = "high"
    COL_NAME_LO = "low"
    COL_NAME_CL = "close"

    def __init__(self, widget):
        super().__init__()

        # Chart Type
        series = QtCharts.QCandlestickSeries()
        series.setDecreasingColor(Qt.red)
        series.setIncreasingColor(Qt.green)

        # Create Chart and set General Chart setting
        chart = QtCharts.QChart()
        #chart.createDefaultAxes()
        chart.layout().setContentsMargins(0, 0, 0, 0)
        chart.setBackgroundRoundness(0)
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

        chart.legend().hide()

        self.setChart(chart)
        self.setParent(widget)
        self.resize(widget.frameSize())

        chart.legend().setVisible(False)

        self._series = series
        self._chart = chart
        self._widget = widget

    @abstractmethod
    def update(self, df):
        raise NotImplementedError()


class CandlestickChartGapFillPrev(CandlestickChartAbs):

    def __init__(self, widget):
        super().__init__(widget)

        # X Axis Settings
        axis_x = QtCharts.QDateTimeAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Date")
        axis_x.setFormat("h:mm")
        axis_x.setLabelsAngle(0)

        now = QDateTime.currentDateTime()
        yesterday = QDateTime(QDate(now.date().year(),
                                    now.date().month(),
                                    now.date().day() - 1),
                              QTime(0, 0))
        today = QDateTime(QDate(now.date().year(),
                                now.date().month(),
                                now.date().day()),
                          QTime(0, 0))

        axis_x.setRange(yesterday, today)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        # axis_y.setTitleText("Ratio")

        # Customize axis label font
        #Lfont = QFont("Sans Serif")
        #Lfont.setPixelSize(16)
        #axis_x.setLabelsFont(Lfont)
        #axis_y.setLabelsFont(Lfont)

        self._chart.addAxis(axis_x, Qt.AlignBottom)
        self._series.attachAxis(axis_x)
        self._chart.addAxis(axis_y, Qt.AlignLeft)
        self._series.attachAxis(axis_y)

        self._callout_dt = CalloutDataTime(self._chart)
        self.scene().addItem(self._callout_dt)

        self._lineItem = QGraphicsLineItem()
        pen = self._lineItem.pen()
        pen.setColor(QColor(93, 93, 93, 75))
        pen.setWidth(3)
        self._lineItem.setPen(pen)
        self.scene().addItem(self._lineItem)

    def update(self, df, min_y, max_y):

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        min_x = QDateTime(qd, qt)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(60 * 10)
        max_x = QDateTime(qd, qt)

        dtstr = dt_.strftime("%Y/%m/%d")

        self._series.clear()
        for dt_, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            qd = QDate(dt_.year, dt_.month, dt_.day)
            qt = QTime(dt_.hour, dt_.minute)
            qdt = QDateTime(qd, qt)
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_,
                                           qdt.toMSecsSinceEpoch())
            self._series.append(cnd)

        self._chart.axisX().setTitleText(dtstr)
        self._chart.axisX().setRange(min_x, max_x)
        self._chart.axisY().setRange(min_y, max_y)
        self.__df = df

    def mouseMoveEvent(self, event):

        flag = self._chart.plotArea().contains(event.pos())
        if flag:
            m2v = self._chart.mapToValue(event.pos())
            dt_ = QDateTime.fromMSecsSinceEpoch(round(m2v.x()))
            dtstr = dt_.toString("yyyy/MM/dd hh:mm")
            pri = m2v.y()
            #print("x_p: {}" .format(dt_))
            m2p = self._chart.mapToPosition(m2v)
            print("-------------------- mouseMoveEvent -------------------------")
            """
            print("pos: {}" .format(event.pos()))
            print("m2v: {}" .format(m2v))
            print("m2p: {}" .format(m2p))
            """

            self._callout_dt.setText(dtstr)
            self._callout_dt.setAnchor(event.pos())
            self._callout_dt.setZValue(11)
            self._callout_dt.updateGeometry()
            self._callout_dt.show()

            plotAreaRect = self._chart.plotArea()
            self._lineItem.setLine(QLineF(event.pos().x(),
                                          plotAreaRect.top(),
                                          event.pos().x(),
                                          plotAreaRect.bottom()))
            self._lineItem.show()
        else:
            self._callout_dt.hide()
            self._lineItem.hide()


class CandlestickChartGapFillCurr(CandlestickChartAbs):

    def __init__(self, widget):
        super().__init__(widget)

        # X Axis Settings
        axis_x = QtCharts.QDateTimeAxis()
        axis_x.setTickCount(2)
        axis_x.setTitleText("Date")
        axis_x.setFormat("h:mm")
        axis_x.setLabelsAngle(0)

        now = QDateTime.currentDateTime()
        yesterday = QDateTime(QDate(now.date().year(),
                                    now.date().month(),
                                    now.date().day() - 1),
                              QTime(0, 0))
        today = QDateTime(QDate(now.date().year(),
                                now.date().month(),
                                now.date().day()),
                          QTime(0, 0))

        axis_x.setRange(yesterday, today)

        # Y Axis Settings
        axis_y = QtCharts.QValueAxis()
        # axis_y.setTitleText("Ratio")

        # Customize axis label font
        #Lfont = QFont("Sans Serif")
        #Lfont.setPixelSize(16)
        #axis_x.setLabelsFont(Lfont)
        #axis_y.setLabelsFont(Lfont)

        self._chart.addAxis(axis_x, Qt.AlignBottom)
        self._series.attachAxis(axis_x)
        self._chart.addAxis(axis_y, Qt.AlignRight)
        self._series.attachAxis(axis_y)

    def update(self, df, min_y, max_y, end_hour):

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(60 * (-10))
        min_x = QDateTime(qd, qt)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        max_x = QDateTime(qd, qt)

        # End vertical line
        qt = QTime(end_hour, 0)
        line_x = QDateTime(qd, qt).toMSecsSinceEpoch()

        dtstr = dt_.strftime("%Y/%m/%d")

        self._series.clear()
        for dt_, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            qd = QDate(dt_.year, dt_.month, dt_.day)
            qt = QTime(dt_.hour, dt_.minute)
            qdt = QDateTime(qd, qt)
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_,
                                           qdt.toMSecsSinceEpoch())
            self._series.append(cnd)

        self._chart.axisX().setTitleText(dtstr)
        self._chart.axisX().setRange(min_x, max_x)
        self._chart.axisY().setRange(min_y, max_y)
