import pandas as pd
from PySide2.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget
from PySide2.QtWidgets import QGraphicsLineItem
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QPointF, QRectF, QRect, QLineF
from PySide2.QtCore import QDateTime, QDate, QTime
from PySide2.QtGui import QPalette, QColor, QFont, QFontMetrics, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient, QPen
from trade_monitor.utilities import GRAN_FREQ_DICT

CALLOUT_PRICE_COLOR = QColor(204, 0, 51)
CALLOUT_DATE_COLOR = QColor(0, 204, 51)


class BaseCalloutChart(QGraphicsItem):

    def __init__(self, parent: QtCharts.QChart):
        super().__init__()
        self._chart = parent
        self._text = ""
        self._anchor = QPointF()
        self._font = QFont()
        self._textRect = QRectF()
        self._rect = QRectF()
        self._backgroundColor = QColor(Qt.black)
        self._textColor = QColor(Qt.white)

    def updateGeometry(self, text: str, point: QPointF):
        raise NotImplementedError()

    def boundingRect(self) -> QRectF:
        # print("--- boundingRect ---")
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
        raise NotImplementedError()

    def setBackgroundColor(self, color: QColor):
        self._backgroundColor = color

    def setTextColor(self, color: QColor):
        self._textColor = color

    def _setText(self, text: str):
        self._text = text

        metrics = QFontMetrics(self._chart.font())
        self._textRect = QRectF(metrics.boundingRect(QRect(0, 0, 0, 0),
                                                     Qt.AlignLeft,
                                                     self._text))
        dx = 5
        dy = 5
        self._textRect.translate(dx, dy)
        self.prepareGeometryChange()
        self._rect = self._textRect.adjusted(-dx, -dy, dx, dy)


class CalloutDataTime(BaseCalloutChart):

    def __init__(self, parent: QtCharts.QChart):
        super().__init__(parent)

    def updateGeometry(self, text: str, point: QPointF):

        self._setText(text)
        self._anchor = point

        self.prepareGeometryChange()
        anchor = QPointF()
        anchor.setX(self._anchor.x() - self._rect.width() / 2)
        anchor.setY(self._chart.plotArea().bottom())
        self.setPos(anchor)

    def paint(self,
              painter: QPainter,
              option: QStyleOptionGraphicsItem,
              widget: QWidget):
        path = QPainterPath()
        path.addRoundedRect(self._rect, 5, 5)   # 丸みを帯びた長方形の角を規定

        # 枠を描写
        painter.setBrush(self._backgroundColor)    # 図形の塗りつぶし
        painter.setPen(QPen(self._backgroundColor))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(self._textColor))
        painter.drawText(self._textRect, self._text)


class CallouPrice(BaseCalloutChart):

    def __init__(self, parent: QtCharts.QChart):
        super().__init__(parent)

    def updateGeometry(self, text: str, point: QPointF):

        self._setText(text)
        self._anchor = point

        self.prepareGeometryChange()
        anchor = QPointF()
        anchor.setX(self._chart.plotArea().left() - self._rect.width())
        anchor.setY(self._anchor.y() - self._rect.height() / 2)
        self.setPos(anchor)

    def paint(self,
              painter: QPainter,
              option: QStyleOptionGraphicsItem,
              widget: QWidget):
        path = QPainterPath()
        path.addRoundedRect(self._rect, 5, 5)   # 丸みを帯びた長方形の角を規定

        # 枠を描写
        painter.setBrush(self._backgroundColor)    # 図形の塗りつぶし
        painter.setPen(QPen(self._backgroundColor))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(self._textColor))
        painter.drawText(self._textRect, self._text)


class BaseCandlestickChart(QtCharts.QChartView):

    COL_NAME_OP = "open"
    COL_NAME_HI = "high"
    COL_NAME_LO = "low"
    COL_NAME_CL = "close"

    def __init__(self, widget):
        super().__init__(widget)

        self._CALLOUT_DT_FMT = "yyyy/MM/dd hh:mm"

        # ---------- Create Chart ----------
        chart = QtCharts.QChart()
        chart.layout().setContentsMargins(0, 0, 0, 0)
        chart.setBackgroundRoundness(0)

        # ---------- Add Series on chart ----------
        ser_cdl = QtCharts.QCandlestickSeries()
        ser_cdl.setDecreasingColor(Qt.red)
        ser_cdl.setIncreasingColor(Qt.green)
        chart.addSeries(ser_cdl)

        """
        # ---------- Set font on chart ----------
        font = QFont("Sans Serif", )
        font.setPixelSize(18)
        chart.setTitleFont(font)
        """

        # ---------- Set palette on chart ----------
        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)
        chart.setPalette(palette)

        # ---------- Set PlotAreaBackground on chart ----------
        plotAreaGradient = QLinearGradient(0, 100, 0, 400)
        plotAreaGradient.setColorAt(0.0, QColor("#f1f1f1"))
        plotAreaGradient.setColorAt(1.0, QColor("#ffffff"))
        chart.setPlotAreaBackgroundBrush(plotAreaGradient)
        chart.setPlotAreaBackgroundVisible(True)

        # ---------- Set X Axis on chart ----------
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
        chart.addAxis(axis_x, Qt.AlignBottom)
        ser_cdl.attachAxis(axis_x)

        # ---------- Set Y Axis on chart ----------
        axis_y = QtCharts.QValueAxis()
        chart.addAxis(axis_y, Qt.AlignLeft)
        ser_cdl.attachAxis(axis_y)

        # ---------- Set Animation on chart ----------
        chart.setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        # ---------- Set Legend on chart ----------
        chart.legend().hide()
        chart.legend().setVisible(False)

        self.setChart(chart)

        # ---------- Add CalloutDataTime on scene ----------
        self._callout_dt = CalloutDataTime(chart)
        self._callout_dt.setBackgroundColor(CALLOUT_DATE_COLOR)
        self._callout_dt.setZValue(100)
        self.scene().addItem(self._callout_dt)

        # ---------- Add CallouPrice on scene ----------
        self._callout_pr = CallouPrice(chart)
        self._callout_pr.setBackgroundColor(CALLOUT_PRICE_COLOR)
        self._callout_pr.setZValue(100)
        self.scene().addItem(self._callout_pr)

        # ---------- Add CallouVerticalLine on scene ----------
        self._callout_vl = QGraphicsLineItem()
        pen = self._callout_vl.pen()
        pen.setColor(CALLOUT_DATE_COLOR)
        pen.setWidth(1)
        self._callout_vl.setPen(pen)
        self._callout_vl.setZValue(100)
        self.scene().addItem(self._callout_vl)

        # ---------- Add CallouHorizontalLine on scene ----------
        self._callout_hl = QGraphicsLineItem()
        pen = self._callout_hl.pen()
        pen.setColor(CALLOUT_PRICE_COLOR)
        pen.setWidth(1)
        self._callout_hl.setPen(pen)
        self._callout_hl.setZValue(100)
        self.scene().addItem(self._callout_hl)

        self.resize(widget.frameSize())

        self._ser_cdl = ser_cdl
        self._decimal_digit = 0
        self._freq = "D"

    def set_max_y(self, max_y):
        self._max_y = max_y

    def set_min_y(self, min_y):
        self._min_y = min_y

    def update(self, df, gran_id, decimal_digit):

        self._ser_cdl.clear()
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
            self._ser_cdl.append(cnd)

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

        self._decimal_digit = decimal_digit
        self._freq = GRAN_FREQ_DICT[gran_id]

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        chart = self.chart()
        flag = chart.plotArea().contains(event.pos())
        if flag:
            m2v = chart.mapToValue(event.pos())
            pdt = QDateTime.fromMSecsSinceEpoch(round(m2v.x())).toPython()
            pdt = pd.to_datetime(pdt).round(self._freq)

            qd = QDate(pdt.year, pdt.month, pdt.day)
            qt = QTime(pdt.hour, pdt.minute, pdt.second)
            qdttm = QDateTime(qd, qt)

            m2v.setX(qdttm.toMSecsSinceEpoch())
            m2p = chart.mapToPosition(m2v)
            dtstr = qdttm.toString(self._CALLOUT_DT_FMT)
            self._callout_dt.updateGeometry(dtstr, m2p)
            self._callout_dt.show()

            fmt = "{:." + str(self._decimal_digit) + "f}"
            prstr = fmt.format(m2v.y())
            self._callout_pr.updateGeometry(prstr, event.pos())
            self._callout_pr.show()

            plotAreaRect = chart.plotArea()
            self._callout_vl.setLine(QLineF(m2p.x(),
                                            plotAreaRect.top(),
                                            m2p.x(),
                                            plotAreaRect.bottom()))
            self._callout_vl.show()

            self._callout_hl.setLine(QLineF(plotAreaRect.left(),
                                            event.pos().y(),
                                            plotAreaRect.right(),
                                            event.pos().y()))
            self._callout_hl.show()

        else:
            self._callout_dt.hide()
            self._callout_pr.hide()
            self._callout_vl.hide()
            self._callout_hl.hide()


class BaseLineChart(QtCharts.QChartView):

    COL_NAME_OP = "open"
    COL_NAME_HI = "high"
    COL_NAME_LO = "low"
    COL_NAME_CL = "close"

    def __init__(self, widget):
        super().__init__(widget)

        self._CALLOUT_DT_FMT = "yyyy/MM/dd hh:mm"

        # ---------- Create Chart ----------
        chart = QtCharts.QChart()
        chart.layout().setContentsMargins(0, 0, 0, 0)
        chart.setBackgroundRoundness(0)

        # ---------- Add Series on chart ----------
        ser_line = QtCharts.QLineSeries()
        chart.addSeries(ser_line)

        """
        # ---------- Set font on chart ----------
        font = QFont("Sans Serif", )
        font.setPixelSize(18)
        chart.setTitleFont(font)
        """

        # ---------- Set palette on chart ----------
        palette = QPalette()
        palette.setColor(QPalette.Text, Qt.red)
        chart.setPalette(palette)

        # ---------- Set PlotAreaBackground on chart ----------
        plotAreaGradient = QLinearGradient(0, 100, 0, 400)
        plotAreaGradient.setColorAt(0.0, QColor("#f1f1f1"))
        plotAreaGradient.setColorAt(1.0, QColor("#ffffff"))
        chart.setPlotAreaBackgroundBrush(plotAreaGradient)
        chart.setPlotAreaBackgroundVisible(True)

        # ---------- Set X Axis on chart ----------
        axis_x = QtCharts.QDateTimeAxis()
        axis_x.setTickCount(2)
        # axis_x.setTitleText("Date")
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
        chart.addAxis(axis_x, Qt.AlignBottom)
        ser_line.attachAxis(axis_x)

        # ---------- Set Y Axis on chart ----------
        axis_y = QtCharts.QValueAxis()
        chart.addAxis(axis_y, Qt.AlignLeft)
        ser_line.attachAxis(axis_y)

        # ---------- Set Animation on chart ----------
        # chart.setAnimationOptions(QtCharts.QChart.SeriesAnimations)

        # ---------- Set Legend on chart ----------
        chart.legend().hide()
        chart.legend().setVisible(False)

        self.setChart(chart)

        # ---------- Add CalloutDataTime on scene ----------
        self._callout_dt = CalloutDataTime(chart)
        self._callout_dt.setBackgroundColor(CALLOUT_DATE_COLOR)
        self._callout_dt.setZValue(100)
        self.scene().addItem(self._callout_dt)

        # ---------- Add CallouPrice on scene ----------
        self._callout_pr = CallouPrice(chart)
        self._callout_pr.setBackgroundColor(CALLOUT_PRICE_COLOR)
        self._callout_pr.setZValue(100)
        self.scene().addItem(self._callout_pr)

        # ---------- Add CallouVerticalLine on scene ----------
        self._callout_vl = QGraphicsLineItem()
        pen = self._callout_vl.pen()
        pen.setColor(CALLOUT_DATE_COLOR)
        pen.setWidth(1)
        self._callout_vl.setPen(pen)
        self._callout_vl.setZValue(100)
        self.scene().addItem(self._callout_vl)

        # ---------- Add CallouHorizontalLine on scene ----------
        self._callout_hl = QGraphicsLineItem()
        pen = self._callout_hl.pen()
        pen.setColor(CALLOUT_PRICE_COLOR)
        pen.setWidth(1)
        self._callout_hl.setPen(pen)
        self._callout_hl.setZValue(100)
        self.scene().addItem(self._callout_hl)

        self.resize(widget.frameSize())

        self._ser_line = ser_line
        self._decimal_digit = 0
        self._freq = "D"

    def set_max_y(self, max_y):
        self._max_y = max_y

    def set_min_y(self, min_y):
        self._min_y = min_y

    def update(self, gran_id, decimal_digit):

        chart = self.chart()
        chart.axisY().setRange(self._min_y, self._max_y)

        self._decimal_digit = decimal_digit
        self._freq = GRAN_FREQ_DICT[gran_id]

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        chart = self.chart()
        flag = chart.plotArea().contains(event.pos())
        if flag:
            m2v = chart.mapToValue(event.pos())
            pdt = QDateTime.fromMSecsSinceEpoch(round(m2v.x())).toPython()
            pdt = pd.to_datetime(pdt).round(self._freq)

            qd = QDate(pdt.year, pdt.month, pdt.day)
            qt = QTime(pdt.hour, pdt.minute, pdt.second)
            qdttm = QDateTime(qd, qt)

            m2v.setX(qdttm.toMSecsSinceEpoch())
            m2p = chart.mapToPosition(m2v)
            dtstr = qdttm.toString(self._CALLOUT_DT_FMT)
            self._callout_dt.updateGeometry(dtstr, m2p)
            self._callout_dt.show()

            fmt = "{:." + str(self._decimal_digit) + "f}"
            prstr = fmt.format(m2v.y())
            self._callout_pr.updateGeometry(prstr, event.pos())
            self._callout_pr.show()

            plotAreaRect = chart.plotArea()
            self._callout_vl.setLine(QLineF(m2p.x(),
                                            plotAreaRect.top(),
                                            m2p.x(),
                                            plotAreaRect.bottom()))
            self._callout_vl.show()

            self._callout_hl.setLine(QLineF(plotAreaRect.left(),
                                            event.pos().y(),
                                            plotAreaRect.right(),
                                            event.pos().y()))
            self._callout_hl.show()

        else:
            self._callout_dt.hide()
            self._callout_pr.hide()
            self._callout_vl.hide()
            self._callout_hl.hide()
