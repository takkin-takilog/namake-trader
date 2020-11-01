from PySide2.QtWidgets import QStyleOptionGraphicsItem, QWidget
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import Qt, QDateTime, QDate, QTime, QPointF, QLineF
from PySide2.QtGui import QPalette, QColor, QFont, QPen, QPainter, QPainterPath
from PySide2.QtGui import QLinearGradient
from PySide2.QtWidgets import QGraphicsLineItem
from trade_manager_msgs.msg import Granularity as Gran
from trade_monitor.abstract import AbstractCalloutChart, AbstractCandlestickChart

CALLOUT_PRICE_COLOR = QColor(204, 0, 51)
CALLOUT_DATE_COLOR = QColor(0, 204, 51)


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

        # Title Font size
        """
        font = QFont("Sans Serif", )
        font.setPixelSize(18)
        chart.setTitleFont(font)
        """

        # chart.setTitle("Temperature in Celcius For Device:")
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

        # Plot area background
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

        # #Customize axis label font
        # Lfont = QFont("Sans Serif")
        # Lfont.setPixelSize(16)
        # axis_x.setLabelsFont(Lfont)
        # axis_y.setLabelsFont(Lfont)

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

        self._series = series
        self._chart = chart
        self._chartview = chartview

    def update(self, df, gran_id):

        max_y = df[self.COL_NAME_HI].max()
        min_y = df[self.COL_NAME_LO].min()

        if gran_id < Gran.GRAN_D:
            fmt = "%Y/%m/%d %H:%M"
        else:
            fmt = "%Y/%m/%d"

        x_axis_label = []
        self._series.clear()
        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_OP]
            h_ = sr[self.COL_NAME_HI]
            l_ = sr[self.COL_NAME_LO]
            c_ = sr[self.COL_NAME_CL]
            x_axis_label.append(time.strftime(fmt))
            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_)
            self._series.append(cnd)

        self._chart.axisX(self._series).setCategories(x_axis_label)
        self._chart.axisX().setRange(x_axis_label[0], x_axis_label[-1])
        self._chart.axisY().setRange(min_y, max_y)

        self._chartview.setRubberBand(
            QtCharts.QChartView.HorizontalRubberBand)

    def resize(self, frame_size):
        self._chartview.resize(frame_size)


class CalloutDataTime(AbstractCalloutChart):

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
        painter.setBrush(CALLOUT_DATE_COLOR)    # 図形の塗りつぶし
        painter.setPen(QPen(CALLOUT_DATE_COLOR))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(QColor(Qt.white)))
        painter.drawText(self._textRect, self._text)


class CallouPrice(AbstractCalloutChart):

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
        painter.setBrush(CALLOUT_PRICE_COLOR)    # 図形の塗りつぶし
        painter.setPen(QPen(CALLOUT_PRICE_COLOR))
        painter.drawPath(path)

        # 文字を描写
        painter.setPen(QPen(QColor(Qt.white)))
        painter.drawText(self._textRect, self._text)


class CandlestickChartGapFillBase(AbstractCandlestickChart):

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
        # Lfont = QFont("Sans Serif")
        # Lfont.setPixelSize(16)
        # axis_x.setLabelsFont(Lfont)
        # axis_y.setLabelsFont(Lfont)
        chart = self.chart()
        chart.addAxis(axis_x, Qt.AlignBottom)
        self._ser_cdl.attachAxis(axis_x)
        chart.addAxis(axis_y, Qt.AlignLeft)
        self._ser_cdl.attachAxis(axis_y)

        self._callout_dt = CalloutDataTime(chart)
        self._callout_pr = CallouPrice(chart)
        self.scene().addItem(self._callout_dt)
        self.scene().addItem(self._callout_pr)

        # Vertical Line (callout)
        self._verline_callout = QGraphicsLineItem()
        pen = self._verline_callout.pen()
        pen.setColor(CALLOUT_DATE_COLOR)
        pen.setWidth(1)
        self._verline_callout.setPen(pen)
        self.scene().addItem(self._verline_callout)

        # Horizontal Line (callout)
        self._horline_callout = QGraphicsLineItem()
        pen = self._horline_callout.pen()
        pen.setColor(CALLOUT_PRICE_COLOR)
        pen.setWidth(1)
        self._horline_callout.setPen(pen)
        self.scene().addItem(self._horline_callout)

        # Horizontal Line (previous close price)
        self._horline_precls = QGraphicsLineItem()
        pen = self._horline_precls.pen()
        pen.setColor(Qt.magenta)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._horline_precls.setPen(pen)
        self.scene().addItem(self._horline_precls)

        # Horizontal Line (current open price)
        self._horline_curopn = QGraphicsLineItem()
        pen = self._horline_curopn.pen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._horline_curopn.setPen(pen)
        self.scene().addItem(self._horline_curopn)

        self._decimal_digit = 0
        self._is_update = False

    def set_max_y(self, max_y):
        self._max_y = max_y

    def set_min_y(self, min_y):
        self._min_y = min_y

    def update(self, df, gap_close_price, gap_open_price, decimal_digit):

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

        point = QPointF(0, gap_close_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._horline_precls.setLine(QLineF(plotAreaRect.left(),
                                            m2p.y(),
                                            plotAreaRect.right(),
                                            m2p.y()))

        point = QPointF(0, gap_open_price)
        m2p = chart.mapToPosition(point)
        plotAreaRect = chart.plotArea()
        self._horline_curopn.setLine(QLineF(plotAreaRect.left(),
                                            m2p.y(),
                                            plotAreaRect.right(),
                                            m2p.y()))

        self._horline_precls.show()
        self._horline_curopn.show()

        self._decimal_digit = decimal_digit

        self._gap_close_price = gap_close_price
        self._gap_open_price = gap_open_price
        self._is_update = True

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            chart = self.chart()
            point = QPointF(0, self._gap_close_price)
            m2p = chart.mapToPosition(point)
            plotAreaRect = chart.plotArea()
            self._horline_precls.setLine(QLineF(plotAreaRect.left(),
                                                m2p.y(),
                                                plotAreaRect.right(),
                                                m2p.y()))

            point = QPointF(0, self._gap_open_price)
            m2p = chart.mapToPosition(point)
            plotAreaRect = chart.plotArea()
            self._horline_curopn.setLine(QLineF(plotAreaRect.left(),
                                                m2p.y(),
                                                plotAreaRect.right(),
                                                m2p.y()))

            self._horline_precls.show()
            self._horline_curopn.show()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        chart = self.chart()
        flag = chart.plotArea().contains(event.pos())
        if flag:
            m2v = chart.mapToValue(event.pos())
            dt_ = QDateTime.fromMSecsSinceEpoch(round(m2v.x()))
            qtime = dt_.time()
            minu = round(qtime.minute() / 10) * 10
            qdttm = QDateTime(dt_.date(),
                              QTime(qtime.hour(), 0)).addSecs(60 * minu)
            m2v.setX(qdttm.toMSecsSinceEpoch())
            m2p = chart.mapToPosition(m2v)
            dtstr = qdttm.toString("yyyy/MM/dd hh:mm")
            self._callout_dt.setZValue(0)
            self._callout_dt.updateGeometry(dtstr, m2p)
            self._callout_dt.show()

            fmt = "{:." + str(self._decimal_digit) + "f}"
            prstr = fmt.format(m2v.y())
            self._callout_pr.setZValue(1)
            self._callout_pr.updateGeometry(prstr, event.pos())
            self._callout_pr.show()

            plotAreaRect = chart.plotArea()
            self._verline_callout.setLine(QLineF(m2p.x(),
                                                 plotAreaRect.top(),
                                                 m2p.x(),
                                                 plotAreaRect.bottom()))
            self._verline_callout.show()

            self._horline_callout.setLine(QLineF(plotAreaRect.left(),
                                                 event.pos().y(),
                                                 plotAreaRect.right(),
                                                 event.pos().y()))
            self._horline_callout.show()

        else:
            self._callout_dt.hide()
            self._callout_pr.hide()
            self._verline_callout.hide()
            self._horline_callout.hide()

        self._horline_precls.show()


class CandlestickChartGapFillPrev(CandlestickChartGapFillBase):

    def __init__(self, widget):
        super().__init__(widget)

    def update(self,
               df,
               gap_close_price,
               gap_open_price,
               decimal_digit):
        super().update(df, gap_close_price, gap_open_price, decimal_digit)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(60 * 10)
        max_x = QDateTime(qd, qt)

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        min_x = QDateTime(qd, qt)
        dtstr = dt_.strftime("%Y/%m/%d (Fri)")

        self.chart().axisX().setTitleText(dtstr)
        self.chart().axisX().setRange(min_x, max_x)


class CandlestickChartGapFillCurr(CandlestickChartGapFillBase):

    def __init__(self, widget):
        super().__init__(widget)

        # Vertical Line (end hour)
        self._verline_endhour = QGraphicsLineItem()
        pen = self._verline_endhour.pen()
        pen.setColor(Qt.cyan)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self._verline_endhour.setPen(pen)
        self.scene().addItem(self._verline_endhour)

        self._end_point = QPointF(0, 0)

    def update(self,
               df,
               gap_close_price,
               gap_open_price,
               decimal_digit,
               end_time):
        super().update(df, gap_close_price, gap_open_price, decimal_digit)

        dt_ = df.index[-1]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute)
        max_x = QDateTime(qd, qt)

        dt_ = df.index[0]
        qd = QDate(dt_.year, dt_.month, dt_.day)
        qt = QTime(dt_.hour, dt_.minute).addSecs(-60 * 10)
        min_x = QDateTime(qd, qt)
        dtstr = dt_.strftime("%Y/%m/%d (Mon)")

        self.chart().axisX().setTitleText(dtstr)
        self.chart().axisX().setRange(min_x, max_x)

        h = end_time.hour
        m = end_time.minute
        qdttm = QDateTime(dt_.date(), QTime(h, m))
        self._end_point.setX(qdttm.toMSecsSinceEpoch())
        self._update_end_hour()

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self._is_update:
            self._update_end_hour()

    def _update_end_hour(self):

        m2p = self.chart().mapToPosition(self._end_point)
        plotAreaRect = self.chart().plotArea()
        self._verline_endhour.setLine(QLineF(m2p.x(),
                                             plotAreaRect.top(),
                                             m2p.x(),
                                             plotAreaRect.bottom()))
        self._verline_endhour.show()
