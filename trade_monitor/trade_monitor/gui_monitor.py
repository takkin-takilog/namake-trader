import sys
import os

import threading
import datetime as dt
import pandas as pd

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import Qt, QFile, QCoreApplication  # @UnresolvedImport
from PySide2.QtCore import QDateTime
from PySide2.QtUiTools import QUiLoader  # @UnresolvedImport
from PySide2.QtCharts import QtCharts
from PySide2.QtGui import QPainter

import rclpy
from rclpy.node import Node
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.msg import InstrumentMnt as InstMnt
from trade_manager_msgs.msg import GranularityMnt as GranMnt
from std_msgs.msg import String


class GuiMonitor(QMainWindow):

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

    COL_NAME_TIME = "time"
    COL_NAME_ASK_OP = "open(Ask)"
    COL_NAME_ASK_HI = "high(Ask)"
    COL_NAME_ASK_LO = "low(Ask)"
    COL_NAME_ASK_CL = "close(Ask)"
    COL_NAME_BID_OP = "open(Bid)"
    COL_NAME_BID_HI = "high(Bid)"
    COL_NAME_BID_LO = "low(Bid)"
    COL_NAME_BID_CL = "close(Bid)"
    COL_NAME_COMP = "complete"

    def __init__(self):
        super(GuiMonitor, self).__init__()
        ui = self.load_ui()

        series = QtCharts.QCandlestickSeries()
        series.setDecreasingColor(Qt.red)
        series.setIncreasingColor(Qt.green)

        # initialize ROS
        rclpy.init()
        node = rclpy.create_node('gui_monitor')
        self.logger = node.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        ui.pushButton_SrvReq.pressed.connect(self.publish_topic)

        self.sub = node.create_subscription(String,
                                            'chatter',
                                            self.listener_callback)

        # Create service client "CandlesMonitor"
        srv_type = CandlesMntSrv
        srv_name = "candles_monitor"
        cli_cdl = node.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli_cdl.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for \"" + srv_name + "\" service...")

        dt_now = dt.datetime.now()
        dt_from = dt_now - dt.timedelta(days=10)
        dt_to = dt_now

        req = CandlesMntSrv.Request()
        req.gran_msg.granularity_id = GranMnt.GRAN_D
        req.inst_msg.instrument_id = InstMnt.INST_USD_JPY
        req.dt_from = dt_from.strftime(self.DT_FMT)
        req.dt_to = dt_to.strftime(self.DT_FMT)

        future = cli_cdl.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        flg = future.done() and future.result() is not None
        assert flg, "initial fetch [Day Candle] failed!"

        data = []
        rsp = future.result()
        for cndl_msg in rsp.cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, self.DT_FMT)
            data.append([dt_,
                         cndl_msg.ask_o,
                         cndl_msg.ask_h,
                         cndl_msg.ask_l,
                         cndl_msg.ask_c,
                         cndl_msg.bid_o,
                         cndl_msg.bid_h,
                         cndl_msg.bid_l,
                         cndl_msg.bid_c,
                         cndl_msg.is_complete
                         ])

        df = pd.DataFrame(data)
        df.columns = [self.COL_NAME_TIME,
                      self.COL_NAME_ASK_OP,
                      self.COL_NAME_ASK_HI,
                      self.COL_NAME_ASK_LO,
                      self.COL_NAME_ASK_CL,
                      self.COL_NAME_BID_OP,
                      self.COL_NAME_BID_HI,
                      self.COL_NAME_BID_LO,
                      self.COL_NAME_BID_CL,
                      self.COL_NAME_COMP
                      ]
        df = df.set_index(self.COL_NAME_TIME)

        print("--------------------------------")
        print(df)

        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_ASK_OP]
            h_ = sr[self.COL_NAME_ASK_HI]
            l_ = sr[self.COL_NAME_ASK_LO]
            c_ = sr[self.COL_NAME_ASK_CL]
            t_ = QDateTime(dt.datetime.date(time))

            cnd = QtCharts.QCandlestickSet(o_, h_, l_, c_, t_.toMSecsSinceEpoch())
            series.append(cnd)

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
        chartview.setParent(ui.graphicsView_candlestickchart)

        #chartview.setChart(chart)
        #chartview.setRenderHint(QPainter.Antialiasing)

        """
        h = ui.graphicsView_candlestickchart.height()
        w = ui.graphicsView_candlestickchart.width()
        print("----------------------------------")
        print(h)
        print(w)

        chartview.resize(w, h)
        """

        ui.show()
        self.__node = node
        self.__ui = ui
        self.__chartview = chartview

    def listener_callback(self, msg):
        self.logger.debug("----- ROS Callback!")
        self.logger.debug(msg.data)

    @property
    def node(self) -> Node:
        return self.__node

    def load_ui(self):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gui_monitor.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, self)
        ui_file.close()

        return ui

    def publish_topic(self):
        self.logger.debug("push")

        h = self.__ui.graphicsView_candlestickchart.height()
        w = self.__ui.graphicsView_candlestickchart.width()
        print("----------------------------------")
        print(h)
        print(w)

        self.__chartview.resize(w, h)

        """
        self.pub.publish(str(self.current_value))
        self.pushButton.setEnabled(False)
        self.is_pub = True
        """


def main():

    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = GuiMonitor()
    #widget.show()

    ros_th = threading.Thread(target=rclpy.spin, args=(widget.node,))
    ros_th.start()

    errcd = app.exec_()
    widget.node.destroy_node()
    rclpy.shutdown()
    sys.exit(errcd)

if __name__ == "__main__":
    main()
