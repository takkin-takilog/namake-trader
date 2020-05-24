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
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran
from std_msgs.msg import String


class MsgDict():

    def __init__(self, msg_id, text) -> None:
        self.__msg_id = msg_id
        self.__text = text

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text


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
    COL_NAME_MID_OP = "open(Mid)"
    COL_NAME_MID_HI = "high(Mid)"
    COL_NAME_MID_LO = "low(Mid)"
    COL_NAME_MID_CL = "close(Mid)"
    COL_NAME_COMP = "complete"

    INST_MSG_LIST = [
        MsgDict(Inst.INST_USD_JPY, "USD/JPY"),
        MsgDict(Inst.INST_EUR_JPY, "EUR/JPY"),
        MsgDict(Inst.INST_EUR_USD, "EUR/USD"),
    ]

    GRAN_MSG_LIST = [
        MsgDict(Gran.GRAN_D, "日足"),
        MsgDict(Gran.GRAN_H4, "４時間足"),
        MsgDict(Gran.GRAN_H1, "１時間足"),
    ]

    def __init__(self, parent=None):
        super(GuiMonitor, self).__init__(parent)
        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        # set comboBox of Instrument
        self.__remove_all_items_of_comboBox(ui.comboBox_inst)
        for obj in self.INST_MSG_LIST:
            ui.comboBox_inst.addItem(obj.text)

        # set comboBox of Granularity
        self.__remove_all_items_of_comboBox(ui.comboBox_gran)
        for obj in self.GRAN_MSG_LIST:
            ui.comboBox_gran.addItem(obj.text)

        ui.labe_srvcon_status.setAlignment(Qt.AlignCenter)

        ui.pushButton_srvcon.toggled.connect(self.__on_srvcon_toggled)
        ui.comboBox_inst.currentIndexChanged.connect(self.on_cb_inst_changed)
        ui.comboBox_gran.currentIndexChanged.connect(self.on_cb_gran_changed)

        series = QtCharts.QCandlestickSeries()
        series.setDecreasingColor(Qt.red)
        series.setIncreasingColor(Qt.green)

        # initialize ROS
        node = rclpy.create_node('gui_monitor')
        self.logger = node.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

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
        chartview.setParent(ui.widget_chart)
        fs = ui.widget_chart.frameSize()
        chartview.resize(fs)

        self.__node = node
        self.__ui = ui
        self.__cli_cdl = cli_cdl
        self.__series = series
        self.__chart = chart
        self.__chartview = chartview

    def listener_callback(self, msg):
        self.logger.debug("----- ROS Callback!")
        self.logger.debug(msg.data)

    @property
    def node(self) -> Node:
        return self.__node

    """
    @property
    def ui(self):
        return self.__ui
    """

    def __load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gui_monitor.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def __remove_all_items_of_comboBox(self, combo_box):

        idx = combo_box.currentIndex()
        while -1 < idx:
            combo_box.removeItem(idx)
            idx = combo_box.currentIndex()

    def on_cb_inst_changed(self, inst_idx):
        gran_idx = self.__ui.comboBox_gran.currentIndex()
        self.__display_chart(inst_idx, gran_idx)

    def on_cb_gran_changed(self, gran_idx):
        inst_idx = self.__ui.comboBox_inst.currentIndex()
        self.__display_chart(inst_idx, gran_idx)

    def __on_srvcon_toggled(self, flag):

        if flag is True:
            gran_idx = self.__ui.comboBox_gran.currentIndex()
            inst_idx = self.__ui.comboBox_inst.currentIndex()
            self.__display_chart(inst_idx, gran_idx)
            self.__ui.pushButton_srvcon.setText("切断")

            self.__ui.labe_srvcon_status.setText("接続中")
            str_ = "background-color: rgb(0, 255, 0);"
            self.__ui.labe_srvcon_status.setStyleSheet(str_);
        else:
            self.__ui.pushButton_srvcon.setText("接続")

            self.__ui.labe_srvcon_status.setText("切断")
            str_ = "background-color: rgb(136, 138, 133);"
            self.__ui.labe_srvcon_status.setStyleSheet(str_);

    def init_resize_qchart(self) -> None:
        fs = self.__ui.widget_chart.frameSize()
        self.__chartview.resize(fs)

    def resizeEvent(self, event):
        fs = self.__ui.widget_chart.frameSize()
        self.__chartview.resize(fs)

    def __display_chart(self, inst_idx, gran_idx):

        dt_now = dt.datetime.now()
        dt_from = dt_now - dt.timedelta(days=20)
        dt_to = dt_now

        inst_id = self.INST_MSG_LIST[inst_idx].msg_id
        gran_id = self.GRAN_MSG_LIST[gran_idx].msg_id

        req = CandlesMntSrv.Request()
        req.gran_msg.granularity_id = gran_id
        req.inst_msg.instrument_id = inst_id
        req.dt_from = dt_from.strftime(self.DT_FMT)
        req.dt_to = dt_to.strftime(self.DT_FMT)

        future = self.__cli_cdl.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)

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

        df[self.COL_NAME_MID_OP] = (
            df[self.COL_NAME_ASK_OP] + df[self.COL_NAME_BID_OP]) / 2
        df[self.COL_NAME_MID_HI] = (
            df[self.COL_NAME_ASK_HI] + df[self.COL_NAME_BID_HI]) / 2
        df[self.COL_NAME_MID_LO] = (
            df[self.COL_NAME_ASK_LO] + df[self.COL_NAME_BID_LO]) / 2
        df[self.COL_NAME_MID_CL] = (
            df[self.COL_NAME_ASK_CL] + df[self.COL_NAME_BID_CL]) / 2

        max_ = df[self.COL_NAME_MID_HI].max()
        min_ = df[self.COL_NAME_MID_LO].min()

        self.__series.clear()
        for time, sr in df.iterrows():
            o_ = sr[self.COL_NAME_MID_OP]
            h_ = sr[self.COL_NAME_MID_HI]
            l_ = sr[self.COL_NAME_MID_LO]
            c_ = sr[self.COL_NAME_MID_CL]
            t_ = QDateTime(dt.datetime.date(time))

            cnd = QtCharts.QCandlestickSet(
                o_, h_, l_, c_, t_.toMSecsSinceEpoch())
            self.__series.append(cnd)

        self.__chart.axisY().setRange(min_, max_)
        self.__chart.createDefaultAxes()


def main():

    rclpy.init()
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = GuiMonitor()

    widget.show()
    widget.init_resize_qchart()

    ros_th = threading.Thread(target=rclpy.spin, args=(widget.node,))
    ros_th.start()

    errcd = app.exec_()
    widget.node.destroy_node()
    rclpy.shutdown()
    sys.exit(errcd)


if __name__ == "__main__":
    main()
