import sys
import os

import threading
import datetime as dt
import pandas as pd

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import Qt, QFile, QTimer, QCoreApplication
from PySide2.QtUiTools import QUiLoader

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from std_msgs.msg import String, Bool
from trade_manager_msgs.srv import CandlesMntSrv
from trade_monitor.candlestick_chart import CandlestickChart
from trade_monitor.gap_fill import GapFill
from trade_monitor.util import INST_MSG_LIST
from trade_monitor.util import GRAN_MSG_LIST
from trade_monitor.util import DT_FMT
from trade_monitor.util import CANDLE_COL_NAME_LIST
from trade_monitor.util import (COL_NAME_TIME,
                                COL_NAME_ASK_OP,
                                COL_NAME_ASK_HI,
                                COL_NAME_ASK_LO,
                                COL_NAME_ASK_CL,
                                COL_NAME_BID_OP,
                                COL_NAME_BID_HI,
                                COL_NAME_BID_LO,
                                COL_NAME_BID_CL,
                                COL_NAME_MID_OP,
                                COL_NAME_MID_HI,
                                COL_NAME_MID_LO,
                                COL_NAME_MID_CL
                                )


class GuiMonitor(QMainWindow):

    def __init__(self, parent=None):
        super(GuiMonitor, self).__init__(parent)

        # --------------- initialize Qt ---------------
        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        # set comboBox of Instrument
        self.__remove_all_items_of_comboBox(ui.comboBox_inst_main)
        for obj in INST_MSG_LIST:
            ui.comboBox_inst_main.addItem(obj.text)
            ui.comboBox_inst_gapfill.addItem(obj.text)

        # set comboBox of Granularity
        self.__remove_all_items_of_comboBox(ui.comboBox_gran_main)
        for obj in GRAN_MSG_LIST:
            ui.comboBox_gran_main.addItem(obj.text)

        # QTimer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.__on_timeout_1s)

        ui.labe_srvcon_status.setAlignment(Qt.AlignCenter)

        callback = self.__on_srvcon_toggled
        ui.pushButton_srvcon.toggled.connect(callback)

        ui.tabWidget.currentChanged.connect(self.__on_tab_changed)

        # ----- Main Tab -----
        callback = self.__on_cb_inst_main_changed
        ui.comboBox_inst_main.currentIndexChanged.connect(callback)
        callback = self.__on_cb_gran_main_changed
        ui.comboBox_gran_main.currentIndexChanged.connect(callback)

        csc_main = CandlestickChart(ui.widget_chart_main)
        #csc_gapfill = CandlestickChart(ui.widget_chart_gapfill)

        # --------------- initialize ROS ---------------
        node = rclpy.create_node('gui_monitor')
        self.logger = node.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.sub = node.create_subscription(String,
                                            'chatter',
                                            self.listener_callback)

        # Create service client "CandlesMonitor"
        srv_type = CandlesMntSrv
        srv_name = "candles_monitor"
        cli_cdl = self.__create_client(node, srv_type, srv_name)

        # Create publisher "HistoricalCandles"
        msg_type = Bool
        topic = "alive"
        pub_alive = node.create_publisher(msg_type, topic)

        self.__ui = ui
        self.__csc_main = csc_main
        self.__node = node
        self.__cli_cdl = cli_cdl
        self.__pub_alive = pub_alive

        self.__gapfill = GapFill(ui, node, cli_cdl)

    def listener_callback(self, msg):
        self.logger.debug("----- ROS Callback!")
        self.logger.debug(msg.data)

    @property
    def node(self) -> Node:
        return self.__node

    def __create_client(self,
                        node: Node,
                        srv_type,
                        srv_name: str
                        ) -> Client:

        cli = node.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for \"" + srv_name + "\" service...")

        return cli

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

    def __on_cb_inst_main_changed(self, inst_idx):
        gran_idx = self.__ui.comboBox_gran_main.currentIndex()
        self.__display_chart(inst_idx, gran_idx)

    def __on_cb_gran_main_changed(self, gran_idx):
        inst_idx = self.__ui.comboBox_inst_main.currentIndex()
        self.__display_chart(inst_idx, gran_idx)

    def __on_srvcon_toggled(self, flag):

        if flag is True:
            gran_idx = self.__ui.comboBox_gran_main.currentIndex()
            inst_idx = self.__ui.comboBox_inst_main.currentIndex()
            self.__display_chart(inst_idx, gran_idx)
            self.__ui.pushButton_srvcon.setText("切断")

            self.__ui.labe_srvcon_status.setText("接続中")
            str_ = "background-color: rgb(0, 255, 0);"
            self.__ui.labe_srvcon_status.setStyleSheet(str_)

            self.timer.start(1000)
        else:
            self.__ui.pushButton_srvcon.setText("接続")

            self.__ui.labe_srvcon_status.setText("切断")
            str_ = "background-color: rgb(136, 138, 133);"
            self.__ui.labe_srvcon_status.setStyleSheet(str_)

            self.timer.stop()

    def __on_tab_changed(self, index):
        self.__resize_chart_widget(index)

    def init_resize_qchart(self) -> None:
        fs = self.__ui.widget_chart_main.frameSize()
        self.__csc_main.resize(fs)

    def __resize_chart_widget(self, tab_index):
        if tab_index == 0:
            fs = self.__ui.widget_chart_main.frameSize()
            self.__csc_main.resize(fs)
        elif tab_index == 1:
            self.__gapfill.resize_chart_widget()

    def resizeEvent(self, event):
        index = self.__ui.tabWidget.currentIndex()
        self.__resize_chart_widget(index)

    def __display_chart(self, inst_idx, gran_idx):

        dt_now = dt.datetime.now()
        dt_from = dt_now - dt.timedelta(days=20)
        dt_to = dt_now

        inst_id = INST_MSG_LIST[inst_idx].msg_id
        gran_id = GRAN_MSG_LIST[gran_idx].msg_id

        req = CandlesMntSrv.Request()
        req.gran_msg.granularity_id = gran_id
        req.inst_msg.instrument_id = inst_id
        req.dt_from = dt_from.strftime(DT_FMT)
        req.dt_to = dt_to.strftime(DT_FMT)

        future = self.__cli_cdl.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)

        flg = future.done() and future.result() is not None
        assert flg, "initial fetch [Day Candle] failed!"

        data = []
        rsp = future.result()
        for cndl_msg in rsp.cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, DT_FMT)
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
        df.columns = CANDLE_COL_NAME_LIST
        df = df.set_index(COL_NAME_TIME)

        df[COL_NAME_MID_OP] = (df[COL_NAME_ASK_OP] + df[COL_NAME_BID_OP]) / 2
        df[COL_NAME_MID_HI] = (df[COL_NAME_ASK_HI] + df[COL_NAME_BID_HI]) / 2
        df[COL_NAME_MID_LO] = (df[COL_NAME_ASK_LO] + df[COL_NAME_BID_LO]) / 2
        df[COL_NAME_MID_CL] = (df[COL_NAME_ASK_CL] + df[COL_NAME_BID_CL]) / 2

        dftmp = df.loc[:, [COL_NAME_MID_OP,
                           COL_NAME_MID_HI,
                           COL_NAME_MID_LO,
                           COL_NAME_MID_CL
                           ]]
        dftmp.columns = [CandlestickChart.COL_NAME_OP,
                         CandlestickChart.COL_NAME_HI,
                         CandlestickChart.COL_NAME_LO,
                         CandlestickChart.COL_NAME_CL
                         ]

        self.__csc_main.update(dftmp, gran_id)

    def __on_timeout_1s(self) -> None:

        if self.__ui.pushButton_srvcon.isChecked():
            self.logger.debug("publish start")
        else:
            self.logger.debug("publish stop")

        msg = Bool()
        msg.data = True
        self.__pub_alive.publish(msg)


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
