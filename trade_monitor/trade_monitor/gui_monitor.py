import sys
import os

import threading
import datetime as dt
import pandas as pd

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import Qt, QFile, QTimer, QCoreApplication
from PySide2.QtUiTools import QUiLoader
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from trade_apl_msgs.srv import GapFillMntSrv
from trade_apl_msgs.msg import GapFillMsg
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran
from std_msgs.msg import String, Bool

from trade_monitor.candlestick_chart import CandlestickChart
from trade_monitor.gap_fill import GapFill


class MsgGranDict():

    def __init__(self,
                 msg_id: int,
                 text: str
                 ) -> None:
        self.__msg_id = msg_id
        self.__text = text

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text


class MsgInstDict():

    def __init__(self,
                 msg_id: int,
                 text: str,
                 decimal_digit: int
                 ) -> None:
        self.__msg_id = msg_id
        self.__text = text
        self.__decimal_digit = decimal_digit

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text

    @property
    def decimal_digit(self) -> int:
        return self.__decimal_digit


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
        MsgInstDict(Inst.INST_USD_JPY, "USD/JPY", 3),
        MsgInstDict(Inst.INST_EUR_JPY, "EUR/JPY", 3),
        MsgInstDict(Inst.INST_EUR_USD, "EUR/USD", 5),
    ]

    GRAN_MSG_LIST = [
        MsgGranDict(Gran.GRAN_D, "日足"),
        MsgGranDict(Gran.GRAN_H4, "４時間足"),
        MsgGranDict(Gran.GRAN_H1, "１時間足"),
        MsgGranDict(Gran.GRAN_M10, "１０分足"),
    ]

    GAP_DIR_DICT = {
        GapFillMsg.GAP_DIR_UP: "Up",
        GapFillMsg.GAP_DIR_DOWN: "Down"
        }

    GAP_FILL_SUCC_DICT = {
        True: "Success",
        False: "Failure"
        }

    GAP_FILL_HEADERS = [
        "Date",
        "Gap dir",
        "Previous close price",
        "Current open price",
        "Gap range price",
        "Gap fill result",
        "Gap filled time",
        "Max open range",
        "End close price"
    ]

    def __init__(self, parent=None):
        super(GuiMonitor, self).__init__(parent)

        # --------------- initialize Qt ---------------
        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        # set comboBox of Instrument
        self.__remove_all_items_of_comboBox(ui.comboBox_inst_main)
        for obj in self.INST_MSG_LIST:
            ui.comboBox_inst_main.addItem(obj.text)
            ui.comboBox_inst_gapfill.addItem(obj.text)

        # set comboBox of Granularity
        self.__remove_all_items_of_comboBox(ui.comboBox_gran_main)
        for obj in self.GRAN_MSG_LIST:
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

        # ----- Gap-Fill Tab -----
        callback = self.__on_cb_inst_gapfill_changed
        ui.comboBox_inst_gapfill.currentIndexChanged.connect(callback)
        callback = self.__on_fetch_gapfill_clicked
        ui.pushButton_fetch_gapfill.clicked.connect(callback)

        tbl_mdl_gapfill = QStandardItemModel()
        self.selModel = QItemSelectionModel(tbl_mdl_gapfill)

        callback = self.__on_selection_gapfill_changed
        self.selModel.selectionChanged.connect(callback)

        # set header
        tbl_mdl_gapfill.setHorizontalHeaderLabels(self.GAP_FILL_HEADERS)
        ui.tableView_gapfill.setModel(tbl_mdl_gapfill)
        ui.treeView_gapfill.setModel(tbl_mdl_gapfill)
        ui.treeView_gapfill.setSelectionModel(self.selModel)

        csc_main = CandlestickChart(ui.widget_chart_main)
        csc_gapfill = CandlestickChart(ui.widget_chart_gapfill)

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

        # Create service client "CandlesMonitor"
        srv_type = GapFillMntSrv
        srv_name = "gapfill_monitor"
        cli_gf = self.__create_client(node, srv_type, srv_name)

        # Create publisher "HistoricalCandles"
        msg_type = Bool
        topic = "alive"
        pub_alive = node.create_publisher(msg_type, topic)

        self.__ui = ui
        self.__csc_main = csc_main
        self.__csc_gapfill = csc_gapfill
        self.__tbl_mdl_gapfill = tbl_mdl_gapfill
        self.__node = node
        self.__cli_cdl = cli_cdl
        self.__cli_gf = cli_gf
        self.__pub_alive = pub_alive

        self.__gapfill = GapFill(self.INST_MSG_LIST[0].msg_id)
        self.__end_hour = 9

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

    def __on_cb_inst_gapfill_changed(self, inst_idx):
        self.__gapfill.inst_id = self.INST_MSG_LIST[inst_idx].msg_id

    def __on_fetch_gapfill_clicked(self):

        self.logger.debug("gapfill start")

        self.__tbl_mdl_gapfill.clear()
        self.__tbl_mdl_gapfill.setHorizontalHeaderLabels(self.GAP_FILL_HEADERS)
        inst_id = self.__gapfill.inst_id
        decimal_digit = self.INST_MSG_LIST[inst_id].decimal_digit
        fmt = "{:." + str(decimal_digit) + "f}"

        # fetch Gap-fill data
        req = GapFillMntSrv.Request()
        req.inst_msg.instrument_id = inst_id

        future = self.__cli_gf.call_async(req)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)

        flg = future.done() and future.result() is not None
        assert flg, "fetch [Gap-Fill] failed!"

        rsp = future.result()
        for gapfillmsg in rsp.gapfillmsg_list:
            items = [
                QStandardItem(gapfillmsg.date),
                QStandardItem(self.GAP_DIR_DICT[gapfillmsg.gap_dir]),
                QStandardItem(fmt.format(gapfillmsg.gap_close_price)),
                QStandardItem(fmt.format(gapfillmsg.gap_open_price)),
                QStandardItem(fmt.format(gapfillmsg.gap_range_price)),
                QStandardItem(self.GAP_FILL_SUCC_DICT[
                    gapfillmsg.is_gapfill_success]),
                QStandardItem(gapfillmsg.gap_filled_time),
                QStandardItem(fmt.format(gapfillmsg.max_open_range)),
                QStandardItem(fmt.format(gapfillmsg.end_close_price))
                ]
            self.__tbl_mdl_gapfill.appendRow(items)

        self.__end_hour = rsp.end_hour

        # fetch Canclestick data
        """
        req = CandlesMntSrv.Request()
        req.gran_msg.granularity_id = Gran.GRAN_M10
        req.inst_msg.instrument_id = inst_id
        req.dt_from = "inf"
        req.dt_to = "inf"

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
        self.__df = df.set_index(self.COL_NAME_TIME)
        """

        self.logger.debug("gapfill end")

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
            fs = self.__ui.widget_chart_gapfill.frameSize()
            self.__csc_gapfill.resize(fs)

    def __on_selection_gapfill_changed(self, selected, deselected):

        gran_id = Gran.GRAN_M10
        model_index = selected.at(0).indexes()[0]
        trg_date_str = self.__tbl_mdl_gapfill.item(model_index.row()).text()
        self.logger.debug("target date: " + trg_date_str)

        trg_date = dt.datetime.strptime(trg_date_str, "%Y-%m-%d")

        dt_from = trg_date - dt.timedelta(days=2)
        dt_to = trg_date + dt.timedelta(hours=self.__end_hour)

        req = CandlesMntSrv.Request()
        req.gran_msg.granularity_id = gran_id
        req.inst_msg.instrument_id = self.__gapfill.inst_id
        req.dt_from = dt_from.strftime(self.DT_FMT)
        req.dt_to = dt_to.strftime(self.DT_FMT)

        self.logger.debug("dt_from: " + req.dt_from)
        self.logger.debug("dt_to: " + req.dt_to)

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

        dftmp = df.loc[:, [self.COL_NAME_ASK_OP,
                           self.COL_NAME_ASK_HI,
                           self.COL_NAME_ASK_LO,
                           self.COL_NAME_ASK_CL
                           ]]
        dftmp.columns = [CandlestickChart.COL_NAME_OP,
                         CandlestickChart.COL_NAME_HI,
                         CandlestickChart.COL_NAME_LO,
                         CandlestickChart.COL_NAME_CL
                         ]

        self.__csc_gapfill.update(dftmp, gran_id)


    def resizeEvent(self, event):
        index = self.__ui.tabWidget.currentIndex()
        self.__resize_chart_widget(index)

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

        dftmp = df.loc[:, [self.COL_NAME_MID_OP,
                           self.COL_NAME_MID_HI,
                           self.COL_NAME_MID_LO,
                           self.COL_NAME_MID_CL
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
