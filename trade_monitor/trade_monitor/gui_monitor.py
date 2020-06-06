import sys
import os
import threading

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import Qt, QFile, QTimer, QCoreApplication
from PySide2.QtUiTools import QUiLoader

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from std_msgs.msg import String, Bool
from trade_manager_msgs.srv import CandlesMntSrv
from trade_monitor.gap_fill_ui import GapFillUi
from trade_monitor.main_ui import MainUi
from trade_monitor.util import INST_MSG_LIST
from trade_monitor.util import GRAN_MSG_LIST


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

        # Create publisher "Alive"
        msg_type = Bool
        topic = "alive"
        pub_alive = node.create_publisher(msg_type, topic)

        self.__ui = ui
        self.__node = node
        self.__cli_cdl = cli_cdl
        self.__pub_alive = pub_alive

        self.__main_ui = MainUi(ui, node, cli_cdl)
        self.__gapfill_ui = GapFillUi(ui, node, cli_cdl)

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

    def __on_srvcon_toggled(self, flag):

        if flag is True:
            gran_idx = self.__ui.comboBox_gran_main.currentIndex()
            inst_idx = self.__ui.comboBox_inst_main.currentIndex()
            self.__main_ui.draw_chart(inst_idx, gran_idx)
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

    def init_resize_qchart(self) -> None:
        self.__main_ui.resize_chart_widget()

    def resizeEvent(self, event):
        index = self.__ui.tabWidget.currentIndex()
        self.__resize_chart_widget(index)

    def __on_tab_changed(self, index):
        self.__resize_chart_widget(index)

    def __resize_chart_widget(self, tab_index):
        if tab_index == 0:
            self.__main_ui.resize_chart_widget()
        elif tab_index == 1:
            self.__gapfill_ui.resize_chart_widget()

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
