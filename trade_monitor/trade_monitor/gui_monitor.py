import sys
import os
import threading

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import Qt, QFile, QTimer, QCoreApplication
from PySide2.QtUiTools import QUiLoader

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.client import Client
from std_msgs.msg import String, Bool
from trade_manager_msgs.srv import CandlesDataSrv
from trade_monitor.gapfill.gapfill_ui import GapFillUi
from trade_monitor.ttm.ttm_ui import TtmUi
from trade_monitor.tech.tech_ui import TechUi
from trade_monitor.main_ui import MainUi
from trade_monitor import utility as utl
# from trade_monitor.constant import INST_MSG_LIST
# from trade_monitor.constant import GRAN_MSG_LIST
from trade_monitor import ros_common as ros_com


class GuiMonitor(QMainWindow):

    def __init__(self, parent=None):
        super(GuiMonitor, self).__init__(parent)

        # --------------- initialize Qt ---------------
        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        """
        # set comboBox of Instrument
        utl.remove_all_items_of_comboBox(ui.comboBox_inst_main)
        for obj in INST_MSG_LIST:
            ui.comboBox_inst_main.addItem(obj.text)

        # set comboBox of Granularity
        utl.remove_all_items_of_comboBox(ui.comboBox_gran_main)
        for obj in GRAN_MSG_LIST:
            ui.comboBox_gran_main.addItem(obj.text)
        """

        # QTimer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timeout_1s)


        """
        ui.labe_srvcon_status.setAlignment(Qt.AlignCenter)

        callback = self._on_srvcon_toggled
        ui.pushButton_srvcon.toggled.connect(callback)
        """

        ui.tabWidget.currentChanged.connect(self._on_tab_changed)

        # --------------- initialize ROS ---------------
        node = rclpy.create_node("gui_monitor")
        self.logger = node.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub = node.create_subscription(String,
                                            "chatter",
                                            self.listener_callback,
                                            qos_profile)

        # Create service client "CandlesData"
        srv_type = CandlesDataSrv
        srv_name = "candles_data"
        cli_cdl = self._create_client(node, srv_type, srv_name)

        # Create publisher "Alive"
        msg_type = Bool
        topic = "alive"
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        pub_alive = node.create_publisher(msg_type, topic, qos_profile)

        ros_com.set_node(node)
        ros_com.set_service_client_candle(cli_cdl)

        self._ui = ui
        self._node = node
        # self._cli_cdl = cli_cdl
        self._pub_alive = pub_alive

        self._main_ui = MainUi(ui)
        self._gapfill_ui = GapFillUi(ui)
        self._ttm_ui = TtmUi(ui)
        self._tech_ui = TechUi(ui)

    def listener_callback(self, msg):
        self.logger.debug("----- ROS Callback!")
        self.logger.debug(msg.data)

    @property
    def node(self) -> Node:
        return self._node

    def _create_client(self,
                       node: Node,
                       srv_type,
                       srv_name: str
                       ) -> Client:

        cli = node.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for \"" + srv_name + "\" service...")

        return cli

    def _load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gui_monitor.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    """
    def _on_srvcon_toggled(self, flag):

        if flag is True:
            gran_idx = self._ui.comboBox_gran_main.currentIndex()
            inst_idx = self._ui.comboBox_inst_main.currentIndex()
            self._main_ui.draw_chart(inst_idx, gran_idx)
            self._ui.pushButton_srvcon.setText("切断")

            self._ui.labe_srvcon_status.setText("接続中")
            str_ = "background-color: rgb(0, 255, 0);"
            self._ui.labe_srvcon_status.setStyleSheet(str_)

            self.timer.start(1000)
        else:
            self._ui.pushButton_srvcon.setText("接続")

            self._ui.labe_srvcon_status.setText("切断")
            str_ = "background-color: rgb(136, 138, 133);"
            self._ui.labe_srvcon_status.setStyleSheet(str_)

            self.timer.stop()
    """

    def init_resize_qchart(self) -> None:
        self._main_ui.resize_chart_widget()

    def resizeEvent(self, event):
        index = self._ui.tabWidget.currentIndex()
        self._resize_chart_widget(index)

    def _on_tab_changed(self, index):
        self._resize_chart_widget(index)

    def _resize_chart_widget(self, tab_index):
        if tab_index == 0:
            self._main_ui.resize_chart_widget()
        elif tab_index == 1:
            pass
        elif tab_index == 2:
            pass

    def _on_timeout_1s(self) -> None:
        pass
        """
        if self._ui.pushButton_srvcon.isChecked():
            self.logger.debug("publish start")
        else:
            self.logger.debug("publish stop")

        msg = Bool()
        msg.data = True
        self._pub_alive.publish(msg)
        """


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
