from enum import Enum, IntEnum, auto
import pandas as pd
import datetime as dt
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from PySide2.QtWidgets import QHeaderView
from PySide2.QtWidgets import QTableWidgetItem
from PySide2.QtCore import QDate, Qt
from PySide2.QtGui import QColor, QBrush
from trade_apl_msgs.action import TechSmaMth01BtAct
from trade_monitor import ros_common as ros_com
from trade_monitor import utility as utl
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.constant import FMT_QT_DATE_YMD
from trade_monitor.constant import FMT_YMDHMS, FMT_DATE_YMD, FMT_DISP_YMDHMS
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.utility import DateRangeManager
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.constant import ColSmaMth01Bt
from trade_monitor.tech.widget import BaseUi


class SmaMethod01Ui(BaseUi):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = ros_com.get_logger()

        ui = self._load_ui(parent, "sma_mth01.ui")
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())
        self.setWindowTitle("SMA Method01 Details")

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_gran)
        for obj in VALID_GRAN_LIST:
            ui.comboBox_tech_gran.addItem(obj.text)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        callback = self._on_analysis_start_clicked
        ui.pushButton_analysis_start.clicked.connect(callback)

        self._sts_bar = StatusProgressBar(ui.statusbar)

        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]

        self._init_ros_service()

    def set_data(self,
                 inst_param: InstParam,
                 gran_param: GranParam):
        self._inst_param = inst_param
        self._gran_param = gran_param
        inst_idx = VALID_INST_LIST.index(inst_param)
        gran_idx = VALID_GRAN_LIST.index(gran_param)
        self._ui.comboBox_tech_inst.setCurrentIndex(inst_idx)
        self._ui.comboBox_tech_gran.setCurrentIndex(gran_idx)

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._init_ros_service()

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._init_ros_service()

    def _init_ros_service(self):
        ns = self._inst_param.namespace + "/" + self._gran_param.namespace + "/"

        # Create action client "TechnicalSmaMethod01BackTest"
        node = ros_com.get_node()
        act_type = TechSmaMth01BtAct
        act_name = "tech_sma_method01_backtest"
        fullname = ns + act_name
        self._act_cli = ActionClient(node, act_type, fullname)

    def _on_analysis_start_clicked(self):

        self._sts_bar.set_label_text("Analyzing...")
        self._sts_bar.set_bar_range(0, 100)

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
        else:
            goal_msg = TechSmaMth01BtAct.Goal()
            goal_msg.start_datetime = ""
            goal_msg.end_datetime = ""
            goal_msg.take_profit_th_pips = self._ui.spinBox_TakeProfitTh.value()
            goal_msg.stop_loss_th_pips = self._ui.spinBox_StopLossTh.value()

            feedback_callback = self._feedback_callback
            self._future = self._act_cli.send_goal_async(goal_msg,
                                                         feedback_callback)

            callback = self._goal_response_callback
            self._future.add_done_callback(callback)

    def _goal_response_callback(self, future):
        # 目標値の設定成功の判別
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._get_result_callback
        self._result_future.add_done_callback(callback)

    def _get_result_callback(self, future):
        self._sts_bar.set_bar_value(100)
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("STATUS_SUCCEEDED")

            # ---------- compose Table "SMA Method01" ----------
            tbl = []
            for rec in rsp.result.tbl:
                record = [
                    dt.datetime.strptime(rec.en_datetime, FMT_YMDHMS),
                    dt.datetime.strptime(rec.ex_datetime, FMT_YMDHMS),
                    rec.profit_pips,
                ]
                tbl.append(record)
            df_sma_mth01bt = pd.DataFrame(tbl, columns=ColSmaMth01Bt.to_list())
            df_sma_mth01bt.set_index(ColSmaMth01Bt.EN_DATETIME.value,
                                     inplace=True)

            self._df_sma_mth01bt = df_sma_mth01bt
            self.logger.debug("----- df_sma_mth01bt -----\n{}"
                              .format(self._df_sma_mth01bt))

    def _feedback_callback(self, msg):
        self.logger.debug("----- feedback -----")
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

    def resizeEvent(self, event):
        super().resizeEvent(event)
