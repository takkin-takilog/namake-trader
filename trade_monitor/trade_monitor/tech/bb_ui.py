import sys
import gc
from enum import Enum, IntEnum
import pandas as pd
import datetime as dt
from dataclasses import dataclass
from PySide2.QtCore import Qt
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from trade_monitor.constant import FMT_YMDHMS, FMT_DISP_YMDHMS
from trade_monitor.constant import SPREAD_MSG_LIST
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.sma01_ui import Sma01Ui
from trade_apl_msgs.action import TechBb01BtAct
from trade_apl_msgs.action import TechBb01TreeViewAct


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


@dataclass
class Result():
    """
    Result.
    """
    sma_th: int         # SMA Th
    std_th: float       # STD Th
    df: pd.DataFrame    # DataFrame


class ColBtRslt(Enum):
    """
    Pandas SMA back test result dataframe column name.
    """
    ENTRY_TIME = "entry_time"
    ENTRY_PRICE = "entry_price"
    ENTRY_DIR = "entry_dir"
    ENTRY_STD_SLOP = "entry_std_slope"
    GAP_STD_SMA_PIPS = "gap_std_sma"
    MAX_LOSS_PIPS = "max_loss"
    EXIT_TIME = "exit_time"
    EXIT_PRICE = "exit_price"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class BollingerBandUi():

    def __init__(self, ui, inst_param, gran_param, sts_bar) -> None:
        self.logger = ros_com.get_logger()

        # ---------- set pushButton analy start ----------
        callback = self._on_pushButton_TechBb01_analy_start_clicked
        ui.pushButton_TechBb01_analy_start.clicked.connect(callback)

        # ---------- set pushButton fetch treeView ----------
        callback = self._on_pushButton_TechBb01_fetch_treeView_clicked
        ui.pushButton_TechBb01_fetch_treeView.clicked.connect(callback)

        # ---------- set TreeView ----------
        self._pdtreeview = PandasTreeView(ui.widget_TreeView_TechBb01)

        header = self._pdtreeview.header()
        callback = self._on_view_header_bb01_sectionClicked
        header.sectionClicked.connect(callback)

        # ---------- set comboBox Ask,Mid,Bid ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_TechBb01_amb)
        for text in SPREAD_MSG_LIST:
            ui.comboBox_TechBb01_amb.addItem(text)

        callback = self._on_comboBox_TechBb01_amb_changed_currentIndexChanged
        ui.comboBox_TechBb01_amb.currentIndexChanged.connect(callback)

        # ----- set widget disable -----
        ui.comboBox_TechBb01_sma.setEnabled(False)
        ui.comboBox_TechBb01_std.setEnabled(False)
        ui.pushButton_TechBb01_fetch_treeView.setEnabled(False)
        ui.widget_TreeView_TechBb01.setEnabled(False)
        ui.comboBox_TechBb01_amb.setEnabled(False)
        ui.widget_ChartView_TechBb01.setEnabled(False)

        # ---------- set field ----------
        self._act_cli_bb01_bt = None
        self._act_cli_bb01_tv = None
        self._ui = ui

        self._init_ros_service(inst_param, gran_param)

        self._inst_param = inst_param
        self._gran_param = gran_param
        self._sts_bar = sts_bar

    def update_inst_param(self, inst_param):
        self._init_ros_service(inst_param, self._gran_param)
        self._inst_param = inst_param

    def update_gran_param(self, gran_param):
        self._init_ros_service(self._inst_param, gran_param)
        self._gran_param = gran_param

    def _init_ros_service(self, inst_param, gran_param):
        ns = inst_param.namespace + "/" + gran_param.namespace + "/"

        if isinstance(self._act_cli_bb01_bt, ActionClient):
            self._act_cli_bb01_bt.destroy()

        if isinstance(self._act_cli_bb01_tv, ActionClient):
            self._act_cli_bb01_tv.destroy()

        # Create action client "TechnicalSmaMethod01BackTest"
        node = ros_com.get_node()
        act_type = TechBb01BtAct
        act_name = "tech_bb01_backtest"
        fullname = ns + act_name
        self._act_cli_bb01_bt = ActionClient(node, act_type, fullname)

        # Create action client "TechnicalSmaMethod01BackTest"
        node = ros_com.get_node()
        act_type = TechBb01TreeViewAct
        act_name = "tech_bb01_fetch_treeview"
        fullname = ns + act_name
        self._act_cli_bb01_tv = ActionClient(node, act_type, fullname)

    def _on_pushButton_TechBb01_analy_start_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli_bb01_bt.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
            return

        self._ui.pushButton_TechBb01_analy_start.setEnabled(False)

        self._sts_bar.set_label_text("Stanby...")
        self._sts_bar.set_bar_range(0, 100)
        self._sts_bar.set_bar_value(0)

        goal_msg = TechBb01BtAct.Goal()
        goal_msg.start_datetime = ""
        goal_msg.end_datetime = ""
        goal_msg.sma_th_start = self._ui.spinBox_TechBb01_SmaThStr.value()
        goal_msg.sma_th_end = self._ui.spinBox_TechBb01_SmaThEnd.value()
        goal_msg.sma_th_decimation = self._ui.spinBox_TechBb01_SmaThDeci.value()
        goal_msg.std_th_start = self._ui.spinBox_TechBb01_StdThStr.value()
        goal_msg.std_th_end = self._ui.spinBox_TechBb01_StdThEnd.value()
        goal_msg.std_th_decimation = self._ui.spinBox_TechBb01_StdThDeci.value()

        sma_rng = range(goal_msg.sma_th_start,
                        goal_msg.sma_th_end + 1,
                        goal_msg.sma_th_decimation)
        std_rng = range(goal_msg.std_th_start,
                        goal_msg.std_th_end + 1,
                        goal_msg.std_th_decimation)

        self._sma_len_max = len(sma_rng)
        self._std_len_max = len(std_rng)

        self._sma_pos = 0
        callback_fb = self._TechBb01_analy_feedback_callback
        self._future = self._act_cli_bb01_bt.send_goal_async(goal_msg,
                                                             callback_fb)

        callback = self._TechBb01_analy_goal_response_callback
        self._future.add_done_callback(callback)

    def _TechBb01_analy_feedback_callback(self, msg):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_label_text("Analyzing...[{}/{}][{}/{}]"
                                         .format(msg.feedback.sma_th_pos,
                                                 self._sma_len_max,
                                                 msg.feedback.std_th_pos,
                                                 self._std_len_max))
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

            if self._sma_pos != msg.feedback.sma_th_pos:
                gc.collect()
                self._sma_pos = msg.feedback.sma_th_pos

    def _TechBb01_analy_goal_response_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._TechBb01_analy_get_result_callback
        self._result_future.add_done_callback(callback)

    def _TechBb01_analy_get_result_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._sts_bar.set_bar_value(100)
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("STATUS_SUCCEEDED")
        else:
            return

        # ----- set SMA comboBox -----
        sma_th_start = self._ui.spinBox_TechBb01_SmaThStr.value()
        sma_th_end = self._ui.spinBox_TechBb01_SmaThEnd.value()
        sma_th_deci = self._ui.spinBox_TechBb01_SmaThDeci.value()
        sma_th_list = list(range(sma_th_start, sma_th_end + 1, sma_th_deci))

        wasBlocked = self._ui.comboBox_TechBb01_sma.blockSignals(True)
        utl.remove_all_items_of_comboBox(self._ui.comboBox_TechBb01_sma)
        for sma_th in sma_th_list:
            self._ui.comboBox_TechBb01_sma.addItem(str(sma_th))
        self._ui.comboBox_TechBb01_sma.blockSignals(wasBlocked)

        # ----- set STD comboBox -----
        std_th_start = self._ui.spinBox_TechBb01_StdThStr.value()
        std_th_end = self._ui.spinBox_TechBb01_StdThEnd.value()
        std_th_deci = self._ui.spinBox_TechBb01_StdThDeci.value()
        std_th_list = list(range(std_th_start, std_th_end + 1, std_th_deci))

        wasBlocked = self._ui.comboBox_TechBb01_std.blockSignals(True)
        utl.remove_all_items_of_comboBox(self._ui.comboBox_TechBb01_std)
        for std_th in std_th_list:
            self._ui.comboBox_TechBb01_std.addItem(str(std_th))
        self._ui.comboBox_TechBb01_std.blockSignals(wasBlocked)

        # ----- set widget enable -----
        self._ui.pushButton_TechBb01_analy_start.setEnabled(True)
        self._ui.comboBox_TechBb01_sma.setEnabled(True)
        self._ui.comboBox_TechBb01_std.setEnabled(True)
        self._ui.pushButton_TechBb01_fetch_treeView.setEnabled(True)
        self._ui.widget_TreeView_TechBb01.setEnabled(True)

        self._sma_th_list = sma_th_list
        self._std_th_list = std_th_list

    def _on_pushButton_TechBb01_fetch_treeView_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli_bb01_tv.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
            return

        self._ui.pushButton_TechBb01_fetch_treeView.setEnabled(False)

        self._sts_bar.set_label_text("Stanby...")
        self._sts_bar.set_bar_range(0, 100)
        self._sts_bar.set_bar_value(0)

        sma_idx = self._ui.comboBox_TechBb01_sma.currentIndex()
        std_idx = self._ui.comboBox_TechBb01_std.currentIndex()

        goal_msg = TechBb01TreeViewAct.Goal()
        goal_msg.sma_th = self._sma_th_list[sma_idx]
        goal_msg.std_th = self._std_th_list[std_idx]

        self.logger.debug("**********************************************")
        self.logger.debug("goal_msg.sma_th:{}".format(goal_msg.sma_th))
        self.logger.debug("goal_msg.std_th:{}".format(goal_msg.std_th))

        callback_fb = self._TechBb01_fetch_treeview_feedback_callback
        self._future = self._act_cli_bb01_tv.send_goal_async(goal_msg,
                                                             callback_fb)

        callback = self._TechBb01_fetch_treeview_goal_response_callback
        self._future.add_done_callback(callback)

    def _TechBb01_fetch_treeview_feedback_callback(self, msg):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_label_text("Fetching...")
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

    def _TechBb01_fetch_treeview_goal_response_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._TechBb01_fetch_treeview_get_result_callback
        self._result_future.add_done_callback(callback)

    def _TechBb01_fetch_treeview_get_result_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._sts_bar.set_bar_value(100)
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("STATUS_SUCCEEDED")
        else:
            return

        tbl = []
        for rec in rsp.result.tbl:
            record = [
                dt.datetime.strptime(rec.entry_time, FMT_YMDHMS),
                utl.roundf(rec.entry_price, digit=self._inst_param.digit),
                rec.entry_dir,
                "{:.5f}".format(rec.entry_std_slope),
                self._inst_param.convert_phy2raw(rec.gap_std_sma),
                self._inst_param.convert_phy2raw(rec.max_loss),
                dt.datetime.strptime(rec.exit_time, FMT_YMDHMS),
                utl.roundf(rec.exit_price, digit=self._inst_param.digit)
            ]
            tbl.append(record)
        df = pd.DataFrame(tbl, columns=ColBtRslt.to_list())
        df.set_index(ColBtRslt.ENTRY_TIME.value, inplace=True)

        self._update_treeview(df)

        # ----- set widget enable -----
        self._ui.pushButton_TechBb01_fetch_treeView.setEnabled(True)
        self._ui.comboBox_TechBb01_amb.setEnabled(True)
        self._ui.widget_ChartView_TechBb01.setEnabled(True)

    def _update_treeview(self, df: pd.DataFrame):

        self.logger.debug("\n{}".format(df))

        # ---------- compose Table for TreeView ----------
        tbl = []
        for t in df.itertuples():
            record = [
                t.Index.strftime(FMT_DISP_YMDHMS),
                t.entry_price,
                t.entry_dir,
                t.entry_std_slope,
                t.gap_std_sma,
                t.max_loss,
                t.exit_time.strftime(FMT_DISP_YMDHMS),
                t.exit_price
            ]
            tbl.append(record)
        df = pd.DataFrame(tbl, columns=ColBtRslt.to_list())
        df.set_index(ColBtRslt.ENTRY_TIME.value, inplace=True)
        self._pdtreeview.set_dataframe(df)

        selmdl = self._pdtreeview.selectionModel()
        callback = self._on_selection_bb01_changed
        selmdl.selectionChanged.connect(callback)

        """
        # self._draw_graph()
        self._ui.widget_graph.setEnabled(True)
        self._ui.pushButton_csv_out.setEnabled(True)
        """

    def _on_selection_bb01_changed(self, selected, _):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if not selected.isEmpty():

            model_index = selected.at(0).indexes()[0]
            r = model_index.row()
            proxy = self._pdtreeview.proxy
            date_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
            self.logger.debug(" - selected date:{}".format(date_str))

    """
    def _draw_graph(self):
        # is_selected = self._pdtreeview.is_selected()
        # df = self._pdtreeview.get_dataframe(is_selected=is_selected)
        df = self._pdtreeview.get_dataframe()

        series_cumsum = df[ColSma01Bt.PROFIT.value].cumsum()
        series_cumsum.rename("profit_cumsum", inplace=True)

        df = pd.concat([df, series_cumsum], axis=1)

        # self.logger.debug("\n{}".format(df))
        min_ = series_cumsum.min()
        end = series_cumsum[-1]
        self.logger.debug("series_cumsum_min:{}".format(min_))
        self.logger.debug("series_cumsum_tail:{}".format(end))
        self.logger.debug("real_profit:{}".format(end - min_))
    """

    def _on_view_header_bb01_sectionClicked(self, logical_index):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._pdtreeview.show_header_menu(logical_index)

    def _on_comboBox_TechBb01_amb_changed_currentIndexChanged(self, index):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
