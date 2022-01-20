import sys
import gc
from enum import Enum
import pandas as pd
import datetime as dt
from dataclasses import dataclass
from PySide2.QtCore import QDateTime, Qt
from PySide2.QtWidgets import QAbstractItemView
from rclpy.action import ActionClient
from rclpy.client import Client
from action_msgs.msg import GoalStatus
from trade_apl_msgs.action import TechSmaBtAct
from trade_apl_msgs.action import TechSmaTreeViewAct
from trade_apl_msgs.srv import PeriodSrv
from trade_apl_msgs.srv import TechSmaChartSrv
from trade_monitor.constant import FMT_YMDHMS, FMT_DISP_YMDHMS, FMT_QT_YMDHMS
from trade_monitor.constant import SPREAD_MSG_LIST
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.tech.sma.constant import ColChart
from trade_monitor.tech.sma.widget import CandlestickChartView as ChartView
from trade_monitor.tech.sma.widget import ChartInfo


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
    MAX_HEIGHT_PIPS = "max_height_pips"
    EXIT_TIME = "exit_time"
    EXIT_PRICE = "exit_price"
    EXIT_PL_PIPS = "exit_pl_pips"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class SimpleMovingAverageUi():

    def __init__(self, ui, inst_param, gran_param, sts_bar) -> None:
        self.logger = ros_com.get_logger()

        # ---------- set comboBox Analysis Type ----------
        """
        utl.remove_all_items_of_comboBox(ui.comboBox_TechSma_AnalyTyp)
        for text in TRADE_TYP_LIST:
            ui.comboBox_TechSma_AnalyTyp.addItem(text)
        """

        # ---------- set pushButton analy start ----------
        callback = self._on_pushButton_backtest_start_clicked
        ui.pushButton_TechSma_backtest_start.clicked.connect(callback)

        # ---------- set pushButton period load ----------
        callback = self._on_pushButton_period_load_clicked
        ui.pushButton_TechSma_PeriodLoad.clicked.connect(callback)

        # ---------- set pushButton fetch treeView ----------
        callback = self._on_pushButton_TechSma_fetch_treeView_clicked
        ui.pushButton_TechSma_fetch_treeView.clicked.connect(callback)

        # ---------- set TreeView ----------
        self._pdtreeview = PandasTreeView(ui.widget_TreeView_TechSma)
        self._pdtreeview.setSelectionMode(QAbstractItemView.SingleSelection)

        header = self._pdtreeview.header()
        callback = self._on_view_header_sma_sectionClicked
        header.sectionClicked.connect(callback)

        # ---------- set comboBox Ask,Mid,Bid ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_TechSma_amb)
        for text in SPREAD_MSG_LIST:
            ui.comboBox_TechSma_amb.addItem(text)

        callback = self._on_comboBox_amb_changed_currentIndexChanged
        ui.comboBox_TechSma_amb.currentIndexChanged.connect(callback)

        # ---------- set spinBox barNum ----------
        callback = self._on_spinBox_barNum_valueChanged
        ui.spinBox_TechSma_barNum.valueChanged.connect(callback)

        # ----- set ChartView widget -----
        self._chartview = ChartView(ui.widget_ChartView_TechSma)

        # ----- set widget disable -----
        ui.comboBox_TechSma_SmaLngSpan.setEnabled(False)
        ui.comboBox_TechSma_SmaShrSpan.setEnabled(False)
        ui.pushButton_TechSma_fetch_treeView.setEnabled(False)
        ui.widget_TreeView_TechSma.setEnabled(False)
        ui.comboBox_TechSma_amb.setEnabled(False)
        ui.spinBox_TechSma_barNum.setEnabled(False)
        ui.widget_ChartView_TechSma.setEnabled(False)

        # ---------- set field ----------
        self._enable_period = False
        self._selected_entry_time = None
        self._chart_info = None
        self._srv_cli_period = None
        self._srv_cli_chart = None
        self._act_cli_bt = None
        self._act_cli_tv = None
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

        if isinstance(self._srv_cli_period, Client):
            self._srv_cli_period.destroy()

        if isinstance(self._srv_cli_chart, Client):
            self._srv_cli_chart.destroy()

        if isinstance(self._act_cli_bt, ActionClient):
            self._act_cli_bt.destroy()

        if isinstance(self._act_cli_tv, ActionClient):
            self._act_cli_tv.destroy()

        node = ros_com.get_node()

        # Create service client "Period"
        srv_type = PeriodSrv
        srv_name = "tech_sma_period"
        fullname = ns + srv_name
        self._srv_cli_period = node.create_client(srv_type, fullname)

        # Create service client "TechSmaChart"
        srv_type = TechSmaChartSrv
        srv_name = "tech_sma_fetch_chart"
        fullname = ns + srv_name
        self._srv_cli_chart = node.create_client(srv_type, fullname)

        # Create action client "TechSmaBackTest"
        act_type = TechSmaBtAct
        act_name = "tech_sma_backtest"
        fullname = ns + act_name
        self._act_cli_bt = ActionClient(node, act_type, fullname)

        # Create action client "TechSmaTreeView"
        act_type = TechSmaTreeViewAct
        act_name = "tech_sma_fetch_treeview"
        fullname = ns + act_name
        self._act_cli_tv = ActionClient(node, act_type, fullname)

    def _on_pushButton_backtest_start_clicked(self):
        self._start_backtest()

    def _on_pushButton_period_load_clicked(self):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if not self._srv_cli_period.service_is_ready():
            self.logger.error("Service server [{}] not ready"
                              .format(self._inst_param.text))
            return

        req = PeriodSrv.Request()
        rsp = ros_com.call_servive_sync(self._srv_cli_period, req)

        q_start_datetime = QDateTime.fromString(rsp.start_datetime, FMT_QT_YMDHMS)
        q_end_datetime = QDateTime.fromString(rsp.end_datetime, FMT_QT_YMDHMS)

        wasBlocked1 = self._ui.dateTimeEdit_TechSma_PeriodStr.blockSignals(True)
        wasBlocked2 = self._ui.dateTimeEdit_TechSma_PeriodEnd.blockSignals(True)

        self._ui.dateTimeEdit_TechSma_PeriodStr.setDateTimeRange(q_start_datetime,
                                                                 q_end_datetime)
        self._ui.dateTimeEdit_TechSma_PeriodStr.setDateTime(q_start_datetime)

        self._ui.dateTimeEdit_TechSma_PeriodEnd.setDateTimeRange(q_start_datetime,
                                                                 q_end_datetime)
        self._ui.dateTimeEdit_TechSma_PeriodEnd.setDateTime(q_end_datetime)

        self._ui.dateTimeEdit_TechSma_PeriodStr.blockSignals(wasBlocked1)
        self._ui.dateTimeEdit_TechSma_PeriodEnd.blockSignals(wasBlocked2)

        self._enable_period = True

    def _start_backtest(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli_bt.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
            return

        self._ui.pushButton_TechSma_backtest_start.setEnabled(False)

        self._sts_bar.set_label_text("Stanby...")
        self._sts_bar.set_bar_range(0, 100)
        self._sts_bar.set_bar_value(0)

        goal_msg = TechSmaBtAct.Goal()
        if self._enable_period:
            goal_msg.start_datetime = self._ui.dateTimeEdit_TechSma_PeriodStr.dateTime().toString(FMT_QT_YMDHMS)
            goal_msg.end_datetime = self._ui.dateTimeEdit_TechSma_PeriodEnd.dateTime().toString(FMT_QT_YMDHMS)
        else:
            goal_msg.start_datetime = ""
            goal_msg.end_datetime = ""
        goal_msg.sma_l_span_start = self._ui.spinBox_TechSma_SmaLngSpanStr.value()
        goal_msg.sma_l_span_end = self._ui.spinBox_TechSma_SmaLngSpanEnd.value()
        goal_msg.sma_l_span_deci = self._ui.spinBox_TechSma_SmaLngSpanDeci.value()
        goal_msg.sma_s_span_start = self._ui.spinBox_TechSma_SmaShrSpanStr.value()
        goal_msg.sma_s_span_end = self._ui.spinBox_TechSma_SmaShrSpanEnd.value()
        goal_msg.sma_s_span_deci = self._ui.spinBox_TechSma_SmaShrSpanDeci.value()
        goal_msg.valid_eval_th = self._ui.doubleSpinBox_TechSma_EvalTh.value()
        goal_msg.entry_offset_pips = self._ui.spinBox_TechSma_EntryOfs.value()

        sma_l_rng = range(goal_msg.sma_l_span_start,
                          goal_msg.sma_l_span_end + 1,
                          goal_msg.sma_l_span_deci)
        sma_s_rng = range(goal_msg.sma_s_span_start,
                          goal_msg.sma_s_span_end + 1,
                          goal_msg.sma_s_span_deci)

        self._sma_l_len_max = len(sma_l_rng)
        self._sma_s_len_max = len(sma_s_rng)

        self._sma_l_pos = 0
        callback_fb = self._backtest_feedback_callback
        self._future = self._act_cli_bt.send_goal_async(goal_msg, callback_fb)

        callback = self._backtest_goal_response_callback
        self._future.add_done_callback(callback)

    def _backtest_feedback_callback(self, msg):
        # self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            fb = msg.feedback
            if fb.sequence_num == 1:
                self._sts_bar.set_label_text("Analyzing...Seq[{}]:[{}/{}][{}/{}]"
                                             .format(fb.sequence_num,
                                                     fb.sma_l_span_pos,
                                                     self._sma_l_len_max,
                                                     fb.sma_s_span_pos,
                                                     self._sma_s_len_max))
                self._sts_bar.set_bar_value(fb.progress_rate)

                if self._sma_l_pos != fb.sma_l_span_pos:
                    gc.collect()
                    self._sma_l_pos = fb.sma_l_span_pos
            """
            elif fb.sequence_num == 2:
                self._sts_bar.set_label_text("Analyzing...Seq[{}]:[{}/{}]"
                                             .format(fb.sequence_num,
                                                     fb.profit_th_pos,
                                                     self._pl_len_max))
                self._sts_bar.set_bar_value(fb.progress_rate)

                if self._sma_pos != fb.sma_th_pos:
                    gc.collect()
                    self._sma_pos = fb.sma_th_pos
            """

    def _backtest_goal_response_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._backtest_get_result_callback
        self._result_future.add_done_callback(callback)

    def _backtest_get_result_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        self._sts_bar.set_bar_value(100)

        # ----- set widget enable -----
        self._ui.pushButton_TechSma_backtest_start.setEnabled(True)

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("GoalStatus:\"Succeeded\"")
        else:
            self.logger.debug("GoalStatus:\"Not Succeeded\"")
            return

        # ----- set SMA comboBox -----
        sma_l_span_start = self._ui.spinBox_TechSma_SmaLngSpanStr.value()
        sma_l_span_end = self._ui.spinBox_TechSma_SmaLngSpanEnd.value()
        sma_l_span_deci = self._ui.spinBox_TechSma_SmaLngSpanDeci.value()
        sma_l_span_list = list(range(sma_l_span_start, sma_l_span_end + 1, sma_l_span_deci))

        wasBlocked = self._ui.comboBox_TechSma_SmaLngSpan.blockSignals(True)
        utl.remove_all_items_of_comboBox(self._ui.comboBox_TechSma_SmaLngSpan)
        for sma_th in sma_l_span_list:
            self._ui.comboBox_TechSma_SmaLngSpan.addItem(str(sma_th))
        self._ui.comboBox_TechSma_SmaLngSpan.blockSignals(wasBlocked)

        # ----- set STD comboBox -----
        sma_s_span_start = self._ui.spinBox_TechSma_SmaShrSpanStr.value()
        sma_s_span_end = self._ui.spinBox_TechSma_SmaShrSpanEnd.value()
        sma_s_span_deci = self._ui.spinBox_TechSma_SmaShrSpanDeci.value()
        sma_s_span_list = list(range(sma_s_span_start, sma_s_span_end + 1, sma_s_span_deci))

        wasBlocked = self._ui.comboBox_TechSma_SmaShrSpan.blockSignals(True)
        utl.remove_all_items_of_comboBox(self._ui.comboBox_TechSma_SmaShrSpan)
        for std_th in sma_s_span_list:
            self._ui.comboBox_TechSma_SmaShrSpan.addItem(str(std_th))
        self._ui.comboBox_TechSma_SmaShrSpan.blockSignals(wasBlocked)

        # ----- set widget enable -----
        self._ui.comboBox_TechSma_SmaLngSpan.setEnabled(True)
        self._ui.comboBox_TechSma_SmaShrSpan.setEnabled(True)
        self._ui.pushButton_TechSma_fetch_treeView.setEnabled(True)
        self._ui.widget_TreeView_TechSma.setEnabled(True)

        self._sma_l_span_list = sma_l_span_list
        self._sma_s_span_list = sma_s_span_list

    def _on_pushButton_TechSma_fetch_treeView_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli_tv.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
            return

        self._ui.pushButton_TechSma_fetch_treeView.setEnabled(False)

        self._sts_bar.set_label_text("Stanby...")
        self._sts_bar.set_bar_range(0, 100)
        self._sts_bar.set_bar_value(0)

        sma_l_idx = self._ui.comboBox_TechSma_SmaLngSpan.currentIndex()
        sma_s_idx = self._ui.comboBox_TechSma_SmaShrSpan.currentIndex()

        goal_msg = TechSmaTreeViewAct.Goal()
        goal_msg.sma_l_span = self._sma_l_span_list[sma_l_idx]
        goal_msg.sma_s_span = self._sma_s_span_list[sma_s_idx]

        callback_fb = self._fetch_treeview_feedback_callback
        self._future = self._act_cli_tv.send_goal_async(goal_msg,
                                                        callback_fb)

        callback = self._fetch_treeview_goal_response_callback
        self._future.add_done_callback(callback)

    def _fetch_treeview_feedback_callback(self, msg):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_label_text("Fetching...")
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

    def _fetch_treeview_goal_response_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._fetch_treeview_get_result_callback
        self._result_future.add_done_callback(callback)

    def _fetch_treeview_get_result_callback(self, future):
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
                self._inst_param.round_pips(rec.entry_price),
                rec.entry_dir,
                rec.max_height_pips,
                dt.datetime.strptime(rec.exit_time, FMT_YMDHMS),
                self._inst_param.round_pips(rec.exit_price),
                rec.exit_pl_pips,
            ]
            tbl.append(record)
        df = pd.DataFrame(tbl, columns=ColBtRslt.to_list())
        df.set_index(ColBtRslt.ENTRY_TIME.value, inplace=True)

        self._update_treeview(df)
        self._df_tv = df

        # ----- set widget enable -----
        self._ui.pushButton_TechSma_fetch_treeView.setEnabled(True)
        self._ui.comboBox_TechSma_amb.setEnabled(True)
        self._ui.spinBox_TechSma_barNum.setEnabled(True)
        self._ui.widget_ChartView_TechSma.setEnabled(True)

    def _update_treeview(self, df: pd.DataFrame):

        self.logger.debug("\n{}".format(df))

        # ---------- compose Table for TreeView ----------
        tbl = []
        for t in df.itertuples():
            record = [
                t.Index.strftime(FMT_DISP_YMDHMS),
                t.entry_price,
                t.entry_dir,
                t.max_height_pips,
                t.exit_time.strftime(FMT_DISP_YMDHMS),
                t.exit_price,
                t.exit_pl_pips
            ]
            tbl.append(record)
        df = pd.DataFrame(tbl, columns=ColBtRslt.to_list())
        df.set_index(ColBtRslt.ENTRY_TIME.value, inplace=True)
        self._pdtreeview.set_dataframe(df)

        selmdl = self._pdtreeview.selectionModel()
        callback = self._on_selection_changed
        selmdl.selectionChanged.connect(callback)

        """
        # self._draw_graph_by_candle_type()
        self._ui.widget_graph.setEnabled(True)
        self._ui.pushButton_csv_out.setEnabled(True)
        """

    def _on_selection_changed(self, selected, _):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if selected.isEmpty():
            return

        model_index = selected.at(0).indexes()[0]
        r = model_index.row()
        proxy = self._pdtreeview.proxy
        entry_time_disp_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
        self.logger.debug(" - selected date:{}".format(entry_time_disp_str))

        if not self._srv_cli_chart.service_is_ready():
            self.logger.error("Service server [{}] not ready"
                              .format(self._inst_param.text))
            return

        entry_time = dt.datetime.strptime(entry_time_disp_str, FMT_DISP_YMDHMS)
        bar_num = self._ui.spinBox_TechSma_barNum.value()
        self._draw_graph(entry_time, bar_num)
        self._selected_entry_time = entry_time

    def _draw_graph(self, entry_time: dt.datetime, bar_num: int):

        sma_l_idx = self._ui.comboBox_TechSma_SmaLngSpan.currentIndex()
        sma_s_idx = self._ui.comboBox_TechSma_SmaShrSpan.currentIndex()

        req = TechSmaChartSrv.Request()
        req.sma_l_span = self._sma_l_span_list[sma_l_idx]
        req.sma_s_span = self._sma_s_span_list[sma_s_idx]
        req.time = entry_time.strftime(FMT_YMDHMS)
        req.number_of_bars = bar_num

        rsp = ros_com.call_servive_sync(self._srv_cli_chart, req)

        tbl = []
        for msg in rsp.tbl:
            rec = [
                utl.convert_ymdhms_fmt_to_disp(msg.time),
                msg.ask_o, msg.ask_h, msg.ask_l, msg.ask_c,
                msg.mid_o, msg.mid_h, msg.mid_l, msg.mid_c,
                msg.bid_o, msg.bid_h, msg.bid_l, msg.bid_c,
                msg.sma_l, msg.sma_s
            ]
            tbl.append(rec)
        df = pd.DataFrame(tbl, columns=ColChart.to_list())
        df.set_index(ColChart.TIME.value, inplace=True)

        row = self._df_tv.loc[entry_time]
        entry_time_str = entry_time.strftime(FMT_DISP_YMDHMS)
        entry_time_loc = df.index.get_loc(entry_time_str)
        entry_price = row[ColBtRslt.ENTRY_PRICE.value]
        exit_time_str = row[ColBtRslt.EXIT_TIME.value].strftime(FMT_DISP_YMDHMS)
        if exit_time_str in df.index:
            exit_time_loc = df.index.get_loc(exit_time_str)
        else:
            exit_time_loc = -10000000
        exit_price = row[ColBtRslt.EXIT_PRICE.value]

        self._chart_info = ChartInfo(df=df,
                                     entry_time_str=entry_time_str,
                                     entry_time_loc=entry_time_loc,
                                     entry_price=entry_price,
                                     exit_time_str=exit_time_str,
                                     exit_time_loc=exit_time_loc,
                                     exit_price=exit_price)

        max_y = self._chart_info.df.max().max()
        min_y = self._chart_info.df.min().min()
        margin = (max_y - min_y) * 0.05
        self._chartview.set_max_y(max_y + margin)
        self._chartview.set_min_y(min_y - margin)

        smb_idx = self._ui.comboBox_TechSma_amb.currentIndex()
        self._draw_graph_by_candle_type(smb_idx)

    def _draw_graph_by_candle_type(self, smb_idx):
        """
        bb_col = [ColChart.BASE_SMA.value,
                  ColChart.POS_STD.value,
                  ColChart.NEG_STD.value]

        if smb_idx == 0:    # Mid
            col = [ColChart.MID_O.value,
                   ColChart.MID_H.value,
                   ColChart.MID_L.value,
                   ColChart.MID_C.value] + bb_col
        elif smb_idx == 1:  # Ask
            col = [ColChart.ASK_O.value,
                   ColChart.ASK_H.value,
                   ColChart.ASK_L.value,
                   ColChart.ASK_C.value] + bb_col
        else:               # Bid
            col = [ColChart.BID_O.value,
                   ColChart.BID_H.value,
                   ColChart.BID_L.value,
                   ColChart.BID_C.value] + bb_col

        df = self._chart_info.df[col]
        df.columns = ChartView.CandleLabel.to_list() + bb_col

        self._chartview.update(df,
                               self._chart_info,
                               self._gran_param,
                               self._inst_param)
        """

    def _on_view_header_sma_sectionClicked(self, logical_index):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._pdtreeview.show_header_menu(logical_index)

    def _on_comboBox_amb_changed_currentIndexChanged(self, index):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if self._chart_info is not None:
            self._draw_graph_by_candle_type(index)

    def _on_spinBox_barNum_valueChanged(self, value: int):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if self._selected_entry_time is not None:
            self._draw_graph(self._selected_entry_time, value)
