import sys
import gc
import pandas as pd
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.sma01_ui import Sma01Ui
from trade_apl_msgs.action import TechBb01BtAct


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


class TechUi():

    def __init__(self, ui) -> None:
        self.logger = ros_com.get_logger()

        # ---------- set Instrument ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        # ---------- set Granularity ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_tech_gran)
        for obj in VALID_GRAN_LIST:
            ui.comboBox_tech_gran.addItem(obj.text)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        # ---------- set SMA ----------
        callback = self._on_tech_sma01_details_clicked
        ui.pushButton_tech_sma_mth01_details.clicked.connect(callback)

        # ---------- set Bollinger Bands ----------
        callback = self._on_pushButton_TechBb01_analy_start_clicked
        ui.pushButton_TechBb01_analy_start.clicked.connect(callback)

        # ---------- set Status Bar ----------
        self._sts_bar = StatusProgressBar(ui.statusbar)

        # ---------- set field ----------
        self._act_cli_bb01_bt = None
        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]

        self._init_ros_service()

        self._sma_mth01_ui = Sma01Ui()

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._init_ros_service()

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._init_ros_service()

    def _init_ros_service(self):
        ns = self._inst_param.namespace + "/" + self._gran_param.namespace + "/"

        if isinstance(self._act_cli_bb01_bt, ActionClient):
            self._act_cli_bb01_bt.destroy()

        # Create action client "TechnicalSmaMethod01BackTest"
        node = ros_com.get_node()
        act_type = TechBb01BtAct
        act_name = "tech_bb01_backtest"
        fullname = ns + act_name
        self._act_cli_bb01_bt = ActionClient(node, act_type, fullname)

    def _on_tech_sma01_details_clicked(self):
        self._sma_mth01_ui.set_data(self._inst_param, self._gran_param)
        self._sma_mth01_ui.show()

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
        self._ui.pushButton_TechBb01_analy_start.setEnabled(True)
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.debug("goal rejected")
            return

        self._sts_bar.set_bar_value(100)
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._TechBb01_get_result_callback
        self._result_future.add_done_callback(callback)

    def _TechBb01_get_result_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._sts_bar.set_bar_value(100)
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("STATUS_SUCCEEDED")

            # ---------- compose Table "SMA Method01 BackTest" ----------
            """
            tbl = []
            for rec in rsp.result.tbl:
                record = [
                    rec.sma_l,
                    rec.take_profit_th,
                    rec.stop_loss_th,
                    dt.datetime.strptime(rec.en_datetime, FMT_YMDHMS),
                    rec.en_price,
                    dt.datetime.strptime(rec.ex_datetime, FMT_YMDHMS),
                    rec.cross_type,
                    rec.co_sma_hhw,
                    rec.profit_pips
                ]
                tbl.append(record)
            df_bt = pd.DataFrame(tbl, columns=ColSma01Bt.to_list())
            idx_columns = [
                ColSma01Bt.SMA_L.value,
                ColSma01Bt.TAKE_PROFIT.value,
                ColSma01Bt.STOP_LOSS.value
            ]
            df_bt.set_index(idx_columns, inplace=True)
            self._df_bt = df_bt

            # ---------- compose Table "SMA Method01 BackTest" for TreeView ----------
            if self._gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_DISP_YMDHMS
            tbl = []
            for idxs, row in df_bt.iterrows():
                record = [
                    idxs[0],
                    idxs[1],
                    idxs[2],
                    row[ColSma01Bt.EN_DATETIME.value].strftime(fmt),
                    utl.roundf(row[ColSma01Bt.EN_PRICE.value], digit=self._inst_param.digit),
                    row[ColSma01Bt.EX_DATETIME.value].strftime(fmt),
                    SMA_MTH01_CRS_TYP_DICT[int(row[ColSma01Bt.CROSS_TYP.value])],
                    row[ColSma01Bt.CO_SMA_HHW.value],
                    row[ColSma01Bt.PROFIT.value]
                ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=ColSma01Bt.to_list())
            df.set_index(idx_columns, inplace=True)

            self._pdtreeview.set_dataframe(df)
            selmdl = self._pdtreeview.selectionModel()
            callback = self._on_selection_changed
            selmdl.selectionChanged.connect(callback)

            # self._draw_graph()
            self._ui.widget_graph.setEnabled(True)
            self._ui.pushButton_csv_out.setEnabled(True)
            """

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
