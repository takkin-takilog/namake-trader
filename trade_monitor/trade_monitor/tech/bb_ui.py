import sys
import gc
from enum import Enum, IntEnum
import pandas as pd
import datetime as dt
from dataclasses import dataclass
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from trade_monitor.constant import FMT_YMDHMS
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
    GAP_STD_SMA = "gap_std_sma"
    MAX_LOSS = "max_loss"
    EXIT_TIME = "exit_time"
    EXIT_PRICE = "exit_price"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class BollingerBandUi():

    def __init__(self, ui, inst_param, gran_param, sts_bar) -> None:
        self.logger = ros_com.get_logger()

        # ---------- set Bollinger Bands ----------
        callback = self._on_pushButton_TechBb01_analy_start_clicked
        ui.pushButton_TechBb01_analy_start.clicked.connect(callback)

        # ---------- set field ----------
        self._act_cli_bb01_bt = None
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

        # Create action client "TechnicalSmaMethod01BackTest"
        node = ros_com.get_node()
        act_type = TechBb01BtAct
        act_name = "tech_bb01_backtest"
        fullname = ns + act_name
        self._act_cli_bb01_bt = ActionClient(node, act_type, fullname)

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

        callback = self._TechBb01_get_result_callback
        self._result_future.add_done_callback(callback)

    def _TechBb01_get_result_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._sts_bar.set_bar_value(100)
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.debug("STATUS_SUCCEEDED")

            bt_rsl_tbl = []
            for rsltbl in rsp.result.tbl_list:
                tbl = []
                for rec in rsltbl.tbl:
                    record = [
                        dt.datetime.strptime(rec.entry_time, FMT_YMDHMS),
                        rec.entry_price,
                        rec.entry_dir,
                        rec.entry_std_slope,
                        rec.gap_std_sma,
                        rec.max_loss,
                        dt.datetime.strptime(rec.exit_time, FMT_YMDHMS),
                        rec.exit_price,
                    ]
                    tbl.append(record)
                df = pd.DataFrame(tbl, columns=ColBtRslt.to_list())
                df.set_index(ColBtRslt.ENTRY_TIME.value, inplace=True)
                bt_rsl_tbl.append(Result(rsltbl.sma_th, rsltbl.std_th, df))
                self.logger.debug("{}".format(bt_rsl_tbl[0]))

            """

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
        self._ui.pushButton_TechBb01_analy_start.setEnabled(True)

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
