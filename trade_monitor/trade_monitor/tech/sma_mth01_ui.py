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
from trade_apl_msgs.srv import TechChartMntSrv
from trade_monitor import ros_common as ros_com
from trade_monitor import utility as utl
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.constant import FMT_QT_DATE_YMD
from trade_monitor.constant import FMT_YMDHMS, FMT_DATE_YMD, FMT_DISP_YMDHMS
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.utility import DateRangeManager
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.constant import SMA_MTH01_CRS_TYP_DICT
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

        # --------------- Tree View ---------------
        # ----- SMA -----
        pdtreeview = PandasTreeView(ui.treeView)

        selmdl = pdtreeview.selectionModel()
        callback = self._on_selection_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview.header()
        callback = self._on_sma_header_sectionClicked
        header.sectionClicked.connect(callback)


        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]
        self._pdtreeview = pdtreeview

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

        # Create service client "tech_chart_monitor"
        srv_type = TechChartMntSrv
        srv_name = "tech_chart_monitor"
        fullname = ns + srv_name
        self._srv_chart_cli = ros_com.get_node().create_client(srv_type, fullname)

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

            # ---------- compose Table "SMA Method01 BackTest" ----------
            tbl = []
            for rec in rsp.result.tbl:
                record = [
                    dt.datetime.strptime(rec.en_datetime, FMT_YMDHMS),
                    dt.datetime.strptime(rec.ex_datetime, FMT_YMDHMS),
                    rec.cross_type,
                    rec.co_bs_h,
                    rec.co_bs_w,
                    rec.co_bs_hhw,
                    rec.co_tp_h,
                    rec.co_tp_w,
                    rec.co_tp_hhw,
                    rec.co_sma_h,
                    rec.co_sma_w,
                    rec.co_sma_hhw,
                    rec.area,
                    rec.profit_pips,
                ]
                tbl.append(record)
            df_bt = pd.DataFrame(tbl, columns=ColSmaMth01Bt.to_list())
            df_bt.set_index(ColSmaMth01Bt.EN_DATETIME.value,
                                     inplace=True)
            self._df_bt = df_bt

            # ---------- compose Table "SMA Method01 BackTest" for TreeView ----------
            if self._gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_DISP_YMDHMS
            tbl = []
            for idx, row in df_bt.iterrows():
                record = [
                    idx.strftime(fmt),
                    row[ColSmaMth01Bt.EX_DATETIME.value].strftime(fmt),
                    SMA_MTH01_CRS_TYP_DICT[int(row[ColSmaMth01Bt.CROSS_TYP.value])],
                    row[ColSmaMth01Bt.CO_BS_H.value],
                    row[ColSmaMth01Bt.CO_BS_W.value],
                    row[ColSmaMth01Bt.CO_BS_HHW.value],
                    row[ColSmaMth01Bt.CO_TP_H.value],
                    row[ColSmaMth01Bt.CO_TP_W.value],
                    row[ColSmaMth01Bt.CO_TP_HHW.value],
                    row[ColSmaMth01Bt.CO_SMA_H.value],
                    row[ColSmaMth01Bt.CO_SMA_W.value],
                    row[ColSmaMth01Bt.CO_SMA_HHW.value],
                    row[ColSmaMth01Bt.AREA.value],
                    row[ColSmaMth01Bt.PROFIT.value],
                ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=ColSmaMth01Bt.to_list())
            index = ColSmaMth01Bt.EN_DATETIME.value
            df.set_index(index, inplace=True)

            self._pdtreeview.set_dataframe(df)
            selmdl = self._pdtreeview.selectionModel()
            callback = self._on_selection_changed
            selmdl.selectionChanged.connect(callback)

    def _feedback_callback(self, msg):
        self.logger.debug("----- feedback -----")
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

    def _on_selection_changed(self, selected, _):
        self.logger.debug("----- _on_sma_selection_changed -----")
        if not selected.isEmpty():
            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("Service server [{}] not ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview.proxy
                dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
                if self._gran_param == GranParam.D:
                    fmt = FMT_DATE_YMD
                    dt_ = dt.datetime.strptime(dt_str, fmt)
                    lvl = self._df_bt.index.get_level_values(ColSmaMth01Bt.EN_DATETIME.value)
                    idx_dt = lvl[dt_ < lvl][0]
                else:
                    fmt = FMT_DISP_YMDHMS
                    idx_dt = dt.datetime.strptime(dt_str, fmt)

                self.logger.error("------------------------------------------")
                self.logger.error("dt_str:[{}]".format(dt_str))
                self.logger.error("idx_dt:[{}]".format(idx_dt))

                # self._draw_chart(dt_str, idx_dt)
                # self._ui.tableWidget_tech.setEnabled(True)

    """
    def _draw_chart(self, dt_str: str, idx_dt: dt.datetime):

        # fetch Tech chart data
        req = TechChartMntSrv.Request()
        req.datetime = idx_dt.strftime(FMT_YMDHMS)
        rsp = ros_com.call_servive_sync(self._srv_chart_cli, req)

        # ---------- compose Table "OHLC" ----------
        tbl = []
        for rec in rsp.tbl_ohlc:
            if self._gran_param == GranParam.D:
                dt_ = rec.datetime.split("T")[0]
            else:
                dt_ = utl.convert_ymdhms_fmt_to_disp(rec.datetime)
            record = [dt_,
                      rec.mid_o, rec.mid_h, rec.mid_l, rec.mid_c,
                      rec.sma_s, rec.sma_m, rec.sma_l,
                      ]
            tbl.append(record)

        columns = [ColOhlc.DATETIME.value] + self._ohlc_columns \
            + ColTrnd.to_list_all() + ColOsci.to_list_all()

        df_all = pd.DataFrame(tbl, columns=columns)

        index = ColOhlc.DATETIME.value
        df_all.set_index(index, inplace=True)
        df_all.where(df_all > -1000, inplace=True)

        columns = self._ohlc_columns + ColTrnd.to_list_all()
        df_trnd = df_all[columns]

        df_tmp = df_trnd.where(df_trnd > 0.0)
        max_y = df_tmp.max().max()
        min_y = df_tmp.min().min()

        self._target_datetime = df_trnd.index.get_loc(dt_str)
        self._df_all = df_all

        self._chartview_cndl.set_max_y(max_y)
        self._chartview_cndl.set_min_y(min_y)

        self._chartview_cndl.update(self._df_all[self._show_columns],
                                    self._target_datetime,
                                    self._inst_param)

        for osc_chart in self._osc_chart_list:
            df = self._df_all[osc_chart.df_columns]
            osc_chart.chartview.update(df,
                                       self._target_datetime,
                                       self._inst_param)
    """

    def _on_sma_header_sectionClicked(self, logical_index):
        self._pdtreeview.show_header_menu(logical_index)

    def resizeEvent(self, event):
        super().resizeEvent(event)
