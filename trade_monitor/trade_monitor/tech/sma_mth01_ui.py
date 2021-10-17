import os
import sys
import gc
import pandas as pd
import datetime as dt
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from PySide2.QtCore import Qt
from PySide2.QtWidgets import QToolButton, QFileDialog, QMessageBox
from trade_apl_msgs.action import TechSma01BtAct
from trade_apl_msgs.srv import TechChartMntSrv
from trade_monitor import ros_common as ros_com
from trade_monitor import utility as utl
from trade_monitor.widget_base import PandasTreeView
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.constant import FMT_YMDHMS, FMT_DATE_YMD, FMT_DISP_YMDHMS
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
# from trade_monitor.tech.constant import SMA_MTH01_CRS_TYP_DICT
from trade_monitor.tech.constant import ColSma01Bt
from trade_monitor.tech.constant import ColOhlc, ColTrnd, SpreadTyp
from trade_monitor.tech.widget import BaseUi
from trade_monitor.tech.widget import CandlestickSmaChartView as CandlestickChartView


class SmaMethod01Ui(BaseUi):

    def __init__(self, parent=None):
        super().__init__(parent)

        self._OHLC_COLUMNS = [CandlestickChartView.CandleLabel.OP.value,
                              CandlestickChartView.CandleLabel.HI.value,
                              CandlestickChartView.CandleLabel.LO.value,
                              CandlestickChartView.CandleLabel.CL.value
                              ]

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

        utl.remove_all_items_of_comboBox(ui.comboBox_spread)
        for text in SpreadTyp.to_list():
            ui.comboBox_spread.addItem(text)
        ui.comboBox_spread.currentIndexChanged.connect(
            self._comboBox_spread_changed)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        callback = self._on_analysis_start_clicked
        ui.pushButton_analysis_start.clicked.connect(callback)

        callback = self._on_csv_out_clicked
        ui.pushButton_csv_out.clicked.connect(callback)

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

        # --------------- Candlestick Chart View ---------------
        chartview = CandlestickChartView(ui.widget_chartView)

        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]
        self._pdtreeview = pdtreeview
        self._spread_idx = 0
        self._chartview = chartview
        self._is_chart_drawing = False

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
        act_type = TechSma01BtAct
        act_name = "tech_sma_method01_backtest"
        fullname = ns + act_name
        self._act_cli = ActionClient(node, act_type, fullname)

        # Create service client "tech_chart_monitor"
        srv_type = TechChartMntSrv
        srv_name = "tech_chart_monitor"
        fullname = ns + srv_name
        self._srv_chart_cli = ros_com.get_node().create_client(srv_type, fullname)

    def _on_analysis_start_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._act_cli.server_is_ready():
            self.logger.error("Action server [{}][{}] not ready"
                              .format(inst_param.text, gran_param.text))
        else:
            self._sts_bar.set_label_text("Stanby...")
            self._sts_bar.set_bar_range(0, 100)
            self._sts_bar.set_bar_value(0)

            goal_msg = TechSma01BtAct.Goal()
            goal_msg.start_datetime = ""
            goal_msg.end_datetime = ""
            goal_msg.sma_l_th_start = self._ui.spinBox_SmaLngThStr.value()
            goal_msg.sma_l_th_end = self._ui.spinBox_SmaLngThEnd.value()
            goal_msg.take_profit_th_start_pips = self._ui.spinBox_takeProfitThStr.value()
            goal_msg.take_profit_th_end_pips = self._ui.spinBox_takeProfitThEnd.value()
            goal_msg.stop_loss_th_start_pips = self._ui.spinBox_stopLossThStr.value()
            goal_msg.stop_loss_th_end_pips = self._ui.spinBox_stopLossThEnd.value()
            goal_msg.decimation = self._ui.spinBox_decimation.value()
            goal_msg.valid_th_pips = self._ui.spinBox_valid_th.value()

            sma_l_rng = range(goal_msg.sma_l_th_start,
                              goal_msg.sma_l_th_end + 1,
                              goal_msg.decimation)
            tp_th_pips_rng = range(goal_msg.take_profit_th_start_pips,
                                   goal_msg.take_profit_th_end_pips + 1,
                                   goal_msg.decimation)
            sl_th_pips_rng = range(goal_msg.stop_loss_th_start_pips,
                                   goal_msg.stop_loss_th_end_pips + 1,
                                   goal_msg.decimation)

            self._sma_l_len_max = len(sma_l_rng)
            self._tpsl_len_max = len(tp_th_pips_rng) * len(sl_th_pips_rng)

            self._sma_l_pos = 0
            feedback_callback = self._feedback_callback
            self._future = self._act_cli.send_goal_async(goal_msg,
                                                         feedback_callback)

            callback = self._goal_response_callback
            self._future.add_done_callback(callback)

    def _goal_response_callback(self, future):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
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

    def _feedback_callback(self, msg):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        rsp = self._future.result()
        if rsp.status == GoalStatus.STATUS_EXECUTING:
            self._sts_bar.set_label_text("Analyzing...[{}/{}][{}/{}]"
                                         .format(msg.feedback.sma_l_pos,
                                                 self._sma_l_len_max,
                                                 msg.feedback.tpsl_pos,
                                                 self._tpsl_len_max))
            self._sts_bar.set_bar_value(msg.feedback.progress_rate)

            if self._sma_l_pos != msg.feedback.sma_l_pos:
                gc.collect()
                self._sma_l_pos = msg.feedback.sma_l_pos

    def _on_selection_changed(self, selected, _):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        if not selected.isEmpty():
            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("Service server [{}] not ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview.proxy
                target_dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
                if self._gran_param == GranParam.D:
                    fmt = FMT_DATE_YMD
                    dt_ = dt.datetime.strptime(target_dt_str, fmt)
                    lvl = self._df_bt.index.get_level_values(ColSma01Bt.EN_DATETIME.value)
                    idx_dt = lvl[dt_ < lvl][0]
                else:
                    fmt = FMT_DISP_YMDHMS
                    idx_dt = dt.datetime.strptime(target_dt_str, fmt)

                self._df_ohlc = self._fech_ohlc(idx_dt)
                self._draw_chart(self._spread_idx, target_dt_str)
                self._target_dt_str = target_dt_str

                self._ui.widget_chartView.setEnabled(True)

    """
    def _fech_ohlc(self, idx_dt: dt.datetime):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        # fetch Tech chart data
        req = TechChartMntSrv.Request()
        req.datetime = idx_dt.strftime(FMT_YMDHMS)
        req.half_num_of_candles = 20
        rsp = ros_com.call_servive_sync(self._srv_chart_cli, req)

        tbl = []
        for rec in rsp.tbl_ohlc:
            record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                      rec.ask_o, rec.ask_h, rec.ask_l, rec.ask_c,
                      rec.mid_o, rec.mid_h, rec.mid_l, rec.mid_c,
                      rec.bid_o, rec.bid_h, rec.bid_l, rec.bid_c,
                      rec.sma_s, rec.sma_m, rec.sma_l
                      ]
            tbl.append(record)

        columns = ColOhlc.to_list() + [
            ColTrnd.SMA_S.value, ColTrnd.SMA_M.value, ColTrnd.SMA_L.value
        ]
        df = pd.DataFrame(tbl, columns=columns)
        index = ColOhlc.DATETIME.value
        df.set_index(index, inplace=True)
        df.where(df > 0.0, inplace=True)

        self._max_y = df.max().max()
        self._min_y = df.min().min()

        return df
    """

    def _draw_chart(self,
                    spread_idx: int,
                    dt_str: str):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if self._gran_param == GranParam.D:
            fmt = FMT_DATE_YMD
        else:
            fmt = FMT_DISP_YMDHMS
        tbl = []
        for idx, row in self._df_ohlc.iterrows():
            if spread_idx == SpreadTyp.get_index(SpreadTyp.MID):
                ohlc_rec = [
                    utl.roundf(row[ColOhlc.MID_O.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.MID_H.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.MID_L.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.MID_C.value], digit=self._inst_param.digit)
                ]
            elif spread_idx == SpreadTyp.get_index(SpreadTyp.ASK):
                ohlc_rec = [
                    utl.roundf(row[ColOhlc.ASK_O.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.ASK_H.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.ASK_L.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.ASK_C.value], digit=self._inst_param.digit)
                ]
            elif spread_idx == SpreadTyp.get_index(SpreadTyp.BID):
                ohlc_rec = [
                    utl.roundf(row[ColOhlc.BID_O.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.BID_H.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.BID_L.value], digit=self._inst_param.digit),
                    utl.roundf(row[ColOhlc.BID_C.value], digit=self._inst_param.digit)
                ]
            else:
                pass

            record = [idx.strftime(fmt)] + ohlc_rec + [
                utl.roundf(row[ColTrnd.SMA_S.value], digit=self._inst_param.digit),
                utl.roundf(row[ColTrnd.SMA_M.value], digit=self._inst_param.digit),
                utl.roundf(row[ColTrnd.SMA_L.value], digit=self._inst_param.digit)
            ]
            tbl.append(record)

        columns = [ColOhlc.DATETIME.value] + self._OHLC_COLUMNS \
            + [ColTrnd.SMA_S.value, ColTrnd.SMA_M.value, ColTrnd.SMA_L.value]

        df = pd.DataFrame(tbl, columns=columns)

        index = ColOhlc.DATETIME.value
        df.set_index(index, inplace=True)

        target_datetime = df.index.get_loc(dt_str)
        self._chartview.set_max_y(self._max_y)
        self._chartview.set_min_y(self._min_y)
        self._chartview.update(df,
                               target_datetime,
                               self._inst_param)

        self._is_chart_drawing = True

    def _on_sma_header_sectionClicked(self, logical_index):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))
        self._pdtreeview.show_header_menu(logical_index)
        self._draw_graph()

    def _comboBox_spread_changed(self, idx):
        self.logger.debug("----- Call \"{}\"".format(sys._getframe().f_code.co_name))

        if self._is_chart_drawing:
            self._draw_chart(idx, self._target_dt_str)

        self._spread_idx = idx

    def _on_csv_out_clicked(self):

        file_name, filter = QFileDialog.getSaveFileName(caption="Save file",
                                                        dir=os.path.expanduser("~"),
                                                        filter="*.csv")

        if (file_name != ""):
            df = self._pdtreeview.get_dataframe()
            df.to_csv(file_name + ".csv")

    def resizeEvent(self, event):
        super().resizeEvent(event)
