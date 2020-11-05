import pandas as pd
import datetime as dt

from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

from trade_apl_msgs.srv import GapFillMntSrv
from trade_apl_msgs.msg import GapFillMsg
from trade_manager_msgs.srv import CandlesMntSrv

from trade_monitor import utilities as utl
from trade_monitor.utilities import SPREAD_MSG_LIST
from trade_monitor.utilities import INST_MSG_LIST
from trade_monitor.utilities import (FMT_DTTM_API,
                                     FMT_DATE_YMD,
                                     FMT_TIME_HMS
                                     )
from trade_monitor.utilities import CANDLE_COL_NAME_LIST
from trade_monitor.utilities import (COL_NAME_TIME,
                                     COL_NAME_ASK_OP,
                                     COL_NAME_ASK_HI,
                                     COL_NAME_ASK_LO,
                                     COL_NAME_ASK_CL,
                                     COL_NAME_MID_OP,
                                     COL_NAME_MID_HI,
                                     COL_NAME_MID_LO,
                                     COL_NAME_MID_CL,
                                     COL_NAME_BID_OP,
                                     COL_NAME_BID_HI,
                                     COL_NAME_BID_LO,
                                     COL_NAME_BID_CL
                                     )

from trade_monitor.gapfill.gapfill_heatmap import GapFillHeatMap

from trade_monitor.gapfill.heatmap_manager import (COL_NAME_DATE,
                                                   COL_NAME_GPA_DIR,
                                                   COL_NAME_GPA_CLOSE_PRICE,
                                                   COL_NAME_GPA_OPEN_PRICE,
                                                   COL_NAME_GPA_PRICE_MID,
                                                   COL_NAME_GPA_PRICE_REAL,
                                                   COL_NAME_VALID_FLAG,
                                                   COL_NAME_SUCCESS_FLAG,
                                                   COL_NAME_GAP_FILLED_TIME,
                                                   COL_NAME_MAX_OPEN_RANGE,
                                                   COL_NAME_END_CLOSE_PRICE)

from trade_monitor.gapfill.candlestick_chart import CandlestickChartGapFillPrev
from trade_monitor.gapfill.candlestick_chart import CandlestickChartGapFillCurr


class GapFillUi():

    GAP_DIR_DICT = {
        GapFillMsg.GAP_DIR_UP: "Up",
        GapFillMsg.GAP_DIR_DOWN: "Down"
    }

    GAP_FILL_VALID_DICT = {
        True: "Valid",
        False: "Invalid"
    }

    GAP_FILL_SUCC_DICT = {
        True: "Success",
        False: "Failure"
    }

    TREEVIEW_HEADERS = [
        "Date",
        "Gap dir",
        "Previous close price",
        "Current open price",
        "Gap price(mid)",
        "Gap price(real)",
        "Valid",
        "Result",
        "Gap filled time",
        "Max open range",
        "End close price"
    ]

    ALL_COLUMNS = [COL_NAME_ASK_OP,
                   COL_NAME_ASK_HI,
                   COL_NAME_ASK_LO,
                   COL_NAME_ASK_CL,
                   COL_NAME_MID_OP,
                   COL_NAME_MID_HI,
                   COL_NAME_MID_LO,
                   COL_NAME_MID_CL,
                   COL_NAME_BID_OP,
                   COL_NAME_BID_HI,
                   COL_NAME_BID_LO,
                   COL_NAME_BID_CL
                   ]

    ASK_COLUMNS = [COL_NAME_ASK_OP,
                   COL_NAME_ASK_HI,
                   COL_NAME_ASK_LO,
                   COL_NAME_ASK_CL
                   ]

    MID_COLUMNS = [COL_NAME_MID_OP,
                   COL_NAME_MID_HI,
                   COL_NAME_MID_LO,
                   COL_NAME_MID_CL
                   ]

    BID_COLUMNS = [COL_NAME_BID_OP,
                   COL_NAME_BID_HI,
                   COL_NAME_BID_LO,
                   COL_NAME_BID_CL
                   ]

    CDL_COLUMNS = [CandlestickChartGapFillPrev.COL_NAME_OP,
                   CandlestickChartGapFillPrev.COL_NAME_HI,
                   CandlestickChartGapFillPrev.COL_NAME_LO,
                   CandlestickChartGapFillPrev.COL_NAME_CL
                   ]

    SPREAD_COLUMNS_LIST = [MID_COLUMNS,
                           ASK_COLUMNS,
                           BID_COLUMNS]

    DRAW_CANDLE_COUNT = 50

    def __init__(self, ui) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_gapfill_inst)
        for obj in INST_MSG_LIST:
            ui.comboBox_gapfill_inst.addItem(obj.text)

        utl.remove_all_items_of_comboBox(ui.comboBox_gapfill_spread_prev)
        utl.remove_all_items_of_comboBox(ui.comboBox_gapfill_spread_curr)
        for text in SPREAD_MSG_LIST:
            ui.comboBox_gapfill_spread_prev.addItem(text)
            ui.comboBox_gapfill_spread_curr.addItem(text)

        ui.comboBox_gapfill_spread_prev.currentIndexChanged.connect(
            self._comboBox_gapfill_spread_prev_changed)
        ui.comboBox_gapfill_spread_curr.currentIndexChanged.connect(
            self._comboBox_gapfill_spread_curr_changed)

        callback = self._on_gapfill_heatmap_clicked
        ui.pushButton_gapfill_heatmap.clicked.connect(callback)

        callback = self._on_fetch_gapfill_clicked
        ui.pushButton_gapfill_fetch.clicked.connect(callback)

        qstd_itm_mdl = QStandardItemModel()
        sel_mdl = QItemSelectionModel(qstd_itm_mdl)

        callback = self._on_selection_gapfill_changed
        sel_mdl.selectionChanged.connect(callback)

        # set header
        qstd_itm_mdl.setHorizontalHeaderLabels(self.TREEVIEW_HEADERS)
        ui.treeView_gapfill.setModel(qstd_itm_mdl)
        ui.treeView_gapfill.setSelectionModel(sel_mdl)
        header = ui.treeView_gapfill.header()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        chart_prev = CandlestickChartGapFillPrev(ui.widget_gapfill_chart_prev)
        chart_curr = CandlestickChartGapFillCurr(ui.widget_gapfill_chart_curr)

        # Create service client "gapfill_monitor"
        srv_type = GapFillMntSrv
        srv_name = "gapfill_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = utl.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._widget_htmap = GapFillHeatMap()

        self._chart_prev = chart_prev
        self._chart_curr = chart_curr
        self._qstd_itm_mdl = qstd_itm_mdl

        self._ui = ui
        self._gran_id = 0
        self._srv_cli_list = srv_cli_list
        self._end_time = dt.time(10, 0, 0)
        self._is_update = False
        self._df_param = pd.DataFrame()
        self._sr_gf = pd.Series()
        self._df_prev = pd.DataFrame()
        self._df_curr = pd.DataFrame()

    def _on_fetch_gapfill_clicked(self):

        self._qstd_itm_mdl.clear()
        self._qstd_itm_mdl.setHorizontalHeaderLabels(self.TREEVIEW_HEADERS)
        inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
        inst_msg = INST_MSG_LIST[inst_idx]

        decimal_digit = inst_msg.decimal_digit
        fmt = "{:." + str(decimal_digit) + "f}"

        # fetch Gap-fill data
        req = GapFillMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            utl.logger().error("service server [{}] not to become ready"
                               .format(inst_msg.text))
        else:

            rsp = utl.call_servive_sync(srv_cli, req, timeout_sec=10.0)
            self._gran_id = rsp.gran_id
            """
            future = srv_cli.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

            flg = future.done() and future.result() is not None
            assert flg, "fetch [Gap-Fill] failed!"

            rsp = future.result()
            """
            # Parameter data
            data = []
            for gapfillmsg in rsp.gapfillmsg_list:
                items = [
                    QStandardItem(gapfillmsg.date),
                    QStandardItem(self.GAP_DIR_DICT[gapfillmsg.gap_dir]),
                    QStandardItem(fmt.format(gapfillmsg.gap_close_price)),
                    QStandardItem(fmt.format(gapfillmsg.gap_open_price)),
                    QStandardItem(fmt.format(gapfillmsg.gap_price_mid)),
                    QStandardItem(fmt.format(gapfillmsg.gap_price_real)),
                    QStandardItem(self.GAP_FILL_VALID_DICT[
                        gapfillmsg.is_valid]),
                    QStandardItem(self.GAP_FILL_SUCC_DICT[
                        gapfillmsg.is_gapfill_success]),
                    QStandardItem(gapfillmsg.gap_filled_time),
                    QStandardItem(fmt.format(gapfillmsg.max_open_range)),
                    QStandardItem(fmt.format(gapfillmsg.end_close_price))
                ]
                self._qstd_itm_mdl.appendRow(items)
                data.append([gapfillmsg.date,
                             gapfillmsg.gap_dir,
                             gapfillmsg.gap_close_price,
                             gapfillmsg.gap_open_price,
                             gapfillmsg.gap_price_mid,
                             gapfillmsg.gap_price_real,
                             gapfillmsg.is_valid,
                             gapfillmsg.is_gapfill_success,
                             gapfillmsg.gap_filled_time,
                             gapfillmsg.max_open_range,
                             gapfillmsg.end_close_price
                             ])

            columns = [COL_NAME_DATE,
                       COL_NAME_GPA_DIR,
                       COL_NAME_GPA_CLOSE_PRICE,
                       COL_NAME_GPA_OPEN_PRICE,
                       COL_NAME_GPA_PRICE_MID,
                       COL_NAME_GPA_PRICE_REAL,
                       COL_NAME_VALID_FLAG,
                       COL_NAME_SUCCESS_FLAG,
                       COL_NAME_GAP_FILLED_TIME,
                       COL_NAME_MAX_OPEN_RANGE,
                       COL_NAME_END_CLOSE_PRICE,
                       ]

            df_param = pd.DataFrame(data, columns=columns)
            self._df_param = df_param.set_index(COL_NAME_DATE)

            """
            # Heat map data
            self._hmap_range_start = rsp.heatmap_range_start
            self._hmap_range_end = rsp.heatmap_range_end
            self._hmap_range_step = rsp.heatmap_range_step

            hm_idx = [COL_NAME_DATE]
            hm_x_range = list(np.arange(self._hmap_range_start,
                                        self._hmap_range_end,
                                        self._hmap_range_step))
            columns = hm_idx + hm_x_range
            data = []
            for heatmapmsg in rsp.heatmapmsg_list:
                idx = [heatmapmsg.date]
                data.append(idx + heatmapmsg.data_list.tolist())
            df_hmap = pd.DataFrame(data, columns=columns)
            self._df_hmap = df_hmap.set_index(hm_idx)
            """

            header = self._ui.treeView_gapfill.header()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

            self._end_time = dt.datetime.strptime(rsp.end_time,
                                                  FMT_TIME_HMS).time()
            self._is_update = True

    def _on_gapfill_heatmap_clicked(self):

        inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
        self._widget_htmap.set_param(inst_idx, self._df_param)
        self._widget_htmap.show()
        self._widget_htmap.init_resize()

    def _on_selection_gapfill_changed(self, selected, _):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            trg_date_str = self._qstd_itm_mdl.item(model_index.row()).text()
            trg_date = dt.datetime.strptime(trg_date_str, FMT_DATE_YMD)

            dt_from = trg_date - dt.timedelta(days=2)
            dt_to = trg_date + dt.timedelta(hours=12)

            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            inst_msg = INST_MSG_LIST[inst_idx]

            req = CandlesMntSrv.Request()
            req.gran_msg.gran_id = self._gran_id
            req.inst_msg.inst_id = inst_msg.msg_id
            req.dt_from = dt_from.strftime(FMT_DTTM_API)
            req.dt_to = dt_to.strftime(FMT_DTTM_API)

            utl.logger().debug("dt_from: " + req.dt_from)
            utl.logger().debug("dt_to: " + req.dt_to)

            rsp = utl.call_servive_sync_candle(req, timeout_sec=10.0)
            data = []
            for cndl_msg in rsp.cndl_msg_list:
                dt_ = dt.datetime.strptime(cndl_msg.time, FMT_DTTM_API)
                data.append([dt_,
                             cndl_msg.ask_o,
                             cndl_msg.ask_h,
                             cndl_msg.ask_l,
                             cndl_msg.ask_c,
                             cndl_msg.mid_o,
                             cndl_msg.mid_h,
                             cndl_msg.mid_l,
                             cndl_msg.mid_c,
                             cndl_msg.bid_o,
                             cndl_msg.bid_h,
                             cndl_msg.bid_l,
                             cndl_msg.bid_c,
                             cndl_msg.is_complete
                             ])

            df = pd.DataFrame(data)
            df.columns = CANDLE_COL_NAME_LIST
            df = df.set_index(COL_NAME_TIME)

            sr_gf = self._df_param.loc[trg_date_str]

            df_prev = df.loc[:, GapFillUi.ALL_COLUMNS]
            df_curr = df.loc[:, GapFillUi.ALL_COLUMNS]

            th = df_prev.index[-1] - dt.timedelta(days=1)
            df_prev = df_prev[df_prev.index < th].tail(self.DRAW_CANDLE_COUNT)

            th = df_curr.index[-1] - dt.timedelta(days=1)
            df_curr = df_curr[th < df_curr.index].head(self.DRAW_CANDLE_COUNT)

            max_prev = df_prev[COL_NAME_ASK_HI].max()
            min_prev = df_prev[COL_NAME_BID_LO].min()
            max_curr = df_curr[COL_NAME_ASK_HI].max()
            min_curr = df_curr[COL_NAME_BID_LO].min()

            max_y = max(max_prev, max_curr)
            min_y = min(min_prev, min_curr)

            self._chart_prev.set_max_y(max_y)
            self._chart_prev.set_min_y(min_y)
            self._chart_curr.set_max_y(max_y)
            self._chart_curr.set_min_y(min_y)

            decimal_digit = inst_msg.decimal_digit

            idx = self._ui.comboBox_gapfill_spread_prev.currentIndex()
            self._update_prev_chart(idx, df_prev, sr_gf, decimal_digit)

            idx = self._ui.comboBox_gapfill_spread_curr.currentIndex()
            self._update_curr_chart(idx, df_curr, sr_gf, decimal_digit)

            self._sr_gf = sr_gf
            self._df_prev = df_prev
            self._df_curr = df_curr

    def resize_chart_widget(self):
        fs = self._ui.widget_gapfill_chart_prev.frameSize()
        self._chart_prev.resize(fs)
        fs = self._ui.widget_gapfill_chart_curr.frameSize()
        self._chart_curr.resize(fs)

    def _comboBox_gapfill_spread_prev_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
            self._update_prev_chart(idx, self._df_prev, self._sr_gf,
                                    decimal_digit)

    def _comboBox_gapfill_spread_curr_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
            self._update_curr_chart(idx, self._df_curr, self._sr_gf,
                                    decimal_digit)

    def _update_prev_chart(self, idx, df, sr_gf, decimal_digit):

        df_prev = df.loc[:, GapFillUi.SPREAD_COLUMNS_LIST[idx]]
        df_prev.columns = GapFillUi.CDL_COLUMNS

        self._chart_prev.update(df_prev,
                                self._gran_id,
                                sr_gf[COL_NAME_GPA_CLOSE_PRICE],
                                sr_gf[COL_NAME_GPA_OPEN_PRICE],
                                decimal_digit)

    def _update_curr_chart(self, idx, df, sr_gf, decimal_digit):

        df_curr = df.loc[:, GapFillUi.SPREAD_COLUMNS_LIST[idx]]
        df_curr.columns = GapFillUi.CDL_COLUMNS

        self._chart_curr.update(df_curr,
                                self._gran_id,
                                sr_gf[COL_NAME_GPA_CLOSE_PRICE],
                                sr_gf[COL_NAME_GPA_OPEN_PRICE],
                                decimal_digit, self._end_time)
