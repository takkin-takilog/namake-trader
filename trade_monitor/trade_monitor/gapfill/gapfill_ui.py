import pandas as pd
import datetime as dt

from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

from trade_apl_msgs.srv import GapFillMntSrv
from trade_apl_msgs.msg import GapFillMsg
from trade_manager_msgs.srv import CandlesMntSrv

from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.constant import SPREAD_MSG_LIST
from trade_monitor.gapfill.constant import VALID_INST_LIST
from trade_monitor.constant import (FMT_DTTM_API,
                                    FMT_DATE_YMD,
                                    FMT_TIME_HMS
                                    )
from trade_monitor.constant import CandleColumnName as CdlColNm
from trade_monitor.gapfill.heatmap_ui import HeatMapUi
from trade_monitor.gapfill.constant import ColumnName as GfColNm
from trade_monitor.gapfill.widget import CandlestickChartViewPrev as ChartViewPrev
from trade_monitor.gapfill.widget import CandlestickChartViewCurr as ChartViewCurr


class GapFillUi():

    _GAP_DIR_DICT = {
        GapFillMsg.GAP_DIR_UP: "Up",
        GapFillMsg.GAP_DIR_DOWN: "Down"
    }

    _GAP_FILL_VALID_DICT = {
        True: "Valid",
        False: "Invalid"
    }

    _GAP_FILL_SUCC_DICT = {
        True: "Success",
        False: "Failure"
    }

    _TREEVIEW_HEADERS = [
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

    _ALL_COLUMNS = [CdlColNm.ASK_OP.value,
                    CdlColNm.ASK_HI.value,
                    CdlColNm.ASK_LO.value,
                    CdlColNm.ASK_CL.value,
                    CdlColNm.MID_OP.value,
                    CdlColNm.MID_HI.value,
                    CdlColNm.MID_LO.value,
                    CdlColNm.MID_CL.value,
                    CdlColNm.BID_OP.value,
                    CdlColNm.BID_HI.value,
                    CdlColNm.BID_LO.value,
                    CdlColNm.BID_CL.value
                    ]

    _ASK_COLUMNS = [CdlColNm.ASK_OP.value,
                    CdlColNm.ASK_HI.value,
                    CdlColNm.ASK_LO.value,
                    CdlColNm.ASK_CL.value
                    ]

    _MID_COLUMNS = [CdlColNm.MID_OP.value,
                    CdlColNm.MID_HI.value,
                    CdlColNm.MID_LO.value,
                    CdlColNm.MID_CL.value
                    ]

    _BID_COLUMNS = [CdlColNm.BID_OP.value,
                    CdlColNm.BID_HI.value,
                    CdlColNm.BID_LO.value,
                    CdlColNm.BID_CL.value
                    ]

    """
    CDL_COLUMNS = [CandlestickChartViewPrev.CdlColNm.OP,
                   CandlestickChartViewPrev.CdlColNm.HI,
                   CandlestickChartViewPrev.CdlColNm.LO,
                   CandlestickChartViewPrev.CdlColNm.CL
                   ]
    """

    _SPREAD_COLUMNS_LIST = [_MID_COLUMNS,
                            _ASK_COLUMNS,
                            _BID_COLUMNS]

    _DRAW_CANDLE_COUNT = 50

    def __init__(self, ui) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_gapfill_inst)
        for obj in VALID_INST_LIST:
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
        qstd_itm_mdl.setHorizontalHeaderLabels(self._TREEVIEW_HEADERS)
        ui.treeView_gapfill.setModel(qstd_itm_mdl)
        ui.treeView_gapfill.setSelectionModel(sel_mdl)
        header = ui.treeView_gapfill.header()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        chartview_prev = ChartViewPrev(ui.widget_ChartView_gapfill_prev)
        chartview_curr = ChartViewCurr(ui.widget_ChartView_gapfill_curr)

        # Create service client "gapfill_monitor"
        srv_type = GapFillMntSrv
        srv_name = "gapfill_monitor"
        srv_cli_list = []
        for obj in VALID_INST_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = ros_com.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._htmap_ui = HeatMapUi()

        self._chartview_prev = chartview_prev
        self._chartview_curr = chartview_curr
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
        self._logger = ros_com.get_logger()

    def _on_fetch_gapfill_clicked(self):

        self._qstd_itm_mdl.clear()
        self._qstd_itm_mdl.setHorizontalHeaderLabels(self._TREEVIEW_HEADERS)
        inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
        inst_info = VALID_INST_LIST[inst_idx]
        fmt = "{:." + str(inst_info.digit) + "f}"

        # fetch Gap-fill data
        req = GapFillMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            self._logger.error("service server [{}] not to become ready"
                               .format(inst_info.text))
        else:

            rsp = ros_com.call_servive_sync(srv_cli, req, timeout_sec=10.0)
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
                    QStandardItem(self._GAP_DIR_DICT[gapfillmsg.gap_dir]),
                    QStandardItem(fmt.format(gapfillmsg.gap_close_price)),
                    QStandardItem(fmt.format(gapfillmsg.gap_open_price)),
                    QStandardItem(fmt.format(gapfillmsg.gap_price_mid)),
                    QStandardItem(fmt.format(gapfillmsg.gap_price_real)),
                    QStandardItem(self._GAP_FILL_VALID_DICT[
                        gapfillmsg.is_valid]),
                    QStandardItem(self._GAP_FILL_SUCC_DICT[
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

            df_param = pd.DataFrame(data, columns=GfColNm.to_list())
            self._df_param = df_param.set_index(GfColNm.DATE.value)

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
        self._htmap_ui.set_param(inst_idx, self._df_param)
        self._htmap_ui.show()
        self._htmap_ui.init_resize()

    def _on_selection_gapfill_changed(self, selected, _):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            trg_date_str = self._qstd_itm_mdl.item(model_index.row()).text()
            trg_date = dt.datetime.strptime(trg_date_str, FMT_DATE_YMD)

            dt_from = trg_date - dt.timedelta(days=2)
            dt_to = trg_date + dt.timedelta(hours=12)

            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            inst_info = VALID_INST_LIST[inst_idx]

            req = CandlesMntSrv.Request()
            req.gran_msg.gran_id = self._gran_id
            req.inst_msg.inst_id = inst_info.msg_id
            req.dt_from = dt_from.strftime(FMT_DTTM_API)
            req.dt_to = dt_to.strftime(FMT_DTTM_API)

            rsp = ros_com.call_servive_sync_candle(req, timeout_sec=10.0)
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
            df.columns = CdlColNm.to_list()
            df = df.set_index(CdlColNm.TIME.value)

            sr_gf = self._df_param.loc[trg_date_str]

            df_prev = df.loc[:, self._ALL_COLUMNS]
            df_curr = df.loc[:, self._ALL_COLUMNS]

            th = df_prev.index[-1] - dt.timedelta(days=1)
            df_prev = df_prev[df_prev.index < th].tail(self._DRAW_CANDLE_COUNT)

            th = df_curr.index[-1] - dt.timedelta(days=1)
            df_curr = df_curr[th < df_curr.index].head(self._DRAW_CANDLE_COUNT)

            max_prev = df_prev[CdlColNm.ASK_HI.value].max()
            min_prev = df_prev[CdlColNm.BID_LO.value].min()
            max_curr = df_curr[CdlColNm.ASK_HI.value].max()
            min_curr = df_curr[CdlColNm.BID_LO.value].min()

            max_y = max(max_prev, max_curr)
            min_y = min(min_prev, min_curr)

            self._chartview_prev.set_max_y(max_y)
            self._chartview_prev.set_min_y(min_y)
            self._chartview_curr.set_max_y(max_y)
            self._chartview_curr.set_min_y(min_y)

            digit = inst_info.digit

            idx = self._ui.comboBox_gapfill_spread_prev.currentIndex()
            self._update_prev_chart(idx, df_prev, sr_gf, digit)

            idx = self._ui.comboBox_gapfill_spread_curr.currentIndex()
            self._update_curr_chart(idx, df_curr, sr_gf, digit)

            self._sr_gf = sr_gf
            self._df_prev = df_prev
            self._df_curr = df_curr

    def _comboBox_gapfill_spread_prev_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            digit = VALID_INST_LIST[inst_idx].digit
            self._update_prev_chart(idx, self._df_prev, self._sr_gf, digit)

    def _comboBox_gapfill_spread_curr_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            digit = VALID_INST_LIST[inst_idx].digit
            self._update_curr_chart(idx, self._df_curr, self._sr_gf, digit)

    def _update_prev_chart(self, idx, df, sr_gf, digit):

        df_prev = df.loc[:, self._SPREAD_COLUMNS_LIST[idx]]
        df_prev.columns = self._chartview_prev.CandleLabel.to_list()

        self._chartview_prev.update(df_prev,
                                    self._gran_id,
                                    sr_gf[GfColNm.GPA_CLOSE_PRICE.value],
                                    sr_gf[GfColNm.GPA_OPEN_PRICE.value],
                                    digit)

    def _update_curr_chart(self, idx, df, sr_gf, digit):

        df_curr = df.loc[:, self._SPREAD_COLUMNS_LIST[idx]]
        df_curr.columns = self._chartview_curr.CandleLabel.to_list()

        self._chartview_curr.update(df_curr,
                                    self._gran_id,
                                    sr_gf[GfColNm.GPA_CLOSE_PRICE.value],
                                    sr_gf[GfColNm.GPA_OPEN_PRICE.value],
                                    digit,
                                    self._end_time)
