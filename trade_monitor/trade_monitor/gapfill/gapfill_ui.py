import pandas as pd
import datetime as dt
from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel
from trade_apl_msgs.srv import GapFillMntSrv
from trade_apl_msgs.msg import GapFillTblGapRecMsg as GapMsg
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.constant import InstParam, GranParam
from trade_monitor.constant import SPREAD_MSG_LIST
from trade_monitor.gapfill.constant import VALID_INST_LIST
from trade_monitor.constant import (FMT_YMDHMS,
                                    FMT_DATE_YMD,
                                    FMT_TIME_HMS
                                    )
from trade_monitor.gapfill.heatmap_ui import HeatMapUi
from trade_monitor.gapfill.constant import ColNameGap as ColNmGap
from trade_monitor.gapfill.constant import ColNameOhlc as ColNmOhlc
from trade_monitor.gapfill.widget import CandlestickChartViewPrev as ChartViewPrev
from trade_monitor.gapfill.widget import CandlestickChartViewCurr as ChartViewCurr


_GAP_DIR_DICT = {
    GapMsg.GAP_DIR_UP: "Up",
    GapMsg.GAP_DIR_DOWN: "Down"
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


_ASK_COLUMNS = [ColNmOhlc.ASK_O.value,
                ColNmOhlc.ASK_H.value,
                ColNmOhlc.ASK_L.value,
                ColNmOhlc.ASK_C.value
                ]

_MID_COLUMNS = [ColNmOhlc.MID_O.value,
                ColNmOhlc.MID_H.value,
                ColNmOhlc.MID_L.value,
                ColNmOhlc.MID_C.value
                ]

_BID_COLUMNS = [ColNmOhlc.BID_O.value,
                ColNmOhlc.BID_H.value,
                ColNmOhlc.BID_L.value,
                ColNmOhlc.BID_C.value
                ]

_SPREAD_COLUMNS_LIST = [_MID_COLUMNS,
                        _ASK_COLUMNS,
                        _BID_COLUMNS]


class GapFillUi():

    def __init__(self, ui) -> None:

        self._DRAW_CANDLE_COUNT = 50

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
        qstd_itm_mdl.setHorizontalHeaderLabels(_TREEVIEW_HEADERS)
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
        self._exit_time = dt.time(10, 0, 0)
        self._is_update = False
        self._df_ohlc = pd.DataFrame()
        self._df_gap = pd.DataFrame()
        self._sr_gf = pd.Series()
        self._df_prev = pd.DataFrame()
        self._df_curr = pd.DataFrame()
        self._logger = ros_com.get_logger()

    def _on_fetch_gapfill_clicked(self):

        self._qstd_itm_mdl.clear()
        self._qstd_itm_mdl.setHorizontalHeaderLabels(_TREEVIEW_HEADERS)
        inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
        inst_param = VALID_INST_LIST[inst_idx]
        fmt = "{:." + str(inst_param.digit) + "f}"

        # fetch Gap-fill data
        req = GapFillMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            self._logger.error("Service server [{}] not ready"
                               .format(inst_param.text))
        else:

            rsp = ros_com.call_servive_sync(srv_cli, req, timeout_sec=10.0)
            self._gran_id = rsp.gran_id

            # OHLC table data
            tbl = []
            for rec in rsp.tbl_ohlc:
                tbl.append([dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                            rec.data_id,
                            rec.ask_o,
                            rec.ask_h,
                            rec.ask_l,
                            rec.ask_c,
                            rec.mid_o,
                            rec.mid_h,
                            rec.mid_l,
                            rec.mid_c,
                            rec.bid_o,
                            rec.bid_h,
                            rec.bid_l,
                            rec.bid_c,
                            ])
            df_ohlc = pd.DataFrame(tbl, columns=ColNmOhlc.to_list())
            self._df_ohlc = df_ohlc.set_index(ColNmOhlc.DATETIME.value)

            # Gap table data
            tbl = []
            for rec in rsp.tbl_gap:
                items = [
                    QStandardItem(rec.date),
                    QStandardItem(_GAP_DIR_DICT[rec.gap_dir]),
                    QStandardItem(fmt.format(rec.close_price_mid)),
                    QStandardItem(fmt.format(rec.open_price_mid)),
                    QStandardItem(fmt.format(rec.gap_price_mid)),
                    QStandardItem(fmt.format(rec.gap_price_real)),
                    QStandardItem(_GAP_FILL_VALID_DICT[rec.is_valid]),
                    QStandardItem(_GAP_FILL_SUCC_DICT[rec.is_gapfill_success]),
                    QStandardItem(rec.gap_filled_time),
                    QStandardItem(fmt.format(rec.max_open_range)),
                    QStandardItem(fmt.format(rec.end_close_price))
                ]
                self._qstd_itm_mdl.appendRow(items)
                tbl.append([rec.date,
                            rec.data_id,
                            rec.gap_dir,
                            rec.close_price_mid,
                            rec.open_price_mid,
                            rec.gap_price_mid,
                            rec.gap_price_real,
                            rec.is_valid,
                            rec.is_gapfill_success,
                            rec.gap_filled_time,
                            rec.max_open_range,
                            rec.end_close_price,
                            rec.end_diff_price
                            ])

            df_gap = pd.DataFrame(tbl, columns=ColNmGap.to_list())
            self._df_gap = df_gap.set_index(ColNmGap.DATE.value)

            header = self._ui.treeView_gapfill.header()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

            self._exit_time = dt.datetime.strptime(rsp.exit_time,
                                                   FMT_TIME_HMS).time()
            self._is_update = True

    def _on_gapfill_heatmap_clicked(self):

        inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
        self._htmap_ui.set_param(inst_idx, self._df_gap)
        self._htmap_ui.show()
        self._htmap_ui.init_resize()

    def _on_selection_gapfill_changed(self, selected, _):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            trg_date_str = self._qstd_itm_mdl.item(model_index.row()).text()
            trg_date = dt.datetime.strptime(trg_date_str, FMT_DATE_YMD)

            sr_gf = self._df_gap.loc[trg_date_str]
            data_id = sr_gf[ColNmGap.DATA_ID.value]
            df_ohlc = self._df_ohlc[self._df_ohlc[ColNmOhlc.DATA_ID.value] == data_id]

            date_th = df_ohlc.index[0] + dt.timedelta(days=1)
            date_th = date_th.replace(hour=0, minute=0, second=0)

            df_prev = df_ohlc[df_ohlc.index < date_th]
            df_curr = df_ohlc[df_ohlc.index > date_th]

            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            inst_param = VALID_INST_LIST[inst_idx]

            df_prev = df_prev.tail(self._DRAW_CANDLE_COUNT)
            df_curr = df_curr.head(self._DRAW_CANDLE_COUNT)

            max_prev = df_prev[ColNmOhlc.ASK_H.value].max()
            min_prev = df_prev[ColNmOhlc.BID_L.value].min()
            max_curr = df_curr[ColNmOhlc.ASK_H.value].max()
            min_curr = df_curr[ColNmOhlc.BID_L.value].min()

            max_y = max(max_prev, max_curr)
            min_y = min(min_prev, min_curr)

            self._chartview_prev.set_max_y(max_y)
            self._chartview_prev.set_min_y(min_y)
            self._chartview_curr.set_max_y(max_y)
            self._chartview_curr.set_min_y(min_y)

            idx = self._ui.comboBox_gapfill_spread_prev.currentIndex()
            self._update_prev_chart(idx, df_prev, sr_gf, inst_param)

            idx = self._ui.comboBox_gapfill_spread_curr.currentIndex()
            self._update_curr_chart(idx, df_curr, sr_gf, inst_param)

            self._sr_gf = sr_gf
            self._df_prev = df_prev
            self._df_curr = df_curr

    def _comboBox_gapfill_spread_prev_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            inst_param = VALID_INST_LIST[inst_idx]
            self._update_prev_chart(idx, self._df_prev, self._sr_gf, inst_param)

    def _comboBox_gapfill_spread_curr_changed(self, idx):

        if self._is_update:
            inst_idx = self._ui.comboBox_gapfill_inst.currentIndex()
            inst_param = VALID_INST_LIST[inst_idx]
            self._update_curr_chart(idx, self._df_curr, self._sr_gf, inst_param)

    def _update_prev_chart(self, idx, df, sr_gf, inst_param: InstParam):

        df_prev = df.loc[:, _SPREAD_COLUMNS_LIST[idx]]
        df_prev.columns = self._chartview_prev.CandleLabel.to_list()
        gran_param = GranParam.get_member_by_msgid(self._gran_id)

        self._chartview_prev.update(df_prev,
                                    sr_gf[ColNmGap.CLOSE_PRICE_MID.value],
                                    sr_gf[ColNmGap.OPEN_PRICE_MID.value],
                                    gran_param,
                                    inst_param)

    def _update_curr_chart(self, idx, df, sr_gf, inst_param: InstParam):

        df_curr = df.loc[:, _SPREAD_COLUMNS_LIST[idx]]
        df_curr.columns = self._chartview_curr.CandleLabel.to_list()
        gran_param = GranParam.get_member_by_msgid(self._gran_id)

        self._chartview_curr.update(df_curr,
                                    sr_gf[ColNmGap.CLOSE_PRICE_MID.value],
                                    sr_gf[ColNmGap.OPEN_PRICE_MID.value],
                                    gran_param,
                                    inst_param,
                                    self._exit_time)
