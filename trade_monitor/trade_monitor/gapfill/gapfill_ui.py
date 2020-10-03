import pandas as pd
import datetime as dt
import numpy as np

from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

import rclpy

from trade_monitor.candlestick_chart import CandlestickChartGapFillPrev
from trade_monitor.candlestick_chart import CandlestickChartGapFillCurr

from trade_apl_msgs.srv import GapFillMntSrv
from trade_apl_msgs.msg import GapFillMsg
from trade_manager_msgs.msg import Granularity as Gran
from trade_manager_msgs.srv import CandlesMntSrv

from trade_monitor import util as utl
from trade_monitor.util import SPREAD_MSG_LIST
from trade_monitor.util import INST_MSG_LIST
from trade_monitor.util import DT_FMT
from trade_monitor.util import CANDLE_COL_NAME_LIST
from trade_monitor.util import (COL_NAME_TIME,
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

    GAP_FILL_HEADERS = [
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

    def __init__(self, ui, node, cli_cdl) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_inst_gapfill)
        for obj in INST_MSG_LIST:
            ui.comboBox_inst_gapfill.addItem(obj.text)

        utl.remove_all_items_of_comboBox(ui.comboBox_spread_prev)
        utl.remove_all_items_of_comboBox(ui.comboBox_spread_curr)
        for text in SPREAD_MSG_LIST:
            ui.comboBox_spread_prev.addItem(text)
            ui.comboBox_spread_curr.addItem(text)

        ui.comboBox_spread_prev.currentIndexChanged.connect(
            self.__combobox_spread_prev_changed)
        ui.comboBox_spread_curr.currentIndexChanged.connect(
            self.__combobox_spread_curr_changed)

        callback = self.__on_gapfill_heatmap_clicked
        ui.pushButton_gapfill_heatmap.clicked.connect(callback)

        logger = node.get_logger()

        callback = self.__on_fetch_gapfill_clicked
        ui.pushButton_fetch_gapfill.clicked.connect(callback)

        qstd_itm_mdl = QStandardItemModel()
        sel_mdl = QItemSelectionModel(qstd_itm_mdl)

        callback = self.__on_selection_gapfill_changed
        sel_mdl.selectionChanged.connect(callback)

        # set header
        qstd_itm_mdl.setHorizontalHeaderLabels(self.GAP_FILL_HEADERS)
        ui.treeView_gapfill.setModel(qstd_itm_mdl)
        ui.treeView_gapfill.setSelectionModel(sel_mdl)
        header = ui.treeView_gapfill.header()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        chart_prev = CandlestickChartGapFillPrev(ui.widget_chart_gapfill_prev)
        chart_curr = CandlestickChartGapFillCurr(ui.widget_chart_gapfill_curr)

        # Create service client "CandlesMonitor"
        srv_type = GapFillMntSrv
        srv_name = "gapfill_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = node.create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)
            #srv_cli.service_is_ready()

        # Wait for a service server
        """
        while not srv_cli.wait_for_service(timeout_sec=1.0):
            logger.info("Waiting for \"" + srv_name + "\" service...")
        """

        self.__widget_htmap = GapFillHeatMap()

        self.__chart_prev = chart_prev
        self.__chart_curr = chart_curr
        self.__qstd_itm_mdl = qstd_itm_mdl

        self.__ui = ui
        self.__node = node
        #self.__srv_cli = srv_cli
        self.__srv_cli_list = srv_cli_list
        self.__cli_cdl = cli_cdl
        self.__end_time = dt.time(10, 0, 0)
        self.__is_update = False
        self.__df_param = pd.DataFrame()
        self.__sr_gf = pd.Series()
        self.__df_prev = pd.DataFrame()
        self.__df_curr = pd.DataFrame()

        self.__logger = logger

    def __on_fetch_gapfill_clicked(self):

        self.__qstd_itm_mdl.clear()
        self.__qstd_itm_mdl.setHorizontalHeaderLabels(self.GAP_FILL_HEADERS)
        inst_idx = self.__ui.comboBox_inst_gapfill.currentIndex()
        inst_msg = INST_MSG_LIST[inst_idx]

        decimal_digit = inst_msg.decimal_digit
        fmt = "{:." + str(decimal_digit) + "f}"

        # fetch Gap-fill data
        req = GapFillMntSrv.Request()

        srv_cli = self.__srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            self.__logger.error("service server [{}] not to become ready"
                                .format(inst_msg.text))
        else:
            future = srv_cli.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)

            flg = future.done() and future.result() is not None
            assert flg, "fetch [Gap-Fill] failed!"

            rsp = future.result()
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
                self.__qstd_itm_mdl.appendRow(items)
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
            self.__df_param = df_param.set_index(COL_NAME_DATE)

            """
            # Heat map data
            self.__hmap_range_start = rsp.heatmap_range_start
            self.__hmap_range_end = rsp.heatmap_range_end
            self.__hmap_range_step = rsp.heatmap_range_step

            hm_idx = [COL_NAME_DATE]
            hm_x_range = list(np.arange(self.__hmap_range_start,
                                        self.__hmap_range_end,
                                        self.__hmap_range_step))
            columns = hm_idx + hm_x_range
            data = []
            for heatmapmsg in rsp.heatmapmsg_list:
                idx = [heatmapmsg.date]
                data.append(idx + heatmapmsg.data_list.tolist())
            df_hmap = pd.DataFrame(data, columns=columns)
            self.__df_hmap = df_hmap.set_index(hm_idx)
            """

            header = self.__ui.treeView_gapfill.header()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

            self.__end_time = dt.datetime.strptime(rsp.end_time, "%H:%M:%S").time()
            #self.__decimal_digit = decimal_digit
            self.__is_update = True

    def __on_gapfill_heatmap_clicked(self):
        self.__logger.debug("gapfill_heatmap_clicked")

        inst_idx = self.__ui.comboBox_inst_gapfill.currentIndex()
        self.__widget_htmap.set_param(inst_idx, self.__df_param)
        self.__widget_htmap.show()
        self.__widget_htmap.init_resize()

    def __on_selection_gapfill_changed(self, selected, deselected):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            trg_date_str = self.__qstd_itm_mdl.item(model_index.row()).text()
            self.__logger.debug("target_date: " + trg_date_str)
            trg_date = dt.datetime.strptime(trg_date_str, "%Y-%m-%d")

            dt_from = trg_date - dt.timedelta(days=2)
            dt_to = trg_date + dt.timedelta(hours=12)

            inst_idx = self.__ui.comboBox_inst_gapfill.currentIndex()
            inst_msg = INST_MSG_LIST[inst_idx]

            req = CandlesMntSrv.Request()
            req.gran_msg.gran_id = Gran.GRAN_M10
            req.inst_msg.inst_id = inst_msg.msg_id
            req.dt_from = dt_from.strftime(DT_FMT)
            req.dt_to = dt_to.strftime(DT_FMT)

            self.__logger.debug("dt_from: " + req.dt_from)
            self.__logger.debug("dt_to: " + req.dt_to)

            future = self.__cli_cdl.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)

            flg = future.done() and future.result() is not None
            assert flg, "initial fetch [Day Candle] failed!"

            data = []
            rsp = future.result()
            for cndl_msg in rsp.cndl_msg_list:
                dt_ = dt.datetime.strptime(cndl_msg.time, DT_FMT)
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

            sr_gf = self.__df_param.loc[trg_date_str]

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

            self.__chart_prev.set_max_y(max_y)
            self.__chart_prev.set_min_y(min_y)
            self.__chart_curr.set_max_y(max_y)
            self.__chart_curr.set_min_y(min_y)

            decimal_digit = inst_msg.decimal_digit

            idx = self.__ui.comboBox_spread_prev.currentIndex()
            self.__update_prev_chart(idx, df_prev, sr_gf, decimal_digit)

            idx = self.__ui.comboBox_spread_curr.currentIndex()
            self.__update_curr_chart(idx, df_curr, sr_gf, decimal_digit)

            self.__sr_gf = sr_gf
            self.__df_prev = df_prev
            self.__df_curr = df_curr

    def resize_chart_widget(self):
        fs = self.__ui.widget_chart_gapfill_prev.frameSize()
        self.__chart_prev.resize(fs)
        fs = self.__ui.widget_chart_gapfill_curr.frameSize()
        self.__chart_curr.resize(fs)

    def __combobox_spread_prev_changed(self, idx):

        if self.__is_update:
            inst_idx = self.__ui.comboBox_inst_gapfill.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
            self.__update_prev_chart(idx, self.__df_prev, self.__sr_gf,
                                     decimal_digit)

    def __combobox_spread_curr_changed(self, idx):

        if self.__is_update:
            inst_idx = self.__ui.comboBox_inst_gapfill.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
            self.__update_curr_chart(idx, self.__df_curr, self.__sr_gf,
                                     decimal_digit)

    def __update_prev_chart(self, idx, df, sr_gf, decimal_digit):

        df_prev = df.loc[:, GapFillUi.SPREAD_COLUMNS_LIST[idx]]
        df_prev.columns = GapFillUi.CDL_COLUMNS

        self.__chart_prev.update(df_prev,
                                 sr_gf[COL_NAME_GPA_CLOSE_PRICE],
                                 sr_gf[COL_NAME_GPA_OPEN_PRICE],
                                 decimal_digit)

    def __update_curr_chart(self, idx, df, sr_gf, decimal_digit):

        df_curr = df.loc[:, GapFillUi.SPREAD_COLUMNS_LIST[idx]]
        df_curr.columns = GapFillUi.CDL_COLUMNS

        self.__chart_curr.update(df_curr,
                                 sr_gf[COL_NAME_GPA_CLOSE_PRICE],
                                 sr_gf[COL_NAME_GPA_OPEN_PRICE],
                                 decimal_digit, self.__end_time)
