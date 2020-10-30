import datetime as dt
import pandas as pd

import rclpy

from trade_monitor.candlestick_chart import CandlestickChart
from trade_manager_msgs.srv import CandlesMntSrv
from trade_monitor import utilities as utl
from trade_monitor.utilities import INST_MSG_LIST, GRAN_MSG_LIST
from trade_monitor.utilities import FMT_DTTM_API
from trade_monitor.utilities import CANDLE_COL_NAME_LIST
from trade_monitor.utilities import (COL_NAME_TIME,
                                     COL_NAME_ASK_OP,
                                     COL_NAME_ASK_HI,
                                     COL_NAME_ASK_LO,
                                     COL_NAME_ASK_CL,
                                     COL_NAME_BID_OP,
                                     COL_NAME_BID_HI,
                                     COL_NAME_BID_LO,
                                     COL_NAME_BID_CL,
                                     COL_NAME_MID_OP,
                                     COL_NAME_MID_HI,
                                     COL_NAME_MID_LO,
                                     COL_NAME_MID_CL
                                     )


class MainUi():

    def __init__(self, ui):

        callback = self._on_cb_inst_main_changed
        ui.comboBox_inst_main.currentIndexChanged.connect(callback)
        callback = self._on_cb_gran_main_changed
        ui.comboBox_gran_main.currentIndexChanged.connect(callback)

        cs_chart = CandlestickChart(ui.widget_chart_main)

        self._ui = ui
        self._cs_chart = cs_chart

    def draw_chart(self, inst_idx, gran_idx):
        self._draw_chart(inst_idx, gran_idx)

    def _on_cb_inst_main_changed(self, inst_idx):
        gran_idx = self._ui.comboBox_gran_main.currentIndex()
        self._draw_chart(inst_idx, gran_idx)

    def _on_cb_gran_main_changed(self, gran_idx):
        inst_idx = self._ui.comboBox_inst_main.currentIndex()
        self._draw_chart(inst_idx, gran_idx)

    def _draw_chart(self, inst_idx, gran_idx):

        dt_now = dt.datetime.now()
        dt_from = dt_now - dt.timedelta(days=20)
        dt_to = dt_now

        inst_id = INST_MSG_LIST[inst_idx].msg_id
        gran_id = GRAN_MSG_LIST[gran_idx].msg_id

        req = CandlesMntSrv.Request()
        req.gran_msg.gran_id = gran_id
        req.inst_msg.inst_id = inst_id
        req.dt_from = dt_from.strftime(FMT_DTTM_API)
        req.dt_to = dt_to.strftime(FMT_DTTM_API)

        rsp = utl.call_servive_sync_candle(req, timeout_sec=10.0)
        data = []
        for cndl_msg in rsp.cndl_msg_list:
            dt_ = dt.datetime.strptime(cndl_msg.time, FMT_DTTM_API)
            data.append([dt_,
                         cndl_msg.ask_o,
                         cndl_msg.ask_h,
                         cndl_msg.ask_l,
                         cndl_msg.ask_c,
                         cndl_msg.bid_o,
                         cndl_msg.bid_h,
                         cndl_msg.bid_l,
                         cndl_msg.bid_c,
                         cndl_msg.is_complete
                         ])

        df = pd.DataFrame(data)
        df.columns = CANDLE_COL_NAME_LIST
        df = df.set_index(COL_NAME_TIME)

        df[COL_NAME_MID_OP] = (df[COL_NAME_ASK_OP] + df[COL_NAME_BID_OP]) / 2
        df[COL_NAME_MID_HI] = (df[COL_NAME_ASK_HI] + df[COL_NAME_BID_HI]) / 2
        df[COL_NAME_MID_LO] = (df[COL_NAME_ASK_LO] + df[COL_NAME_BID_LO]) / 2
        df[COL_NAME_MID_CL] = (df[COL_NAME_ASK_CL] + df[COL_NAME_BID_CL]) / 2

        dftmp = df.loc[:, [COL_NAME_MID_OP,
                           COL_NAME_MID_HI,
                           COL_NAME_MID_LO,
                           COL_NAME_MID_CL
                           ]]
        dftmp.columns = [CandlestickChart.COL_NAME_OP,
                         CandlestickChart.COL_NAME_HI,
                         CandlestickChart.COL_NAME_LO,
                         CandlestickChart.COL_NAME_CL
                         ]

        self._cs_chart.update(dftmp, gran_id)

    def resize_chart_widget(self):
        fs = self._ui.widget_chart_main.frameSize()
        self._cs_chart.resize(fs)
