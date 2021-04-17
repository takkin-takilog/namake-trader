from enum import Enum
import pandas as pd
import datetime as dt
from PySide2.QtCore import Qt
from PySide2.QtWidgets import QCheckBox, QGroupBox, QVBoxLayout
from PySide2.QtWidgets import QMenu, QWidgetAction
from PySide2.QtWidgets import QToolButton
from trade_apl_msgs.srv import TechMntSrv, TechChartMntSrv
from trade_monitor.widget_base import PandasTreeView
from trade_monitor import utility as utl
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.constant import FMT_YMDHMS, FMT_DATE_YMD
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.constant import ColNameOhlc
from trade_monitor.tech.constant import ColNameTrnd
from trade_monitor.tech.constant import ColNameOsci
from trade_monitor.tech.constant import ColNameSma
from trade_monitor.tech.widget import CandlestickChartView
# from trade_monitor.widget_base import CandlestickChartViewBarCategoryAxis as CandlestickChartView
# from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis as CandlestickChartView
from trade_monitor import ros_common as ros_com
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran
from trade_monitor.widget_base import BaseCandlestickChartView
from trade_apl_msgs.msg import TechTblSmaRecMsg as SmaMsg


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format

_CROSS_TYP_DICT = {
    SmaMsg.CROSS_TYP_GOLDEN: "Golden",
    SmaMsg.CROSS_TYP_DEAD: "Dead"
}

_CROSS_LVL_DICT = {
    SmaMsg.CROSS_LVL_LNGMID: "Long-Mid",
    SmaMsg.CROSS_LVL_MIDSHR: "Mid-Short"
}


class TechUi():

    def __init__(self, ui) -> None:

        self.logger = ros_com.get_logger()

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_gran)
        for obj in VALID_GRAN_LIST:
            ui.comboBox_tech_gran.addItem(obj.text)

        callback = self._on_fetch_tech_clicked
        ui.pushButton_tech_fetch.clicked.connect(callback)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        # set Tree View
        pdtreeview_sma = PandasTreeView(ui.widget_TreeView_tech_sma)

        selmdl = pdtreeview_sma.selectionModel()
        callback = self._on_selection_sma_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview_sma.header()
        callback = self._on_sma_header_sectionClicked
        header.sectionClicked.connect(callback)

        # chartview = CandlestickChartView(ui.widget_ChartView_tech)
        chartview = CandlestickChartView()

        col_cnt = ui.tableWidget_tech.columnCount()
        for i in reversed(range(col_cnt)):
            ui.tableWidget_tech.removeColumn(i)

        row_cnt = ui.tableWidget_tech.rowCount()
        for i in reversed(range(row_cnt)):
            ui.tableWidget_tech.removeRow(i)

        ui.tableWidget_tech.insertColumn(0)
        # set Item "Chart View"
        ui.tableWidget_tech.insertRow(0)
        ui.tableWidget_tech.setCellWidget(0, 0, chartview)
        ui.tableWidget_tech.setRowHeight(0, 500)

        # ---------- toolButton Trend ----------
        groupBox = QGroupBox()
        checkbox_sma = QCheckBox("単純移動平均")
        checkbox_ema = QCheckBox("指数平滑移動平均")
        checkbox_wma = QCheckBox("加重移動平均")
        checkbox_ich = QCheckBox("一目均衡表")
        checkbox_bb = QCheckBox("ボリンジャーバンド")
        checkbox_sma.stateChanged.connect(self._on_checkbox_stateChanged_sma)
        checkbox_ema.stateChanged.connect(self._on_checkbox_stateChanged_ema)
        checkbox_wma.stateChanged.connect(self._on_checkbox_stateChanged_wma)
        checkbox_ich.stateChanged.connect(self._on_checkbox_stateChanged_ich)
        checkbox_bb.stateChanged.connect(self._on_checkbox_stateChanged_bb)
        checkbox_sma.setChecked(True)
        checkbox_ema.setChecked(True)
        checkbox_wma.setChecked(True)
        checkbox_ich.setChecked(True)
        checkbox_bb.setChecked(True)
        vbox = QVBoxLayout()
        vbox.addWidget(checkbox_sma)
        vbox.addWidget(checkbox_ema)
        vbox.addWidget(checkbox_wma)
        vbox.addWidget(checkbox_ich)
        vbox.addWidget(checkbox_bb)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        menu=QMenu()
        checkableAction = QWidgetAction(menu)
        checkableAction.setDefaultWidget(groupBox);
        menu.addAction(checkableAction);
        ui.toolButton_tech_trn.setPopupMode(QToolButton.InstantPopup)
        ui.toolButton_tech_trn.setMenu(menu)
        self._menu_t = menu

        # ---------- toolButton Oscillator ----------
        groupBox = QGroupBox()
        checkbox_rsi = QCheckBox("RSI")
        checkbox_macd = QCheckBox("MACD")
        checkbox_stch = QCheckBox("ストキャスティクス")
        checkbox_rsi.stateChanged.connect(self._on_checkbox_stateChanged_rsi)
        checkbox_macd.stateChanged.connect(self._on_checkbox_stateChanged_macd)
        checkbox_stch.stateChanged.connect(self._on_checkbox_stateChanged_stch)
        checkbox_rsi.setChecked(True)
        checkbox_macd.setChecked(True)
        checkbox_stch.setChecked(True)
        vbox = QVBoxLayout()
        vbox.addWidget(checkbox_rsi)
        vbox.addWidget(checkbox_macd)
        vbox.addWidget(checkbox_stch)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        menu=QMenu()
        checkableAction = QWidgetAction(menu)
        checkableAction.setDefaultWidget(groupBox);
        menu.addAction(checkableAction);
        ui.toolButton_tech_osc.setPopupMode(QToolButton.InstantPopup)
        ui.toolButton_tech_osc.setMenu(menu)
        self._menu_o = menu

        self._chartview = chartview
        self._ui = ui
        self._pdtreeview_sma = pdtreeview_sma
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]

        self._init_ros_service()

    def _on_checkbox_stateChanged_sma(self, state):
        self.logger.debug("_on_checkbox_stateChanged_sma:{}".format(state))

    def _on_checkbox_stateChanged_ema(self, state):
        self.logger.debug("_on_checkbox_stateChanged_ema:{}".format(state))

    def _on_checkbox_stateChanged_wma(self, state):
        self.logger.debug("_on_checkbox_stateChanged_wma:{}".format(state))

    def _on_checkbox_stateChanged_ich(self, state):
        self.logger.debug("_on_checkbox_stateChanged_ich:{}".format(state))

    def _on_checkbox_stateChanged_bb(self, state):
        self.logger.debug("_on_checkbox_stateChanged_bb:{}".format(state))

    def _on_checkbox_stateChanged_rsi(self, state):
        self.logger.debug("_on_checkbox_stateChanged_rsi:{}".format(state))

    def _on_checkbox_stateChanged_macd(self, state):
        self.logger.debug("_on_checkbox_stateChanged_macd:{}".format(state))

    def _on_checkbox_stateChanged_stch(self, state):
        self.logger.debug("_on_checkbox_stateChanged_stch:{}".format(state))

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._init_ros_service()

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._init_ros_service()

    def _init_ros_service(self):
        ns = self._inst_param.namespace + "_" \
            + self._gran_param.namespace + "/"

        # Create service client "tech_monitor"
        srv_type = TechMntSrv
        srv_name = "tech_monitor"
        fullname = ns + srv_name
        self._srv_cli = ros_com.get_node().create_client(srv_type, fullname)

        # Create service client "tech_chart_monitor"
        srv_type = TechChartMntSrv
        srv_name = "tech_chart_monitor"
        fullname = ns + srv_name
        self._srv_chart_cli = ros_com.get_node().create_client(srv_type, fullname)

    def _on_sma_header_sectionClicked(self, logical_index):
        self._pdtreeview_sma.show_header_menu(logical_index)

    def _on_fetch_tech_clicked(self):
        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._srv_cli.service_is_ready():
            self.logger.error("service server [{}] not to become ready"
                              .format(inst_param.text))
        else:
            # fetch Tech data
            req = TechMntSrv.Request()
            rsp = ros_com.call_servive_sync(self._srv_cli, req)

            # ---------- compose Table "SMA" ----------
            tbl = []
            for rec in rsp.tbl_sma:
                # date_ = rec.datetime.split("T")[0]
                record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                          rec.cross_type,
                          rec.cross_level,
                          rec.angle_s,
                          rec.angle_m,
                          rec.angle_l,
                          ]
                tbl.append(record)

            columns = [ColNameSma.DATETIME.value,
                       ColNameSma.CRS_TYP.value,
                       ColNameSma.CRS_LVL.value,
                       ColNameSma.ANG_S.value,
                       ColNameSma.ANG_M.value,
                       ColNameSma.ANG_L.value
                       ]
            df_sma = pd.DataFrame(tbl, columns=columns)

            index = ColNameSma.DATETIME.value
            df_sma.set_index(index, inplace=True)

            # ---------- compose Table "SMA" for TreeView ----------
            if gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_YMDHMS
            tbl = []
            for idx, row in df_sma.iterrows():
                record = [idx.strftime(fmt),
                          _CROSS_TYP_DICT[int(row[ColNameSma.CRS_TYP.value])],
                          _CROSS_LVL_DICT[int(row[ColNameSma.CRS_LVL.value])],
                          utl.roundf(row[ColNameSma.ANG_S.value], digit=inst_param.digit),
                          utl.roundf(row[ColNameSma.ANG_M.value], digit=inst_param.digit),
                          utl.roundf(row[ColNameSma.ANG_L.value], digit=inst_param.digit)
                          ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=columns)
            index = ColNameSma.DATETIME.value
            df.set_index(index, inplace=True)

            self._pdtreeview_sma.set_dataframe(df)
            selmdl = self._pdtreeview_sma.selectionModel()
            callback = self._on_selection_sma_changed
            selmdl.selectionChanged.connect(callback)

            self._df_sma = df_sma

            self.logger.debug("----- check Head & Tail DataFrame -----")
            self.logger.debug("  << ---------- df_sma ---------- >>")
            """
            self.logger.debug("\n  << --- Head --- >>\n{}".format(self._df_ohlc[:6]))
            self.logger.debug("\n  << --- Tail --- >>\n{}".format(self._df_ohlc[-5:]))
            self.logger.debug("\n  << --- Head --- >>\n{}".format(df_sma))
            """

    def _on_selection_sma_changed(self, selected, _):

        self.logger.debug("----- _on_selection_sma_changed -----")
        if not selected.isEmpty():

            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("service server [{}] not to become ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview_sma.proxy
                dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)

                self._show_chart(dt_str)
                self._ui.tableWidget_tech.setEnabled(True)

    def _show_chart(self, dt_str: str):

        df_sma = self._df_sma

        if self._gran_param == GranParam.D:
            fmt = FMT_DATE_YMD
            dt_ = dt.datetime.strptime(dt_str, fmt)
            lvl = df_sma.index.get_level_values(ColNameSma.DATETIME.value)
            trg_dt = lvl[dt_ < lvl][0]
        else:
            fmt = FMT_YMDHMS
            dt_ = dt.datetime.strptime(dt_str, fmt)
            lvl = df_sma.index.get_level_values(ColNameSma.DATETIME.value)
            trg_dt = lvl[dt_ == lvl]

        # fetch Tech chart data
        req = TechChartMntSrv.Request()
        req.datetime = trg_dt.strftime(FMT_YMDHMS)
        rsp = ros_com.call_servive_sync(self._srv_chart_cli, req)

        # ---------- compose Table "OHLC" ----------
        tbl = []
        for rec in rsp.tbl_ohlc:
            # date_ = rec.datetime.split("T")[0]
            # record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
            if self._gran_param == GranParam.D:
                dt_ = rec.datetime.split("T")[0]
            else:
                dt_ = rec.datetime
            record = [dt_,
                      rec.mid_o, rec.mid_h, rec.mid_l, rec.mid_c,
                      rec.sma_s, rec.sma_m, rec.sma_l,
                      rec.ema_s, rec.ema_m, rec.ema_l,
                      rec.wma_s, rec.wma_m, rec.wma_l,
                      rec.ichmk_base, rec.ichmk_conv,
                      rec.ichmk_sapn_a, rec.ichmk_sapn_b, rec.ichmk_lag,
                      rec.blngr_base,
                      rec.blngr_ps1, rec.blngr_ps2, rec.blngr_ps3,
                      rec.blngr_ns1, rec.blngr_ns2, rec.blngr_ns3,
                      rec.rsi_sma, rec.rsi_ema,
                      rec.macd_macd, rec.macd_sig,
                      rec.stcha_k, rec.stcha_d, rec.stcha_slow_d,
                      ]
            tbl.append(record)

        columns = [ColNameOhlc.DATETIME.value,
                   CandlestickChartView.CandleLabel.OP.value,
                   CandlestickChartView.CandleLabel.HI.value,
                   CandlestickChartView.CandleLabel.LO.value,
                   CandlestickChartView.CandleLabel.CL.value
                   ] + ColNameTrnd.to_list_all() + ColNameOsci.to_list()

        df_all = pd.DataFrame(tbl, columns=columns)

        index = ColNameOhlc.DATETIME.value
        df_all.set_index(index, inplace=True)
        df_all.where(df_all > -1000, inplace=True)

        columns = [CandlestickChartView.CandleLabel.OP.value,
                   CandlestickChartView.CandleLabel.HI.value,
                   CandlestickChartView.CandleLabel.LO.value,
                   CandlestickChartView.CandleLabel.CL.value
                   ] + ColNameTrnd.to_list_all()
        df_trnd = df_all[columns]

        max_y = df_trnd.max().max()
        min_y = df_trnd.min().min()
        trg_loc = df_trnd.index.get_loc(dt_str)

        self._chartview.set_max_y(max_y)
        self._chartview.set_min_y(min_y)
        self._chartview.update(df_all, trg_loc, self._inst_param)

    def _get_dataframe(self):

        is_selected = self._pdtreeview_sma.is_selected()
        dftv = self._pdtreeview_sma.get_dataframe(is_selected=is_selected)

        date_list = sorted(dftv.index.to_list())
        df = self._df_base.loc[(date_list), :]

        return df
