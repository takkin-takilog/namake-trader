from enum import Enum
from dataclasses import dataclass
import pandas as pd
import datetime as dt
from typing import List
from PySide2.QtCore import Qt
from PySide2.QtGui import QColor, QPen, QPainter, QPainterPath, QPixmap
from PySide2.QtWidgets import QCheckBox, QGroupBox, QVBoxLayout
from PySide2.QtWidgets import QMenu, QWidgetAction
from PySide2.QtWidgets import QToolButton
from trade_apl_msgs.srv import TechSmaMntSrv
from trade_apl_msgs.srv import TechMacdMntSrv
from trade_apl_msgs.srv import TechChartMntSrv
from trade_monitor.widget_base import PandasTreeView
from trade_monitor import utility as utl
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.constant import FMT_YMDHMS, FMT_DATE_YMD, FMT_DISP_YMDHMS
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.constant import ColOhlc
from trade_monitor.tech.constant import ColTrnd
from trade_monitor.tech.constant import ColOsci
from trade_monitor.tech.constant import ColSma
from trade_monitor.tech.constant import ColMacdGdc, ColMacdZlc
from trade_monitor.tech.constant import OsciTyp
from trade_monitor.tech.constant import SMA_CRS_TYP_DICT, SMA_CRS_LVL_DICT
from trade_monitor.tech.constant import MACD_GDC_SIG_TYP_DICT, MACD_GDC_EXIT_DICT
from trade_monitor.tech.constant import MACD_ZLC_SIG_TYP_DICT, MACD_ZLC_EXIT_DICT
from trade_monitor.tech.widget import CandlestickChartView
from trade_monitor.tech.widget import LineChartViewTech as LineChartView
from trade_monitor import ros_common as ros_com
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


@dataclass
class OscChartParam():
    """
    Oscillator Chart Parameter.
    """
    osci_type: OsciTyp = None
    chartview: LineChartView = None
    df_columns: int = None


class TechUi():

    def __init__(self, ui) -> None:

        self.logger = ros_com.get_logger()

        self._ohlc_columns = [CandlestickChartView.CandleLabel.OP.value,
                              CandlestickChartView.CandleLabel.HI.value,
                              CandlestickChartView.CandleLabel.LO.value,
                              CandlestickChartView.CandleLabel.CL.value
                              ]

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

        # --------------- Tree View ---------------
        # ----- SMA -----
        pdtreeview_sma = PandasTreeView(ui.widget_TreeView_tech_sma)

        selmdl = pdtreeview_sma.selectionModel()
        callback = self._on_sma_selection_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview_sma.header()
        callback = self._on_sma_header_sectionClicked
        header.sectionClicked.connect(callback)

        # ----- MACD(Golden/Death cross) -----
        pdtreeview_macd_gdc = PandasTreeView(ui.widget_TreeView_tech_macd_gdc)

        selmdl = pdtreeview_macd_gdc.selectionModel()
        callback = self._on_macd_gdc_selection_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview_macd_gdc.header()
        callback = self._on_macd_gdc_header_sectionClicked
        header.sectionClicked.connect(callback)

        # ----- MACD(Zero line cross) -----
        pdtreeview_macd_zlc = PandasTreeView(ui.widget_TreeView_tech_macd_zlc)

        selmdl = pdtreeview_macd_zlc.selectionModel()
        callback = self._on_macd_zlc_selection_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview_macd_zlc.header()
        callback = self._on_macd_zlc_header_sectionClicked
        header.sectionClicked.connect(callback)

        # --------------- Candlestick Chart View ---------------
        chartview_cndl = CandlestickChartView()

        col_cnt = ui.tableWidget_tech.columnCount()
        for i in reversed(range(col_cnt)):
            ui.tableWidget_tech.removeColumn(i)

        row_cnt = ui.tableWidget_tech.rowCount()
        for i in reversed(range(row_cnt)):
            ui.tableWidget_tech.removeRow(i)

        ui.tableWidget_tech.insertColumn(0)
        # set Item "Chart View"
        ui.tableWidget_tech.insertRow(0)
        ui.tableWidget_tech.setCellWidget(0, 0, chartview_cndl)
        ui.tableWidget_tech.setRowHeight(0, 500)

        # ---------- toolButton Trend ----------
        groupBox = QGroupBox()
        checkbox_sma = QCheckBox("単純移動平均")
        checkbox_ema = QCheckBox("指数平滑移動平均")
        checkbox_wma = QCheckBox("加重移動平均")
        checkbox_ich = QCheckBox("一目均衡表")
        checkbox_bb = QCheckBox("ボリンジャーバンド")
        checkbox_sma.setChecked(False)
        checkbox_ema.setChecked(False)
        checkbox_wma.setChecked(False)
        checkbox_ich.setChecked(False)
        checkbox_bb.setChecked(False)
        checkbox_sma.stateChanged.connect(self._on_checkbox_trn_stateChanged)
        checkbox_ema.stateChanged.connect(self._on_checkbox_trn_stateChanged)
        checkbox_wma.stateChanged.connect(self._on_checkbox_trn_stateChanged)
        checkbox_ich.stateChanged.connect(self._on_checkbox_trn_stateChanged)
        checkbox_bb.stateChanged.connect(self._on_checkbox_trn_stateChanged)
        vbox = QVBoxLayout()
        vbox.addWidget(checkbox_sma)
        vbox.addWidget(checkbox_ema)
        vbox.addWidget(checkbox_wma)
        vbox.addWidget(checkbox_ich)
        vbox.addWidget(checkbox_bb)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        menu = QMenu()
        checkableAction = QWidgetAction(menu)
        checkableAction.setDefaultWidget(groupBox)
        menu.addAction(checkableAction)
        ui.toolButton_tech_trn.setPopupMode(QToolButton.InstantPopup)
        ui.toolButton_tech_trn.setMenu(menu)
        self._menu_t = menu

        self._checkbox_sma = checkbox_sma
        self._checkbox_ema = checkbox_ema
        self._checkbox_wma = checkbox_wma
        self._checkbox_ich = checkbox_ich
        self._checkbox_bb = checkbox_bb
        self._trend_columns = []

        # ---------- toolButton Oscillator ----------
        groupBox = QGroupBox()
        checkbox_rsi = QCheckBox("RSI")
        checkbox_macd = QCheckBox("MACD")
        checkbox_stch = QCheckBox("ストキャスティクス")
        checkbox_rsi.setChecked(False)
        checkbox_macd.setChecked(False)
        checkbox_stch.setChecked(False)
        checkbox_rsi.stateChanged.connect(self._on_checkbox_osc_stateChanged)
        checkbox_macd.stateChanged.connect(self._on_checkbox_osc_stateChanged)
        checkbox_stch.stateChanged.connect(self._on_checkbox_osc_stateChanged)
        vbox = QVBoxLayout()
        vbox.addWidget(checkbox_rsi)
        vbox.addWidget(checkbox_macd)
        vbox.addWidget(checkbox_stch)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        menu = QMenu()
        checkableAction = QWidgetAction(menu)
        checkableAction.setDefaultWidget(groupBox)
        menu.addAction(checkableAction)
        ui.toolButton_tech_osc.setPopupMode(QToolButton.InstantPopup)
        ui.toolButton_tech_osc.setMenu(menu)
        self._menu_o = menu

        self._checkbox_rsi = checkbox_rsi
        self._checkbox_macd = checkbox_macd
        self._checkbox_stch = checkbox_stch
        self._osci_columns = []
        self._osc_chart_list = []

        # ==================== RSI config ====================
        self._config_tbl_rsi = []
        # --------------- SMA ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "RSI(SMA)"
        self._config_tbl_rsi.append([ColOsci.RSI_SMA.value, pen, name])
        # --------------- EMA ---------------
        """
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        self._config_tbl_rsi.append([ColOsci.RSI_EMA.value, pen])
        """

        # ==================== MACD config ====================
        self._config_tbl_macd = []
        # --------------- MACD ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "MACD"
        self._config_tbl_macd.append([ColOsci.MACD_MACD.value, pen, name])
        # --------------- Signal ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "Signal"
        self._config_tbl_macd.append([ColOsci.MACD_SIG.value, pen, name])

        # ==================== Stochastics config ====================
        self._config_tbl_stcha = []
        # --------------- %K ---------------
        pen = QPen()
        pen.setColor(Qt.red)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "%K"
        self._config_tbl_stcha.append([ColOsci.STCHA_K.value, pen, name])
        # --------------- %D ---------------
        pen = QPen()
        pen.setColor(Qt.blue)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "%D"
        self._config_tbl_stcha.append([ColOsci.STCHA_D.value, pen, name])
        # --------------- Slow%D ---------------
        pen = QPen()
        pen.setColor(Qt.magenta)
        pen.setWidth(1)
        pen.setStyle(Qt.SolidLine)
        name = "Slow%D"
        self._config_tbl_stcha.append([ColOsci.STCHA_SD.value, pen, name])

        # ---------- set field ----------
        self._show_columns = self._ohlc_columns
        self._chartview_cndl = chartview_cndl
        self._ui = ui
        self._pdtreeview_sma = pdtreeview_sma
        self._pdtreeview_macd_gdc = pdtreeview_macd_gdc
        self._pdtreeview_macd_zlc = pdtreeview_macd_zlc
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]
        self._target_datetime = None

        self._init_ros_service()

    def _on_checkbox_trn_stateChanged(self, _):

        trend_columns = []
        if self._checkbox_sma.checkState() == Qt.Checked:
            trend_columns.extend(ColTrnd.to_list_sma())
        if self._checkbox_ema.checkState() == Qt.Checked:
            trend_columns.extend(ColTrnd.to_list_ema())
        if self._checkbox_wma.checkState() == Qt.Checked:
            trend_columns.extend(ColTrnd.to_list_wma())
        if self._checkbox_ich.checkState() == Qt.Checked:
            trend_columns.extend(ColTrnd.to_list_ichmk())
        if self._checkbox_bb.checkState() == Qt.Checked:
            trend_columns.extend(ColTrnd.to_list_bb())

        self._trend_columns = trend_columns
        self._show_columns = self._ohlc_columns + self._trend_columns + self._osci_columns

        if not self._target_datetime is None:
            self._chartview_cndl.update(self._df_all[self._show_columns],
                                        self._target_datetime,
                                        self._inst_param)

    def _on_checkbox_osc_stateChanged(self, _):

        osci_columns = []
        if self._checkbox_rsi.checkState() == Qt.Checked:
            osci_columns.extend(ColOsci.to_list_rsi())
        if self._checkbox_macd.checkState() == Qt.Checked:
            osci_columns.extend(ColOsci.to_list_macd())
        if self._checkbox_stch.checkState() == Qt.Checked:
            osci_columns.extend(ColOsci.to_list_stochastic())

        self._osci_columns = osci_columns
        self._show_columns = self._ohlc_columns + self._trend_columns + self._osci_columns

        row_cnt = self._ui.tableWidget_tech.rowCount()
        for i in reversed(range(1, row_cnt)):
            self._ui.tableWidget_tech.removeRow(i)

        self._osc_chart_list = []
        insert_row_pos = 0
        if self._checkbox_rsi.checkState() == Qt.Checked:
            insert_row_pos += 1
            param = self._generate_osc_chart_param(insert_row_pos,
                                                   OsciTyp.RSI,
                                                   self._config_tbl_rsi,
                                                   ColOsci.to_list_rsi(),
                                                   100, 0)
            self._osc_chart_list.append(param)
        if self._checkbox_macd.checkState() == Qt.Checked:
            insert_row_pos += 1
            param = self._generate_osc_chart_param(insert_row_pos,
                                                   OsciTyp.MACD,
                                                   self._config_tbl_macd,
                                                   ColOsci.to_list_macd())
            self._osc_chart_list.append(param)
        if self._checkbox_stch.checkState() == Qt.Checked:
            insert_row_pos += 1
            param = self._generate_osc_chart_param(insert_row_pos,
                                                   OsciTyp.STOCHASTICS,
                                                   self._config_tbl_stcha,
                                                   ColOsci.to_list_stochastic(),
                                                   100, 0)
            self._osc_chart_list.append(param)

        if not self._target_datetime is None:
            for osc_chart in self._osc_chart_list:
                df = self._df_all[osc_chart.df_columns]
                osc_chart.chartview.update(df,
                                           self._target_datetime,
                                           self._inst_param)

    def _generate_osc_chart_param(self,
                                  insert_row_pos: int,
                                  osci_type: OsciTyp,
                                  config_tbl: List,
                                  columns: List[str],
                                  max_y: float = None,
                                  min_y: float = None):
        chartview = LineChartView(config_tbl)
        chartview.set_max_y(max_y)
        chartview.set_min_y(min_y)
        chartview.set_title(osci_type.value)
        self._ui.tableWidget_tech.insertRow(insert_row_pos)
        self._ui.tableWidget_tech.setCellWidget(insert_row_pos, 0, chartview)
        self._ui.tableWidget_tech.setRowHeight(insert_row_pos, 200)
        return OscChartParam(osci_type, chartview, columns)

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._init_ros_service()

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._init_ros_service()

    def _init_ros_service(self):
        ns = self._inst_param.namespace + "_" \
            + self._gran_param.namespace + "/"

        # Create service client "tech_sma_monitor"
        srv_type = TechSmaMntSrv
        srv_name = "tech_sma_monitor"
        fullname = ns + srv_name
        self._srv_sma_cli = ros_com.get_node().create_client(srv_type, fullname)

        # Create service client "tech_macd_gdc_monitor"
        srv_type = TechMacdMntSrv
        srv_name = "tech_macd_monitor"
        fullname = ns + srv_name
        self._srv_macd_cli = ros_com.get_node().create_client(srv_type, fullname)

        # Create service client "tech_chart_monitor"
        srv_type = TechChartMntSrv
        srv_name = "tech_chart_monitor"
        fullname = ns + srv_name
        self._srv_chart_cli = ros_com.get_node().create_client(srv_type, fullname)

    def _on_fetch_tech_clicked(self):

        tab_name = self._ui.tabWidget_tech.currentWidget().objectName()
        if tab_name == "sma":
            self._on_fetch_tech_sma_clicked()
        elif tab_name == "macd":
            self._on_fetch_tech_macd_clicked()
            self.logger.debug("---------- MACD ----------")
        else:
            self.logger.debug("---------- tab_name ----------")
            self.logger.debug("{}".format(tab_name))

    def _on_fetch_tech_sma_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._srv_sma_cli.service_is_ready():
            self.logger.error("service server [{}] not to become ready"
                              .format(inst_param.text))
        else:
            # fetch Tech data
            req = TechSmaMntSrv.Request()
            rsp = ros_com.call_servive_sync(self._srv_sma_cli, req)

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

            df_sma = pd.DataFrame(tbl, columns=ColSma.to_list())

            index = ColSma.DATETIME.value
            df_sma.set_index(index, inplace=True)

            # ---------- compose Table "SMA" for TreeView ----------
            if gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_DISP_YMDHMS
            tbl = []
            for idx, row in df_sma.iterrows():
                record = [idx.strftime(fmt),
                          SMA_CRS_TYP_DICT[int(row[ColSma.CRS_TYP.value])],
                          SMA_CRS_LVL_DICT[int(row[ColSma.CRS_LVL.value])],
                          utl.roundf(row[ColSma.ANG_S.value], digit=inst_param.digit),
                          utl.roundf(row[ColSma.ANG_M.value], digit=inst_param.digit),
                          utl.roundf(row[ColSma.ANG_L.value], digit=inst_param.digit)
                          ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=ColSma.to_list())
            index = ColSma.DATETIME.value
            df.set_index(index, inplace=True)

            self._pdtreeview_sma.set_dataframe(df)
            selmdl = self._pdtreeview_sma.selectionModel()
            callback = self._on_sma_selection_changed
            selmdl.selectionChanged.connect(callback)

            self._df_sma = df_sma

            self.logger.debug("----- check Head & Tail DataFrame -----")
            self.logger.debug("  << ---------- df_sma ---------- >>")
            """
            self.logger.debug("\n  << --- Head --- >>\n{}".format(df_sma))
            """

    def _on_fetch_tech_macd_clicked(self):

        inst_param = self._inst_param
        gran_param = self._gran_param

        if not self._srv_macd_cli.service_is_ready():
            self.logger.error("service server [{}] not to become ready"
                              .format(inst_param.text))
        else:
            # fetch Tech data
            req = TechMacdMntSrv.Request()
            rsp = ros_com.call_servive_sync(self._srv_macd_cli, req)

            # ---------- compose Table "MACD(Dolden/Death cross)" ----------
            tbl = []
            for rec in rsp.tbl_macd_gdc:
                record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                          rec.signal_type,
                          rec.signal_value,
                          rec.macd_slope,
                          rec.exit_cond,
                          dt.datetime.strptime(rec.exit_datetime, FMT_YMDHMS),
                          rec.profit_loss_price,
                          rec.max_loss_price,
                          rec.add_entry_id,
                          ]
                tbl.append(record)

            df_macd_gdc = pd.DataFrame(tbl, columns=ColMacdGdc.to_list())

            index = ColMacdGdc.DATETIME.value
            df_macd_gdc.set_index(index, inplace=True)

            # ---------- compose Table "MACD(Zero line cross)" ----------
            tbl = []
            for rec in rsp.tbl_macd_zlc:
                record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                          rec.add_entry_id,
                          rec.signal_type,
                          rec.exit_cond,
                          dt.datetime.strptime(rec.exit_datetime, FMT_YMDHMS),
                          rec.profit_loss_price,
                          rec.max_loss_price,
                          ]
                tbl.append(record)

            df_macd_zlc = pd.DataFrame(tbl, columns=ColMacdZlc.to_list())

            index = ColMacdZlc.DATETIME.value
            df_macd_zlc.set_index(index, inplace=True)

            # ---------- compose Table "MACD(Dolden/Death cross)" for TreeView ----------
            if gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_DISP_YMDHMS
            tbl = []
            for idx, row in df_macd_gdc.iterrows():
                record = [idx.strftime(fmt),
                          MACD_GDC_SIG_TYP_DICT[int(row[ColMacdGdc.SIG_TYP.value])],
                          utl.roundf(row[ColMacdGdc.SIG_VAL.value], digit=inst_param.digit),
                          utl.roundf(row[ColMacdGdc.MACD_SLP.value], digit=inst_param.digit+1),
                          MACD_GDC_EXIT_DICT[int(row[ColMacdGdc.EXIT_COND.value])],
                          row[ColMacdGdc.EXIT_DATETIME.value].strftime(fmt),
                          utl.roundf(row[ColMacdGdc.PL_PRICE.value], digit=inst_param.digit),
                          utl.roundf(row[ColMacdGdc.MAX_LOSS_PRICE.value], digit=inst_param.digit),
                          int(row[ColMacdGdc.ADD_ENTRY_ID.value])
                          ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=ColMacdGdc.to_list())
            index = ColMacdGdc.DATETIME.value
            df.set_index(index, inplace=True)
            self._pdtreeview_macd_gdc.set_dataframe(df)
            selmdl = self._pdtreeview_macd_gdc.selectionModel()
            callback = self._on_macd_gdc_selection_changed
            selmdl.selectionChanged.connect(callback)

            # ---------- compose Table "MACD(Zero line cross)" for TreeView ----------
            if gran_param == GranParam.D:
                fmt = FMT_DATE_YMD
            else:
                fmt = FMT_DISP_YMDHMS
            tbl = []
            for idx, row in df_macd_zlc.iterrows():
                record = [idx.strftime(fmt),
                          int(row[ColMacdZlc.ADD_ENTRY_ID.value]),
                          MACD_ZLC_SIG_TYP_DICT[int(row[ColMacdZlc.SIG_TYP.value])],
                          MACD_ZLC_EXIT_DICT[int(row[ColMacdZlc.EXIT_COND.value])],
                          row[ColMacdZlc.EXIT_DATETIME.value].strftime(fmt),
                          utl.roundf(row[ColMacdZlc.PL_PRICE.value], digit=inst_param.digit),
                          utl.roundf(row[ColMacdZlc.MAX_LOSS_PRICE.value], digit=inst_param.digit),
                          ]
                tbl.append(record)
            df = pd.DataFrame(tbl, columns=ColMacdZlc.to_list())
            index = ColMacdZlc.DATETIME.value
            df.set_index(index, inplace=True)
            self._pdtreeview_macd_zlc.set_dataframe(df)
            selmdl = self._pdtreeview_macd_zlc.selectionModel()
            callback = self._on_macd_zlc_selection_changed
            selmdl.selectionChanged.connect(callback)

            self._df_macd_gdc = df_macd_gdc
            self._df_macd_zlc = df_macd_zlc

    def _on_sma_selection_changed(self, selected, _):
        self.logger.debug("----- _on_sma_selection_changed -----")
        if not selected.isEmpty():
            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("service server [{}] not to become ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview_sma.proxy
                dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
                if self._gran_param == GranParam.D:
                    fmt = FMT_DATE_YMD
                    dt_ = dt.datetime.strptime(dt_str, fmt)
                    lvl = self._df_sma.index.get_level_values(ColSma.DATETIME.value)
                    idx_dt = lvl[dt_ < lvl][0]
                else:
                    fmt = FMT_DISP_YMDHMS
                    idx_dt = dt.datetime.strptime(dt_str, fmt)

                self._draw_chart(dt_str, idx_dt)
                self._ui.tableWidget_tech.setEnabled(True)

    def _on_macd_gdc_selection_changed(self, selected, _):
        self.logger.debug("----- _on_macd_gdc_selection_changed -----")
        if not selected.isEmpty():
            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("service server [{}] not to become ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview_macd_gdc.proxy
                dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
                if self._gran_param == GranParam.D:
                    fmt = FMT_DATE_YMD
                    dt_ = dt.datetime.strptime(dt_str, fmt)
                    lvl = self._df_macd_gdc.index.get_level_values(ColSma.DATETIME.value)
                    idx_dt = lvl[dt_ < lvl][0]
                else:
                    fmt = FMT_DISP_YMDHMS
                    idx_dt = dt.datetime.strptime(dt_str, fmt)

                self._draw_chart(dt_str, idx_dt)
                self._ui.tableWidget_tech.setEnabled(True)

    def _on_macd_zlc_selection_changed(self, selected, _):
        self.logger.debug("----- _on_macd_zlc_selection_changed -----")
        if not selected.isEmpty():
            if not self._srv_chart_cli.service_is_ready():
                self.logger.error("service server [{}] not to become ready"
                                  .format(self._inst_param.text))
            else:
                model_index = selected.at(0).indexes()[0]
                r = model_index.row()
                proxy = self._pdtreeview_macd_zlc.proxy
                dt_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)
                if self._gran_param == GranParam.D:
                    fmt = FMT_DATE_YMD
                    dt_ = dt.datetime.strptime(dt_str, fmt)
                    lvl = self._df_macd_zlc.index.get_level_values(ColSma.DATETIME.value)
                    idx_dt = lvl[dt_ < lvl][0]
                else:
                    fmt = FMT_DISP_YMDHMS
                    idx_dt = dt.datetime.strptime(dt_str, fmt)

                self._draw_chart(dt_str, idx_dt)
                self._ui.tableWidget_tech.setEnabled(True)

    def _on_sma_header_sectionClicked(self, logical_index):
        self._pdtreeview_sma.show_header_menu(logical_index)

    def _on_macd_gdc_header_sectionClicked(self, logical_index):
        self._pdtreeview_macd_gdc.show_header_menu(logical_index)

    def _on_macd_zlc_header_sectionClicked(self, logical_index):
        self._pdtreeview_macd_zlc.show_header_menu(logical_index)

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

        columns = [ColOhlc.DATETIME.value] + self._ohlc_columns \
            + ColTrnd.to_list_all() + ColOsci.to_list_all()

        df_all = pd.DataFrame(tbl, columns=columns)

        index = ColOhlc.DATETIME.value
        df_all.set_index(index, inplace=True)
        df_all.where(df_all > -1000, inplace=True)

        columns = self._ohlc_columns + ColTrnd.to_list_all()
        df_trnd = df_all[columns]

        max_y = df_trnd.max().max()
        min_y = df_trnd.min().min()
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

            # self.logger.debug("{}".format(osc_chart.df_columns))

    def _get_dataframe(self):

        is_selected = self._pdtreeview_sma.is_selected()
        dftv = self._pdtreeview_sma.get_dataframe(is_selected=is_selected)

        date_list = sorted(dftv.index.to_list())
        df = self._df_base.loc[(date_list), :]

        return df
