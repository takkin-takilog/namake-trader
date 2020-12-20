import pandas as pd
import datetime as dt
from PySide2.QtCore import Qt
from trade_apl_msgs.srv import TtmMntSrv
from trade_monitor.base import PandasTreeView
from trade_monitor import utilities as utl
from trade_monitor.utilities import INST_MSG_LIST
from trade_monitor.utilities import GRAN_FREQ_DICT
from trade_monitor.utilities import (FMT_DTTM_API,
                                     FMT_DATE_YMD,
                                     FMT_TIME_HM,
                                     FMT_TIME_HMS
                                     )
from trade_monitor.ttm.chart import CandlestickChartTtm
from trade_monitor.ttm.ttm_details import TtmDetails
from trade_monitor.ttm.ttm_common import (COL_DATE,
                                          COL_TIME,
                                          COL_O,
                                          COL_H,
                                          COL_L,
                                          COL_C,
                                          COL_WEEKDAY_ID,
                                          COL_GOTO_ID,
                                          COL_IS_GOTO,
                                          COL_GAP_TYP,
                                          COL_DATA_TYP,
                                          COL_MONTH
                                          )
from trade_monitor.ttm.ttm_common import GAP_TYP_CO
from trade_monitor.ttm.ttm_common import WEEKDAY_ID_DICT
from trade_monitor.ttm.ttm_common import GOTODAY_ID_DICT

pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
pd.options.display.float_format = '{:.3f}'.format


class TtmUi():

    """
    _GAP_TYP_DICT = {
        1: "High  - Open",
        2: "Low   - Open",
        3: "Close - Open"
    }
    """

    _TREEVIEW_ITEM_DATE = "Date"
    _TREEVIEW_ITEM_WEEKDAY = "Weekday"
    _TREEVIEW_ITEM_GOTODAY = "Goto day"
    _TREEVIEW_ITEM_DATATYP = "Data type"

    _TREEVIEW_HEADERS = [
        _TREEVIEW_ITEM_DATE,
        _TREEVIEW_ITEM_WEEKDAY,
        _TREEVIEW_ITEM_GOTODAY,
        _TREEVIEW_ITEM_DATATYP
    ]

    """
    _COL_DATE = "Date"
    _COL_TIME = "Time"
    _COL_O = "O"
    _COL_H = "H"
    _COL_L = "L"
    _COL_C = "C"
    _COL_WEEKDAY_ID = "Weekday_id"
    _COL_GOTO_ID = "Goto_id"
    _COL_IS_GOTO = "Is_Goto"
    _COL_GAP_TYP = "Gap_type"
    _COL_DATA_TYP = "Data_type"
    _COL_MONTH = "Month"

    _GAP_TYP_HO = 1    # High - Open price
    _GAP_TYP_LO = 2    # Low - Open price
    _GAP_TYP_CO = 3    # Close - Open price

    _DATA_TYP_HO_MEAN = 1   # Mean of High - Open price
    _DATA_TYP_HO_STD = 2    # Std of High - Open price
    _DATA_TYP_LO_MEAN = 3   # Mean of Low - Open price
    _DATA_TYP_LO_STD = 4    # Std of Low - Open price
    _DATA_TYP_CO_MEAN = 5   # Mean of Close - Open price
    _DATA_TYP_CO_STD = 6    # Std of Close - Open price
    _DATA_TYP_CO_CSUM = 7   # Cumsum of Close - Open price
    """

    _CDL_COLUMNS = [CandlestickChartTtm.COL_NAME_OP,
                    CandlestickChartTtm.COL_NAME_HI,
                    CandlestickChartTtm.COL_NAME_LO,
                    CandlestickChartTtm.COL_NAME_CL
                    ]

    def __init__(self, ui) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_ttm_inst)
        for obj in INST_MSG_LIST:
            ui.comboBox_ttm_inst.addItem(obj.text)

        callback = self._on_fetch_ttm_clicked
        ui.pushButton_ttm_fetch.clicked.connect(callback)

        """
        qstd_itm_mdl = QStandardItemModel()
        sel_mdl = QItemSelectionModel(qstd_itm_mdl)

        callback = self._on_selection_ttm_changed
        sel_mdl.selectionChanged.connect(callback)
        """

        callback = self._on_ttm_details_clicked
        ui.pushButton_ttm_details.clicked.connect(callback)

        # set Tree View
        pdtreeview = PandasTreeView(ui.widget_TreeView_ttm)

        selmdl = pdtreeview.selectionModel()
        callback = self._on_selection_ttm_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview.header()
        callback = self._on_view_header_sectionClicked
        header.sectionClicked.connect(callback)

        chart = CandlestickChartTtm(ui.widget_ChartView_ttm)

        # Create service client "ttm_monitor"
        srv_type = TtmMntSrv
        srv_name = "ttm_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = utl.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._widget_details = TtmDetails()

        # self._qstd_itm_mdl = qstd_itm_mdl
        self._chart = chart

        self._ui = ui
        self._pdtreeview = pdtreeview
        self._gran_id = 0
        self._srv_cli_list = srv_cli_list
        self._logger = utl.get_logger()

    def _on_view_header_sectionClicked(self, logical_index):
        self._pdtreeview.show_header_menu(logical_index)

    def _on_fetch_ttm_clicked(self):
        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        inst_msg = INST_MSG_LIST[inst_idx]

        # fetch TTM data
        req = TtmMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            self._logger.error("service server [{}] not to become ready"
                               .format(inst_msg.text))
        else:

            rsp = utl.call_servive_sync(srv_cli, req, timeout_sec=10.0)
            self._gran_id = rsp.gran_id

            start_time_str = rsp.start_time
            end_time_str = rsp.end_time
            freq = GRAN_FREQ_DICT[rsp.gran_id]
            time_range_list = pd.date_range(start_time_str,
                                            end_time_str,
                                            freq=freq
                                            ).strftime(FMT_TIME_HM).to_list()

            # ---------- compose Table "OHLC" ----------
            tbl = []
            for rec in rsp.ttm_tbl_ohlc:
                record = [rec.date,
                          rec.time,
                          rec.open,
                          rec.high,
                          rec.low,
                          rec.close
                          ]
                tbl.append(record)

            columns = [COL_DATE,
                       COL_TIME,
                       COL_O,
                       COL_H,
                       COL_L,
                       COL_C,
                       ]
            df_ohlc = pd.DataFrame(tbl, columns=columns)

            index = [COL_DATE,
                     COL_TIME
                     ]
            df_ohlc.set_index(index, inplace=True)

            # ---------- compose Table "Base" ----------
            tbl = []
            for rec in rsp.ttm_tbl_base:
                record = [rec.date,
                          rec.month,
                          rec.weekday_id,
                          rec.goto_id,
                          rec.is_goto,
                          rec.gap_type
                          ] + rec.data_list.tolist()
                tbl.append(record)

            columns = [COL_DATE,
                       COL_MONTH,
                       COL_WEEKDAY_ID,
                       COL_GOTO_ID,
                       COL_IS_GOTO,
                       COL_GAP_TYP] + time_range_list
            df_base = pd.DataFrame(tbl, columns=columns)

            index = [COL_DATE,
                     COL_MONTH,
                     COL_WEEKDAY_ID,
                     COL_GOTO_ID,
                     COL_IS_GOTO,
                     COL_GAP_TYP,
                     ]
            df_base.set_index(index, inplace=True)

            # ---------- compose Table "week_goto" ----------
            """
            level = [COL_WEEKDAY_ID,
                     COL_IS_GOTO,
                     COL_GAP_TYP,
                     ]
            df_week_goto = self.__make_statistics_dataframe(df_base, level)
            """
            """
            df_week_goto = ttmcom.convert_base2weekgoto(df_base)
            """

            # ---------- compose Table "month_goto" ----------
            """
            level = [COL_MONTH,
                     COL_GOTO_ID,
                     COL_GAP_TYP,
                     ]
            df_month_goto = self.__make_statistics_dataframe(df_base, level)
            """
            """
            df_month_goto = ttmcom.convert_base2monthgoto(df_base)
            """

            # ---------- compose Tree View ----------
            flg = df_base.index.get_level_values(COL_GAP_TYP) == GAP_TYP_CO
            """
            df = df_base[flg]
            for index, row in df.iterrows():
                items = [
                    QStandardItem(index[0]),  # date
                    QStandardItem(WEEKDAY_ID_DICT[index[2]]),
                    QStandardItem(GOTODAY_ID_DICT[index[3]]),
                ]
                self._qstd_itm_mdl.appendRow(items)
            """
            tbl = []
            for index, row in df_base[flg].iterrows():
                items = [
                    index[0],  # date
                    WEEKDAY_ID_DICT[index[2]],
                    GOTODAY_ID_DICT[index[3]],
                    0
                ]
                tbl.append(items)
            df = pd.DataFrame(tbl,
                              columns=self._TREEVIEW_HEADERS)
            df.set_index([self._TREEVIEW_ITEM_DATE], inplace=True)

            self._pdtreeview.set_dataframe(df)
            selmdl = self._pdtreeview.selectionModel()
            callback = self._on_selection_ttm_changed
            selmdl.selectionChanged.connect(callback)

            self._df_ohlc = df_ohlc
            self._df_base = df_base
            """
            self._df_week_goto = df_week_goto
            self._df_month_goto = df_month_goto
            """

    def _on_selection_ttm_changed(self, selected, _):

        if not selected.isEmpty():

            model_index = selected.at(0).indexes()[0]
            r = model_index.row()
            proxy = self._pdtreeview.proxy
            date_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)

            df_ohlc = self._df_ohlc
            flg = df_ohlc.index.get_level_values(COL_DATE) == date_str
            df = df_ohlc[flg].reset_index(level=COL_DATE, drop=True)
            fmt = FMT_DATE_YMD + FMT_TIME_HM
            df = df.rename(index=lambda t: dt.datetime.strptime(date_str + t, fmt))
            df.columns = self._CDL_COLUMNS

            max_y = df[CandlestickChartTtm.COL_NAME_HI].max()
            min_y = df[CandlestickChartTtm.COL_NAME_LO].min()
            dif = (max_y - min_y) * 0.05
            self._chart.set_max_y(max_y + dif)
            self._chart.set_min_y(min_y - dif)

            inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit

            self._chart.update(df, self._gran_id, decimal_digit)

    def _on_ttm_details_clicked(self):

        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit

        is_selected = self._pdtreeview.is_selected()
        dftv = self._pdtreeview.get_dataframe(is_selected=is_selected)

        date_list = sorted(dftv.index.to_list())
        df = self._df_base.loc[(date_list), :]

        self._widget_details.show()
        self._widget_details.init_resize()
        self._widget_details.set_data(df,
                                      # self._df_week_goto,
                                      # self._df_month_goto,
                                      self._gran_id,
                                      decimal_digit)

    """
    def __make_statistics_dataframe(self,
                                    df_base: pd.DataFrame,
                                    level: list
                                    ) -> pd.DataFrame:

        # ----- make DataFrame "Mean" -----
        df_mean = df_base.mean(level=level).sort_index()
        df_mean.reset_index(COL_GAP_TYP, inplace=True)

        df_mean[COL_DATA_TYP] = 0
        cond = df_mean[COL_GAP_TYP] == self._GAP_TYP_HO
        df_mean.loc[cond, COL_DATA_TYP] = self._DATA_TYP_HO_MEAN
        cond = df_mean[COL_GAP_TYP] == self._GAP_TYP_LO
        df_mean.loc[cond, COL_DATA_TYP] = self._DATA_TYP_LO_MEAN
        cond = df_mean[COL_GAP_TYP] == self._GAP_TYP_CO
        df_mean.loc[cond, COL_DATA_TYP] = self._DATA_TYP_CO_MEAN

        df_mean.drop(columns=COL_GAP_TYP, inplace=True)
        index = COL_DATA_TYP
        df_mean.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Std" -----
        df_std = df_base.std(level=level).sort_index()
        df_std.reset_index(COL_GAP_TYP, inplace=True)

        df_std[COL_DATA_TYP] = 0
        cond = df_std[COL_GAP_TYP] == self._GAP_TYP_HO
        df_std.loc[cond, COL_DATA_TYP] = self._DATA_TYP_HO_STD
        cond = df_std[COL_GAP_TYP] == self._GAP_TYP_LO
        df_std.loc[cond, COL_DATA_TYP] = self._DATA_TYP_LO_STD
        cond = df_std[COL_GAP_TYP] == self._GAP_TYP_CO
        df_std.loc[cond, COL_DATA_TYP] = self._DATA_TYP_CO_STD

        df_std.drop(columns=COL_GAP_TYP, inplace=True)
        index = COL_DATA_TYP
        df_std.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Cumulative Sum" -----
        cond = df_mean.index.get_level_values(COL_DATA_TYP) == self._DATA_TYP_CO_MEAN
        df_csum = df_mean[cond].rename(index={self._DATA_TYP_CO_MEAN: self._DATA_TYP_CO_CSUM},
                                       level=COL_DATA_TYP)
        df_csum = df_csum.cumsum(axis=1)

        # concat "df_mean" and "df_std" and "df_csum"
        return pd.concat([df_mean, df_std, df_csum]).sort_index()
        """
