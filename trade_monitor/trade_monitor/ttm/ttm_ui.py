import pandas as pd
import datetime as dt

from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

from trade_apl_msgs.srv import TtmMntSrv
from trade_apl_msgs.msg import TtmTblOhlcRecMsg, TtmTblBaseRecMsg
from trade_monitor import utilities as utl
from trade_monitor.utilities import INST_MSG_LIST
from trade_monitor.utilities import GRAN_TIME_DICT
from trade_monitor.utilities import (FMT_DTTM_API,
                                     FMT_DATE_YMD,
                                     FMT_TIME_HM,
                                     FMT_TIME_HMS
                                     )
from trade_monitor.ttm.candlestick_chart import CandlestickChartTtm

pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
pd.options.display.float_format = '{:.3f}'.format


class TtmUi():

    _WEEKDAY_ID_DICT = {
        0: "Mon",
        1: "Tue",
        2: "Wed",
        3: "Thu",
        4: "Fri",
        5: "Sat",
        6: "Sun"
    }

    _GOTODAY_ID_DICT = {
        0: "-",
        1: "5",
        2: "10",
        3: "15",
        4: "20",
        5: "25",
        6: "L/D"
    }

    _GAP_TYP_DICT = {
        1: "High  - Open",
        2: "Low   - Open",
        3: "Close - Open"
    }

    _TREEVIEW_HEADERS = [
        "Date",
        "Weekday",
        "Goto day",
        "Data type"
    ]

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

        qstd_itm_mdl = QStandardItemModel()
        sel_mdl = QItemSelectionModel(qstd_itm_mdl)

        callback = self._on_selection_ttm_changed
        sel_mdl.selectionChanged.connect(callback)

        # set header
        qstd_itm_mdl.setHorizontalHeaderLabels(self._TREEVIEW_HEADERS)
        ui.treeView_ttm.setModel(qstd_itm_mdl)
        ui.treeView_ttm.setSelectionModel(sel_mdl)
        header = ui.treeView_ttm.header()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        chart = CandlestickChartTtm(ui.widget_ttm_chart1m)

        # Create service client "ttm_monitor"
        srv_type = TtmMntSrv
        srv_name = "ttm_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = utl.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._qstd_itm_mdl = qstd_itm_mdl
        self._chart = chart

        self._ui = ui
        self._srv_cli_list = srv_cli_list

    def _on_fetch_ttm_clicked(self):

        self._qstd_itm_mdl.clear()
        self._qstd_itm_mdl.setHorizontalHeaderLabels(self._TREEVIEW_HEADERS)
        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        inst_msg = INST_MSG_LIST[inst_idx]

        # fetch TTM data
        req = TtmMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            utl.logger().error("service server [{}] not to become ready"
                               .format(inst_msg.text))
        else:

            rsp = utl.call_servive_sync(srv_cli, req, timeout_sec=10.0)

            start_time_str = rsp.start_time
            end_time_str = rsp.end_time
            freq = GRAN_TIME_DICT[rsp.gran_id]
            time_range_list = pd.date_range(start_time_str,
                                            end_time_str,
                                            freq=freq
                                            ).strftime(FMT_TIME_HM).to_list()

            print("---------- time_range_list -----------")
            print(time_range_list)

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

            columns = [self._COL_DATE,
                       self._COL_TIME,
                       self._COL_O,
                       self._COL_H,
                       self._COL_L,
                       self._COL_C,
                       ]
            df_ohlc = pd.DataFrame(tbl, columns=columns)

            index = [self._COL_DATE,
                     self._COL_TIME
                     ]
            df_ohlc.set_index(index, inplace=True)

            print("----------------- df_ohlc -----------------")
            print(df_ohlc)

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

            columns = [self._COL_DATE,
                       self._COL_MONTH,
                       self._COL_WEEKDAY_ID,
                       self._COL_GOTO_ID,
                       self._COL_IS_GOTO,
                       self._COL_GAP_TYP] + time_range_list
            df_base = pd.DataFrame(tbl, columns=columns)

            index = [self._COL_DATE,
                     self._COL_MONTH,
                     self._COL_WEEKDAY_ID,
                     self._COL_GOTO_ID,
                     self._COL_IS_GOTO,
                     self._COL_GAP_TYP,
                     ]
            df_base.set_index(index, inplace=True)

            print("----------------- df_base -----------------")
            print(df_base)

            # ---------- compose Table "week_goto" ----------
            level = [self._COL_WEEKDAY_ID,
                     self._COL_IS_GOTO,
                     self._COL_GAP_TYP,
                     ]

            df_week_goto = self.__make_statistics_dataframe(df_base, level)
            print("----------------- df_week_goto -------------------")
            print(df_week_goto)

            # ---------- compose Table "month_goto" ----------
            level = [self._COL_MONTH,
                     self._COL_GOTO_ID,
                     self._COL_GAP_TYP,
                     ]
            df_month_goto = self.__make_statistics_dataframe(df_base, level)
            print("----------------- df_week_goto -------------------")
            print(df_month_goto)

            # ---------- compose Tree View ----------
            flg = df_base.index.get_level_values(self._COL_GAP_TYP) == self._GAP_TYP_CO
            df = df_base[flg]
            for index, row in df.iterrows():
                items = [
                    QStandardItem(index[0]),  # date
                    QStandardItem(self._WEEKDAY_ID_DICT[index[2]]),
                    QStandardItem(self._GOTODAY_ID_DICT[index[3]]),
                ]
                self._qstd_itm_mdl.appendRow(items)

            header = self._ui.treeView_ttm.header()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

            self.__df_ohlc = df_ohlc
            self.__df_base = df_base
            self.__df_week_goto = df_week_goto
            self.__df_month_goto = df_month_goto

    def _on_selection_ttm_changed(self, selected, _):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            date_str = self._qstd_itm_mdl.item(model_index.row()).text()
            utl.logger().debug("target_date: " + date_str)
            # trg_date = dt.datetime.strptime(date_str, FMT_DATE_YMD)

            df_ohlc = self.__df_ohlc
            flg = df_ohlc.index.get_level_values(self._COL_DATE) == date_str
            df = df_ohlc[flg].reset_index(level=self._COL_DATE, drop=True)
            fmt = FMT_DATE_YMD + FMT_TIME_HM
            df = df.rename(index=lambda t: dt.datetime.strptime(date_str + t, fmt))
            df.columns = self._CDL_COLUMNS

            max_y = df[CandlestickChartTtm.COL_NAME_HI].max()
            min_y = df[CandlestickChartTtm.COL_NAME_LO].min()
            self._chart.set_max_y(max_y)
            self._chart.set_min_y(min_y)

            inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
            decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit

            self._chart.update(df, decimal_digit)

    def resize_chart_widget(self):
        pass

    def __make_statistics_dataframe(self,
                                    df_base: pd.DataFrame,
                                    level: list
                                    ) -> pd.DataFrame:

        # ----- make DataFrame "Mean" -----
        df_mean = df_base.mean(level=level).sort_index()
        df_mean.reset_index(self._COL_GAP_TYP, inplace=True)

        df_mean[self._COL_DATA_TYP] = 0
        cond = df_mean[self._COL_GAP_TYP] == self._GAP_TYP_HO
        df_mean.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_HO_MEAN
        cond = df_mean[self._COL_GAP_TYP] == self._GAP_TYP_LO
        df_mean.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_LO_MEAN
        cond = df_mean[self._COL_GAP_TYP] == self._GAP_TYP_CO
        df_mean.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_CO_MEAN

        df_mean.drop(columns=self._COL_GAP_TYP, inplace=True)
        index = self._COL_DATA_TYP
        df_mean.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Std" -----
        df_std = df_base.std(level=level).sort_index()
        df_std.reset_index(self._COL_GAP_TYP, inplace=True)

        df_std[self._COL_DATA_TYP] = 0
        cond = df_std[self._COL_GAP_TYP] == self._GAP_TYP_HO
        df_std.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_HO_STD
        cond = df_std[self._COL_GAP_TYP] == self._GAP_TYP_LO
        df_std.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_LO_STD
        cond = df_std[self._COL_GAP_TYP] == self._GAP_TYP_CO
        df_std.loc[cond, self._COL_DATA_TYP] = self._DATA_TYP_CO_STD

        df_std.drop(columns=self._COL_GAP_TYP, inplace=True)
        index = self._COL_DATA_TYP
        df_std.set_index(index, append=True, inplace=True)

        # ----- make DataFrame "Cumulative Sum" -----
        cond = df_mean.index.get_level_values(self._COL_DATA_TYP) == self._DATA_TYP_CO_MEAN
        df_csum = df_mean[cond].rename(index={self._DATA_TYP_CO_MEAN: self._DATA_TYP_CO_CSUM},
                                       level=self._COL_DATA_TYP)
        df_csum = df_csum.cumsum(axis=1)

        # concat "df_mean" and "df_std" and "df_csum"
        return pd.concat([df_mean, df_std, df_csum]).sort_index()
