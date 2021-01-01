from enum import Enum
import pandas as pd
import datetime as dt
from PySide2.QtCore import Qt
from trade_apl_msgs.srv import TtmMntSrv
from trade_monitor.widget_base import PandasTreeView
from trade_monitor import utility as utl
from trade_monitor.ttm.constant import VALID_INST_LIST
from trade_monitor.constant import GranParam
from trade_monitor.constant import (FMT_DTTM_API,
                                    FMT_DATE_YMD,
                                    FMT_TIME_HM,
                                    FMT_TIME_HMS
                                    )
from trade_monitor.ttm.widget import CandlestickChartView as ChartView
from trade_monitor.ttm.weekday_ui import WeekdayUi
from trade_monitor.ttm.gotoday_ui import GotodayUi
from trade_monitor.ttm.constant import ColumnName, GapType
from trade_monitor import ros_common as ros_com

pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


class _WeekdayId(Enum):
    """
    Weekday ID.
    """
    MON = (0, "Mon")
    TUE = (1, "Tue")
    WED = (2, "Wed")
    THU = (3, "Thu")
    FRI = (4, "Fri")
    SAT = (5, "Sat")
    SUN = (6, "Sun")

    def __init__(self, id_: int, text: str):
        self.id = id_
        self.text = text

    @classmethod
    def get_member_by_id(cls, id_: int):
        for m in cls:
            if id_ == m.id:
                return m
        return None


class _GotodayId(Enum):
    """
    Gotoday ID.
    """
    NON = (0, "-")
    D05 = (1, "5")
    D10 = (2, "10")
    D15 = (3, "15")
    D20 = (4, "20")
    D25 = (5, "25")
    LSD = (6, "L/D")

    def __init__(self, id_: int, text: str):
        self.id = id_
        self.text = text

    @classmethod
    def get_member_by_id(cls, id_: int):
        for m in cls:
            if id_ == m.id:
                return m
        return None

    @property
    def is_goto(self) -> bool:
        """
        Least significant bit (Resolution).
        return type is "float".
        """
        return False if self is _GotodayId.NON else True


class _TreeViewColumn(Enum):
    """
    Tree view column.
    """
    DATE = "Date"
    WEEKDAY = "Weekday"
    GOTODAY = "Gotoday"
    IS_GOTO = "Is-Gotoday"
    CLOP0930 = "CL-OP:9:30"
    CLOP0950 = "CL-OP:9:50"
    CLOP0955 = "CL-OP:9:55"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class TtmUi():

    def __init__(self, ui) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_ttm_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_ttm_inst.addItem(obj.text)

        callback = self._on_fetch_ttm_clicked
        ui.pushButton_ttm_fetch.clicked.connect(callback)

        callback = self._on_ttm_weekday_clicked
        ui.pushButton_ttm_weekday.clicked.connect(callback)

        callback = self._on_ttm_gotoday_clicked
        ui.pushButton_ttm_gotoday.clicked.connect(callback)

        # set Tree View
        pdtreeview = PandasTreeView(ui.widget_TreeView_ttm)

        selmdl = pdtreeview.selectionModel()
        callback = self._on_selection_ttm_changed
        selmdl.selectionChanged.connect(callback)

        header = pdtreeview.header()
        callback = self._on_view_header_sectionClicked
        header.sectionClicked.connect(callback)

        chartview = ChartView(ui.widget_ChartView_ttm)

        # Create service client "ttm_monitor"
        srv_type = TtmMntSrv
        srv_name = "ttm_monitor"
        srv_cli_list = []
        for obj in VALID_INST_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = ros_com.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._weekday_ui = WeekdayUi()
        self._gotoday_ui = GotodayUi()

        # self._qstd_itm_mdl = qstd_itm_mdl
        self._chartview = chartview

        self._ui = ui
        self._pdtreeview = pdtreeview
        self._gran_param = GranParam.D
        self._srv_cli_list = srv_cli_list
        self._logger = ros_com.get_logger()

    def _on_view_header_sectionClicked(self, logical_index):
        self._pdtreeview.show_header_menu(logical_index)

    def _on_fetch_ttm_clicked(self):
        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        inst_param = VALID_INST_LIST[inst_idx]

        # fetch TTM data
        req = TtmMntSrv.Request()

        srv_cli = self._srv_cli_list[inst_idx]
        if not srv_cli.service_is_ready():
            self._logger.error("service server [{}] not to become ready"
                               .format(inst_param.text))
        else:
            rsp = ros_com.call_servive_sync(srv_cli, req)
            self._gran_param = GranParam.get_member_by_msgid(rsp.gran_id)

            start_time_str = rsp.start_time
            end_time_str = rsp.end_time
            gran_param = GranParam.get_member_by_msgid(rsp.gran_id)
            time_range_list = pd.date_range(start_time_str,
                                            end_time_str,
                                            freq=gran_param.freq
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

            columns = [ColumnName.DATE.value,
                       ColumnName.TIME.value,
                       ColumnName.CDL_O.value,
                       ColumnName.CDL_H.value,
                       ColumnName.CDL_L.value,
                       ColumnName.CDL_C.value,
                       ]
            df_ohlc = pd.DataFrame(tbl, columns=columns)

            index = [ColumnName.DATE.value,
                     ColumnName.TIME.value
                     ]
            df_ohlc.set_index(index, inplace=True)

            # ---------- compose Table "Base" ----------
            tbl = []
            for rec in rsp.ttm_tbl_base:
                record = [rec.date,
                          # rec.month,
                          rec.weekday_id,
                          rec.gotoday_id,
                          rec.is_goto,
                          rec.gap_type
                          ] + rec.data_list.tolist()
                tbl.append(record)

            columns = [ColumnName.DATE.value,
                       # ColumnName.MONTH.value,
                       ColumnName.WEEKDAY_ID.value,
                       ColumnName.GOTODAY_ID.value,
                       ColumnName.IS_GOTO.value,
                       ColumnName.GAP_TYP.value
                       ] + time_range_list
            df_base = pd.DataFrame(tbl, columns=columns)

            index = [ColumnName.DATE.value,
                     # ColumnName.MONTH.value,
                     ColumnName.WEEKDAY_ID.value,
                     ColumnName.GOTODAY_ID.value,
                     ColumnName.IS_GOTO.value,
                     ColumnName.GAP_TYP.value,
                     ]
            df_base.set_index(index, inplace=True)

            # ---------- compose Tree View ----------
            inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
            inst_param = VALID_INST_LIST[inst_idx]
            is_goto_dict = {True: "Yes", False: "No"}
            mltidx_name = df_base.index.names
            date_pos = mltidx_name.index(ColumnName.DATE.value)
            weekdayid_pos = mltidx_name.index(ColumnName.WEEKDAY_ID.value)
            gotodayid_pos = mltidx_name.index(ColumnName.GOTODAY_ID.value)
            flg = df_base.index.get_level_values(ColumnName.GAP_TYP.value) == GapType.CO.value
            tbl = []
            for mltidx, row in df_base[flg].iterrows():
                date = mltidx[date_pos]
                weekdayid = mltidx[weekdayid_pos]
                gotodayid = mltidx[gotodayid_pos]
                wdmem = _WeekdayId.get_member_by_id(weekdayid)
                gdmem = _GotodayId.get_member_by_id(gotodayid)
                items = [
                    date,
                    wdmem.text,
                    gdmem.text,
                    is_goto_dict[gdmem.is_goto],
                    utl.roundf(row["09:30"], digit=inst_param.digit),
                    utl.roundf(row["09:50"], digit=inst_param.digit),
                    utl.roundf(row["09:55"], digit=inst_param.digit)
                ]
                tbl.append(items)
            df = pd.DataFrame(tbl,
                              columns=_TreeViewColumn.to_list())
            label_date = _TreeViewColumn.DATE.value
            df.set_index([label_date], inplace=True)

            self._pdtreeview.set_dataframe(df)
            selmdl = self._pdtreeview.selectionModel()
            callback = self._on_selection_ttm_changed
            selmdl.selectionChanged.connect(callback)

            self._df_ohlc = df_ohlc
            self._df_base = df_base

            self._ui.pushButton_ttm_weekday.setEnabled(True)
            self._ui.pushButton_ttm_gotoday.setEnabled(True)

    def _on_selection_ttm_changed(self, selected, _):

        if not selected.isEmpty():

            model_index = selected.at(0).indexes()[0]
            r = model_index.row()
            proxy = self._pdtreeview.proxy
            date_str = proxy.index(r, 0, model_index).data(role=Qt.UserRole)

            df_ohlc = self._df_ohlc
            flg = df_ohlc.index.get_level_values(ColumnName.DATE.value) == date_str
            df = df_ohlc[flg].reset_index(level=ColumnName.DATE.value, drop=True)
            fmt = FMT_DATE_YMD + FMT_TIME_HM
            df = df.rename(index=lambda t: dt.datetime.strptime(date_str + t, fmt))
            # df.columns = self._CDL_COLUMNS
            df.columns = self._chartview.get_candle_labels_list()

            max_y = df[ChartView.CandleLabel.HI.value].max()
            min_y = df[ChartView.CandleLabel.LO.value].min()

            dif = (max_y - min_y) * 0.05
            self._chartview.set_max_y(max_y + dif)
            self._chartview.set_min_y(min_y - dif)

            inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
            inst_param = VALID_INST_LIST[inst_idx]

            self._chartview.update(df, self._gran_param, inst_param)
            self._ui.widget_ChartView_ttm.setEnabled(True)

    def _on_ttm_weekday_clicked(self):

        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        inst_param = VALID_INST_LIST[inst_idx]

        df = self._get_dataframe()

        self._weekday_ui.show()
        self._weekday_ui.set_data(df, self._gran_param, inst_param)

    def _on_ttm_gotoday_clicked(self):

        inst_idx = self._ui.comboBox_ttm_inst.currentIndex()
        inst_param = VALID_INST_LIST[inst_idx]

        df = self._get_dataframe()

        self._gotoday_ui.show()
        self._gotoday_ui.set_data(df, self._gran_param, inst_param)

    def _get_dataframe(self):

        is_selected = self._pdtreeview.is_selected()
        dftv = self._pdtreeview.get_dataframe(is_selected=is_selected)

        date_list = sorted(dftv.index.to_list())
        df = self._df_base.loc[(date_list), :]

        return df
