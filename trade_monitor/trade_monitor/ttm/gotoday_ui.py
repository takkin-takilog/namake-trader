import os
import pandas as pd
from PySide2.QtWidgets import QMainWindow, QHeaderView
from PySide2.QtWidgets import QTableWidgetItem
from PySide2.QtCore import QFile, QDate, Qt
from PySide2.QtUiTools import QUiLoader
from PySide2.QtGui import QColor, QBrush
from trade_monitor.constant import FMT_QT_DATE_YMD
from trade_monitor.utility import DateRangeManager
from trade_monitor.ttm.constant import ColumnName
from trade_monitor.ttm.constant import (DATA_TYP_CO_CSUM,
                                        DATA_TYP_CO_MEAN,
                                        DATA_TYP_HO_MEAN,
                                        DATA_TYP_LO_MEAN)
from trade_monitor.ttm.widget import LineChartViewStats
from trade_monitor.ttm.widget import LineChartViewCumsum
from trade_monitor.ttm import constant as ttmcom
from trade_monitor import ros_common as ros_com


class TableItemConfig():

    def __init__(self,
                 text: str,
                 foreground_color=QColor(Qt.black),
                 background_color=QColor(Qt.white)
                 ):

        self._text = text
        self._fore_brush = QBrush(QColor(foreground_color))
        self._back_brush = QBrush(QColor(background_color))

    @property
    def text(self):
        return self._text

    @property
    def foreground(self):
        return self._fore_brush

    @property
    def background(self):
        return self._back_brush


class GotodayUi(QMainWindow):

    # Table Column Type
    _TBL_COL_GOTODAYTYP = 0
    _TBL_COL_CHARTTYP = 1
    _TBL_COL_CHARTVIEW = 2

    # define Goto-Day Type ID
    _GOTODAYTYP_ID_NTD = 0
    _GOTODAYTYP_ID_05D = 1
    _GOTODAYTYP_ID_10D = 2
    _GOTODAYTYP_ID_15D = 3
    _GOTODAYTYP_ID_20D = 4
    _GOTODAYTYP_ID_25D = 5
    _GOTODAYTYP_ID_LSD = 6

    # define Chart Type ID
    _CHARTTYP_ID_STATS = 0
    _CHARTTYP_ID_CUMSUM = 1

    _GOTODAYTYP_ID_DICT = {
        _GOTODAYTYP_ID_NTD: TableItemConfig("N/D", background_color="#f5dcdc"),
        _GOTODAYTYP_ID_05D: TableItemConfig("5Days", background_color="#d9ead3"),
        _GOTODAYTYP_ID_10D: TableItemConfig("10Days", background_color="#fce5cd"),
        _GOTODAYTYP_ID_15D: TableItemConfig("15Days", background_color="#cfe2f3"),
        _GOTODAYTYP_ID_20D: TableItemConfig("20Days", background_color="#fff2cc"),
        _GOTODAYTYP_ID_25D: TableItemConfig("25Days", background_color="#fff2cc"),
        _GOTODAYTYP_ID_LSD: TableItemConfig("L/D", background_color="#fff2cc")
    }

    _CHARTTYP_ID_DICT = {
        _CHARTTYP_ID_STATS: TableItemConfig("Stats"),
        _CHARTTYP_ID_CUMSUM: TableItemConfig("CumSum")
    }

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("TTM Gotoday")

        callback = self._on_pushButton_update_clicked
        ui.pushButton_update.clicked.connect(callback)
        callback = self._on_checkBox_autoupdate_stateChanged
        ui.checkBox_AutoUpdate.stateChanged.connect(callback)
        callback = self._on_lower_date_changed
        ui.dateEdit_lower.dateChanged.connect(callback)
        callback = self._on_upper_date_changed
        ui.dateEdit_upper.dateChanged.connect(callback)
        callback = self._on_spinBox_step_value_changed
        ui.spinBox_Step.valueChanged.connect(callback)
        callback = self._on_ScrollBar_DateRange_value_changed
        ui.ScrollBar_DateRange.valueChanged.connect(callback)

        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_ChartTyp_stat.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_ChartTyp_cumsum.stateChanged.connect(callback)

        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_Nd.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_5d.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_10d.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_15d.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_20d.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_25d.stateChanged.connect(callback)
        callback = self._on_checkBox_ShowItem_stateChanged
        ui.checkBox_GotoDayTyp_Ld.stateChanged.connect(callback)

        row_cnt = ui.tableWidget.rowCount()
        for i in reversed(range(row_cnt)):
            ui.tableWidget.removeRow(i)

        col_cnt = ui.tableWidget.columnCount()
        header = ui.tableWidget.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        for i in range(col_cnt - 1):
            header.setSectionResizeMode(i, QHeaderView.ResizeToContents)

        vHeaderView = ui.tableWidget.verticalHeader()
        # hHeaderView = ui.tableWidget.horizontalHeader()
        vHeaderView.setDefaultSectionSize(300)

        callback = self._on_vHeaderView_sectionResized
        vHeaderView.sectionResized.connect(callback)

        self._CHECKSTATE_GOTODAYTYP_DICT = {
            self._GOTODAYTYP_ID_NTD: ui.checkBox_GotoDayTyp_Nd.checkState,
            self._GOTODAYTYP_ID_05D: ui.checkBox_GotoDayTyp_5d.checkState,
            self._GOTODAYTYP_ID_10D: ui.checkBox_GotoDayTyp_10d.checkState,
            self._GOTODAYTYP_ID_15D: ui.checkBox_GotoDayTyp_15d.checkState,
            self._GOTODAYTYP_ID_20D: ui.checkBox_GotoDayTyp_20d.checkState,
            self._GOTODAYTYP_ID_25D: ui.checkBox_GotoDayTyp_25d.checkState,
            self._GOTODAYTYP_ID_LSD: ui.checkBox_GotoDayTyp_Ld.checkState
        }

        self._CHECKSTATE_CHARTTYP_DICT = {
            self._CHARTTYP_ID_STATS: ui.checkBox_ChartTyp_stat.checkState,
            self._CHARTTYP_ID_CUMSUM: ui.checkBox_ChartTyp_cumsum.checkState,
        }

        self._drm = DateRangeManager()
        self._logger = ros_com.get_logger()
        self._ui = ui
        self._df_base = pd.DataFrame()
        self._gran_id = None
        self._decimal_digit = None
        self._chart_idx_list = []
        self._is_require_reconstruct_table = True

    def set_data(self, df_base, gran_id, decimal_digit):

        date_list = list(df_base.groupby(ColumnName.DATE.value).groups.keys())

        self._drm.init_date_list(date_list)

        qdate_l = QDate.fromString(self._drm.lower_date, FMT_QT_DATE_YMD)
        qdate_u = QDate.fromString(self._drm.upper_date, FMT_QT_DATE_YMD)

        self._df_base = df_base
        self._gran_id = gran_id
        self._decimal_digit = decimal_digit

        wasBlocked1 = self._ui.dateEdit_lower.blockSignals(True)
        wasBlocked2 = self._ui.dateEdit_upper.blockSignals(True)
        wasBlocked3 = self._ui.spinBox_Step.blockSignals(True)
        wasBlocked4 = self._ui.ScrollBar_DateRange.blockSignals(True)

        self._ui.dateEdit_lower.setDateRange(qdate_l, qdate_u)
        self._ui.dateEdit_lower.setDate(qdate_l)
        self._ui.dateEdit_upper.setDateRange(qdate_l, qdate_u)
        self._ui.dateEdit_upper.setDate(qdate_u)
        self._ui.spinBox_Step.setMinimum(1)
        self._ui.spinBox_Step.setMaximum(self._drm.length)
        self._ui.spinBox_Step.setValue(self._drm.length)
        self._ui.ScrollBar_DateRange.setMaximum(self._drm.slidable_count)
        self._ui.ScrollBar_DateRange.setValue(self._drm.lower_pos)

        self._ui.dateEdit_lower.blockSignals(wasBlocked1)
        self._ui.dateEdit_upper.blockSignals(wasBlocked2)
        self._ui.spinBox_Step.blockSignals(wasBlocked3)
        self._ui.ScrollBar_DateRange.blockSignals(wasBlocked4)

    def _on_vHeaderView_sectionResized(self, index, oldSize, newSize):
        # self._logger.debug("--- on_tableWidget_rowResized ----------")

        vHeaderView = self._ui.tableWidget.verticalHeader()
        vHeaderView.setDefaultSectionSize(newSize)

    """
    def _on_hHeaderView_sectionResized(self, index, oldSize, newSize):
        self._logger.debug("--- on_tableWidget_columnResized ----------")
    """

    def _on_pushButton_update_clicked(self, checked):
        self._update_table()

    def _on_checkBox_autoupdate_stateChanged(self, state):
        # self._logger.debug("---on_checkBox_autoupdate_stateChanged ----------")
        # self._logger.debug("state:{}".format(state))

        if state == Qt.Checked:
            self._ui.pushButton_update.setEnabled(False)
            if self._is_require_reconstruct_table:
                self._update_table()
        else:
            self._ui.pushButton_update.setEnabled(True)

    def _on_lower_date_changed(self, qdate):
        # self._logger.debug("---[1]on_start_date_changed ----------")

        sdt_str = qdate.toString(FMT_QT_DATE_YMD)
        self._drm.set_lower(sdt_str)

        stat = self._ui.checkBox_AutoUpdate.checkState()
        if stat == Qt.Checked:
            self._update_table()

        wasBlocked1 = self._ui.dateEdit_upper.blockSignals(True)
        wasBlocked2 = self._ui.spinBox_Step.blockSignals(True)
        wasBlocked3 = self._ui.ScrollBar_DateRange.blockSignals(True)

        self._ui.dateEdit_upper.setMinimumDate(qdate)
        self._ui.spinBox_Step.setValue(self._drm.count)
        self._ui.ScrollBar_DateRange.setMaximum(self._drm.slidable_count)
        self._ui.ScrollBar_DateRange.setValue(self._drm.lower_pos)

        self._ui.dateEdit_upper.blockSignals(wasBlocked1)
        self._ui.spinBox_Step.blockSignals(wasBlocked2)
        self._ui.ScrollBar_DateRange.blockSignals(wasBlocked3)

    def _on_upper_date_changed(self, qdate):
        # self._logger.debug("---[2]on_end_date_changed ----------")

        sdt_str = qdate.toString(FMT_QT_DATE_YMD)
        self._drm.set_upper(sdt_str)

        stat = self._ui.checkBox_AutoUpdate.checkState()
        if stat == Qt.Checked:
            self._update_table()

        wasBlocked1 = self._ui.dateEdit_lower.blockSignals(True)
        wasBlocked2 = self._ui.spinBox_Step.blockSignals(True)
        wasBlocked3 = self._ui.ScrollBar_DateRange.blockSignals(True)

        self._ui.dateEdit_lower.setMaximumDate(qdate)
        self._ui.spinBox_Step.setValue(self._drm.count)
        self._ui.ScrollBar_DateRange.setMaximum(self._drm.slidable_count)
        self._ui.ScrollBar_DateRange.setValue(self._drm.lower_pos)

        self._ui.dateEdit_lower.blockSignals(wasBlocked1)
        self._ui.spinBox_Step.blockSignals(wasBlocked2)
        self._ui.ScrollBar_DateRange.blockSignals(wasBlocked3)

    def _on_spinBox_step_value_changed(self, value):
        # self._logger.debug("---[3]on_spinBox_step_value_changed ----------")

        slide_cnt = value - self._drm.count

        if 0 < slide_cnt:
            self._drm.expand_range(slide_cnt)
        else:
            self._drm.shrink_range(-slide_cnt)

        stat = self._ui.checkBox_AutoUpdate.checkState()
        if stat == Qt.Checked:
            self._update_table()

        qdate_l = QDate.fromString(self._drm.lower_date, FMT_QT_DATE_YMD)
        qdate_u = QDate.fromString(self._drm.upper_date, FMT_QT_DATE_YMD)

        wasBlocked1 = self._ui.dateEdit_lower.blockSignals(True)
        wasBlocked2 = self._ui.dateEdit_upper.blockSignals(True)
        wasBlocked3 = self._ui.ScrollBar_DateRange.blockSignals(True)

        self._ui.dateEdit_lower.setMaximumDate(qdate_u)
        self._ui.dateEdit_lower.setDate(qdate_l)
        self._ui.dateEdit_upper.setMinimumDate(qdate_l)
        self._ui.dateEdit_upper.setDate(qdate_u)
        self._ui.ScrollBar_DateRange.setMaximum(self._drm.slidable_count)
        self._ui.ScrollBar_DateRange.setValue(self._drm.lower_pos)

        self._ui.dateEdit_lower.blockSignals(wasBlocked1)
        self._ui.dateEdit_upper.blockSignals(wasBlocked2)
        self._ui.ScrollBar_DateRange.blockSignals(wasBlocked3)

    def _on_ScrollBar_DateRange_value_changed(self, value):
        # self._logger.debug("---[4]on_ScrollBar_DateRange_value_changed ----------")

        step = value - self._drm.lower_pos
        self._drm.slide(step)

        stat = self._ui.checkBox_AutoUpdate.checkState()
        if stat == Qt.Checked:
            self._update_table()

        qdate_l = QDate.fromString(self._drm.lower_date, FMT_QT_DATE_YMD)
        qdate_u = QDate.fromString(self._drm.upper_date, FMT_QT_DATE_YMD)

        wasBlocked1 = self._ui.dateEdit_lower.blockSignals(True)
        wasBlocked2 = self._ui.dateEdit_upper.blockSignals(True)

        self._ui.dateEdit_lower.setMaximumDate(qdate_u)
        self._ui.dateEdit_lower.setDate(qdate_l)
        self._ui.dateEdit_upper.setMinimumDate(qdate_l)
        self._ui.dateEdit_upper.setDate(qdate_u)

        self._ui.dateEdit_lower.blockSignals(wasBlocked1)
        self._ui.dateEdit_upper.blockSignals(wasBlocked2)

    def _on_checkBox_ShowItem_stateChanged(self, state):
        # self._logger.debug("--- on_checkBox_ShowItem_stateChanged ----------")

        self._is_require_reconstruct_table = True

        stat = self._ui.checkBox_AutoUpdate.checkState()
        if stat == Qt.Checked:
            self._update_table()

    def _update_table(self):

        df = self._get_latest_dataframe()

        if self._is_require_reconstruct_table:
            self._reconstruct_table()

        self._update_chart(df)

    def _get_latest_dataframe(self):
        # self._logger.debug("---[99]update_dataframe ----------")

        sdt_str = self._drm.lower_date
        sdt_end = self._drm.upper_date
        mst_list = self._df_base.index.get_level_values(level=ColumnName.DATE.value)
        df_base = self._df_base[(sdt_str <= mst_list) & (mst_list <= sdt_end)]
        df = ttmcom.convert_base2monthgoto(df_base)

        return df

    def _reconstruct_table(self):

        row_cnt = self._ui.tableWidget.rowCount()
        for i in reversed(range(row_cnt)):
            self._ui.tableWidget.removeRow(i)

        chart_idx_list = []
        for gotodaytyp_id in self._GOTODAYTYP_ID_DICT.keys():
            checkState = self._CHECKSTATE_GOTODAYTYP_DICT[gotodaytyp_id]
            wd_stat = checkState()
            if wd_stat != Qt.Checked:
                continue

            for charttyp_id in self._CHARTTYP_ID_DICT.keys():
                checkState = self._CHECKSTATE_CHARTTYP_DICT[charttyp_id]
                gd_stat = checkState()
                if gd_stat != Qt.Checked:
                    continue

                chart_idx_list.append([gotodaytyp_id, charttyp_id])

        for i, chart_idx in enumerate(chart_idx_list):
            gotodaytyp_id = chart_idx[0]
            charttyp_id = chart_idx[1]

            self._ui.tableWidget.insertRow(i)

            # set Table Item "Goto-Day Type"
            tbl_itm_cnf = self._GOTODAYTYP_ID_DICT[gotodaytyp_id]
            back_color = tbl_itm_cnf.background
            item_wd = QTableWidgetItem(tbl_itm_cnf.text)
            item_wd.setTextAlignment(Qt.AlignCenter)
            item_wd.setForeground(tbl_itm_cnf.foreground)
            item_wd.setBackground(back_color)
            self._ui.tableWidget.setItem(i, self._TBL_COL_GOTODAYTYP, item_wd)

            # set Table Item "Chart Type"
            tbl_itm_cnf = self._CHARTTYP_ID_DICT[charttyp_id]
            item_ct = QTableWidgetItem(tbl_itm_cnf.text)
            item_ct.setTextAlignment(Qt.AlignCenter)
            item_ct.setForeground(tbl_itm_cnf.foreground)
            item_ct.setBackground(back_color)
            self._ui.tableWidget.setItem(i, self._TBL_COL_CHARTTYP, item_ct)

            # set Table Item "Chart View"
            if charttyp_id == self._CHARTTYP_ID_STATS:
                chartview = LineChartViewStats()
            else:
                chartview = LineChartViewCumsum()
            chartview.chart().setBackgroundBrush(back_color)
            self._ui.tableWidget.setCellWidget(i, self._TBL_COL_CHARTVIEW, chartview)

        self._chart_idx_list = chart_idx_list
        self._is_require_reconstruct_table = False

    def _update_chart(self, df):

        mst_list = df.index.get_level_values(level=ColumnName.DATA_TYP.value)
        df_stats = df[mst_list < DATA_TYP_CO_CSUM]

        cond = ((mst_list == DATA_TYP_CO_MEAN) |
                (mst_list == DATA_TYP_HO_MEAN) |
                (mst_list == DATA_TYP_LO_MEAN))
        df_stats_max = df[cond]

        max_ = df_stats_max.max().max()
        min_ = df_stats_max.min().min()
        stats_max = max(abs(max_), abs(min_))

        df_csum = df[mst_list == DATA_TYP_CO_CSUM]
        max_ = df_csum.max().max()
        min_ = df_csum.min().min()
        csum_max = max(abs(max_), abs(min_))

        for i, chart_idx in enumerate(self._chart_idx_list):
            gotodaytyp_id = chart_idx[0]
            charttyp_id = chart_idx[1]

            if charttyp_id == self._CHARTTYP_ID_STATS:
                df_trg = df_stats
                max_y = stats_max
            else:
                df_trg = df_csum
                max_y = csum_max

            chart = self._ui.tableWidget.cellWidget(i, self._TBL_COL_CHARTVIEW)
            chart.set_max_y(max_y)
            chart.set_min_y(-max_y)
            idxloc = (gotodaytyp_id)
            if idxloc in df_trg.index:
                chart.update(df_trg.loc[idxloc], self._gran_id, self._decimal_digit)
            else:
                chart.clear()

    def _load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gotoday.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
