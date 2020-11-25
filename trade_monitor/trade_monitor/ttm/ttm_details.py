import os
import pandas as pd
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QFile, QDate, Qt
from PySide2.QtUiTools import QUiLoader
from trade_monitor import utilities as utl
from trade_monitor.utilities import FMT_QT_DATE_YMD
from trade_monitor.utilities import DateRangeManager
from trade_monitor.ttm.ttm_common import COL_DATE


class TtmDetails(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("TTM Details")

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

        self._drm = DateRangeManager()
        self._logger = utl.get_logger()
        self._ui = ui
        self._df_week_goto = pd.DataFrame()
        self._df_month_goto = pd.DataFrame()

    def set_data(self, df_base, df_week_goto, df_month_goto):

        date_list = list(df_base.groupby(COL_DATE).groups.keys())

        self._drm.init_date_list(date_list)

        qdate_l = QDate.fromString(self._drm.lower_date, FMT_QT_DATE_YMD)
        qdate_u = QDate.fromString(self._drm.upper_date, FMT_QT_DATE_YMD)

        self._df_base_mst = df_base
        self._df_base = df_base
        self._df_week_goto = df_week_goto
        self._df_month_goto = df_month_goto

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

    def _on_pushButton_update_clicked(self, checked):
        self._logger.debug("=========================================")

    def _on_checkBox_autoupdate_stateChanged(self, state):
        self._logger.debug("---on_checkBox_autoupdate_stateChanged ----------")
        self._logger.debug("state:{}".format(state))

        if state == Qt.Checked:
            self._ui.pushButton_update.setEnabled(False)
        else:
            self._ui.pushButton_update.setEnabled(True)

    def _on_lower_date_changed(self, qdate):
        self._logger.debug("---[1]on_start_date_changed ----------")

        sdt_str = qdate.toString(FMT_QT_DATE_YMD)
        self._drm.set_lower(sdt_str)

        self._update_dataframe()

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
        self._logger.debug("---[2]on_end_date_changed ----------")

        sdt_str = qdate.toString(FMT_QT_DATE_YMD)
        self._drm.set_upper(sdt_str)

        self._update_dataframe()

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
        self._logger.debug("---[3]on_spinBox_step_value_changed ----------")

        slide_cnt = value - self._drm.count

        if 0 < slide_cnt:
            self._drm.expand_range(slide_cnt)
        else:
            self._drm.shrink_range(-slide_cnt)

        self._update_dataframe()

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
        self._logger.debug("---[4]on_ScrollBar_DateRange_value_changed ----------")

        step = value - self._drm.lower_pos
        self._drm.slide(step)

        self._update_dataframe()

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

    def _update_dataframe(self):
        self._logger.debug("---[99]update_dataframe ----------")

        sdt_str = self._drm.lower_date
        sdt_end = self._drm.upper_date

        self._logger.debug(" - sdt_str:{}".format(sdt_str))
        self._logger.debug(" - sdt_end:{}".format(sdt_end))

        mst_list = self._df_base_mst.index.get_level_values(level=COL_DATE)
        self._df_base = self._df_base_mst[(sdt_str <= mst_list) & (mst_list <= sdt_end)]

    def _load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "ttm_details.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
