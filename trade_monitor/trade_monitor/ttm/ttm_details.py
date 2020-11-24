import os
import pandas as pd
import datetime as dt
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QFile, QDate
from PySide2.QtUiTools import QUiLoader
from trade_monitor import utilities as utl
from trade_monitor.utilities import FMT_DATE_YMD, FMT_QT_DATE_YMD
from trade_monitor.ttm.ttm_common import COL_DATE


class DateRangeManager():

    def __init__(self):
        self._date_list = []
        self._upper_pos = 0
        self._lower_pos = 0

    def init_date_list(self, date_list):

        self._date_list = sorted(date_list)

        if date_list:
            self._upper_pos = len(date_list) - 1
        else:
            self._upper_pos = 0

        self._lower_pos = 0

    @property
    def length(self):
        return len(self._date_list)

    @property
    def count(self):
        if self._date_list:
            count = self._upper_pos - self._lower_pos + 1
        else:
            count = 0

        return count

    @property
    def list_all(self):
        return self._date_list.copy()

    @property
    def list_within_range(self):
        return self._date_list[self._lower_pos:self._upper_pos + 1]

    @property
    def upper_pos(self):
        return self._upper_pos

    @property
    def lower_pos(self):
        return self._lower_pos

    def slide(self, step):

        if self._date_list:
            if 0 < step:
                max_pos = len(self._date_list) - 1
                updated_pos = self._upper_pos + step
                if max_pos < updated_pos:
                    slide_cnt = max_pos - self._upper_pos
                else:
                    slide_cnt = step
            else:
                updated_pos = self._lower_pos + step
                if updated_pos < 0:
                    slide_cnt = 0 - self._lower_pos
                else:
                    slide_cnt = step

            self._upper_pos += slide_cnt
            self._lower_pos += slide_cnt

    def expand_range(self, step, priority_next=True):

        if self._date_list:
            if 0 < step:
                max_pos = len(self._date_list) - 1
                if priority_next:
                    updated_pos = self._upper_pos + step
                    if max_pos < updated_pos:
                        self._upper_pos = max_pos
                        slide_cnt = updated_pos - max_pos
                        lower_pos = self._lower_pos
                        lower_pos -= slide_cnt
                        if lower_pos < 0:
                            lower_pos = 0
                        self._lower_pos = lower_pos

                    else:
                        self._upper_pos += step
                else:
                    updated_pos = self._lower_pos - step
                    if updated_pos < 0:
                        self._lower_pos = 0
                        upper_pos = self._upper_pos
                        upper_pos -= updated_pos
                        if max_pos < upper_pos:
                            upper_pos = max_pos
                        self._upper_pos = upper_pos
                    else:
                        self._lower_pos -= step

    def shrink_range(self, step, priority_next=True):

        if self._date_list:
            if 0 < step:
                if self.count <= step:
                    slide_cnt = self.count - 1
                else:
                    slide_cnt = step

                if priority_next:
                    self._upper_pos -= slide_cnt
                else:
                    self._lower_pos += slide_cnt


class TtmDetails(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("TTM Details")

        callback = self._on_start_date_changed
        ui.dateEdit_Start.dateChanged.connect(callback)
        callback = self._on_end_date_changed
        ui.dateEdit_End.dateChanged.connect(callback)
        callback = self._on_spinBox_step_value_changed
        ui.spinBox_Step.valueChanged.connect(callback)

        self._logger = utl.get_logger()
        self._ui = ui
        self._df_week_goto = pd.DataFrame()
        self._df_month_goto = pd.DataFrame()

    def set_data(self, df_base, df_week_goto, df_month_goto):

        date_list = list(df_base.groupby(COL_DATE).groups.keys())

        #qd = QDate(dt_.year, dt_.month, dt_.day)

        date_max_str = date_list[-1]
        date_min_str = date_list[0]

        date_max = dt.datetime.strptime(date_max_str, FMT_DATE_YMD).date()
        date_min = dt.datetime.strptime(date_min_str, FMT_DATE_YMD).date()

        q_date_max = QDate(date_max.year, date_max.month, date_max.day)
        q_date_min = QDate(date_min.year, date_min.month, date_min.day)

        self._df_base_mst = df_base
        self._df_base = df_base
        self._df_week_goto = df_week_goto
        self._df_month_goto = df_month_goto

        self._ui.dateEdit_Start.setDate(q_date_min)
        self._ui.dateEdit_Start.setDateRange(q_date_min, q_date_max)
        self._ui.dateEdit_End.setDate(q_date_max)
        self._ui.dateEdit_End.setDateRange(q_date_min, q_date_max)
        self._ui.spinBox_Step.setMaximum(len(date_list))
        self._ui.spinBox_Step.setValue(len(date_list))

    def _on_start_date_changed(self, date):
        self._logger.debug("---------- on_start_date_changed ----------")
        self._logger.debug("{}" .format(date))

        self._ui.dateEdit_End.setMinimumDate(date)

        self._update_dataframe()

    def _on_end_date_changed(self, date):
        self._logger.debug("---------- on_end_date_changed ----------")
        self._logger.debug("{}" .format(date))

        self._ui.dateEdit_Start.setMaximumDate(date)

        self._update_dataframe()

    def _on_spinBox_step_value_changed(self, value):
        # TODO:
        pass

    def _update_dataframe(self):

        qdt = self._ui.dateEdit_Start.date()
        sdt_str = qdt.toString(FMT_QT_DATE_YMD)

        qdt = self._ui.dateEdit_End.date()
        sdt_end = qdt.toString(FMT_QT_DATE_YMD)

        self._logger.debug("sdt_str:[{}]" .format(sdt_str))
        self._logger.debug("sdt_end:[{}]" .format(sdt_end))

        mst_list = self._df_base_mst.index.get_level_values(level=COL_DATE)
        self._df_base = self._df_base_mst[(sdt_str <= mst_list) & (mst_list <= sdt_end)]

        date_list = list(self._df_base.groupby(COL_DATE).groups.keys())

        self._ui.spinBox_Step.setMaximum(len(date_list))
        self._ui.spinBox_Step.setValue(len(date_list))

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


if __name__ == '__main__':

    drm = DateRangeManager()

    date_list = ["2020/09/01",
                 "2020/09/02",
                 "2020/09/03",
                 "2020/09/04",
                 "2020/09/05",
                 "2020/09/06",
                 "2020/09/07",
                 ]

    drm.init_date_list(date_list)
    print("1-----------------------------")
    print("length = {}".format(drm.length))
    print("count = {}".format(drm.count))
    print("list_all = {}".format(drm.list_all))
    print("list_within_range = {}".format(drm.list_within_range))
