import os
import pandas as pd
import datetime as dt
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QFile, QDate
from PySide2.QtUiTools import QUiLoader
from trade_monitor import utilities as utl
from trade_monitor.utilities import FMT_DATE_YMD
from trade_monitor.ttm.ttm_common import COL_DATE


class TtmDetails(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("TTM Details")

        self._logger = utl.get_logger()
        self._ui = ui
        self._df_week_goto = pd.DataFrame()
        self._df_month_goto = pd.DataFrame()

    def set_data(self, df_base, df_week_goto, df_month_goto):

        date_list = df_base.index.get_level_values(level=COL_DATE)

        #qd = QDate(dt_.year, dt_.month, dt_.day)

        date_max_str = date_list[-1]
        date_min_str = date_list[0]

        date_max = dt.datetime.strptime(date_max_str, FMT_DATE_YMD).date()
        date_min = dt.datetime.strptime(date_min_str, FMT_DATE_YMD).date()

        q_date_max = QDate(date_max.year, date_max.month, date_max.day)
        q_date_min = QDate(date_min.year, date_min.month, date_min.day)

        self._ui.dateEdit_Start.setDate(q_date_min)
        self._ui.dateEdit_Start.setDateRange(q_date_min, q_date_max)
        self._ui.dateEdit_End.setDate(q_date_max)
        self._ui.dateEdit_End.setDateRange(q_date_min, q_date_max)

        self._ui.spinBox_Step.setMaximum(len(date_list))
        self._ui.spinBox_Step.setValue(len(date_list))

        self._df_base = df_base
        self._df_week_goto = df_week_goto
        self._df_month_goto = df_month_goto

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
