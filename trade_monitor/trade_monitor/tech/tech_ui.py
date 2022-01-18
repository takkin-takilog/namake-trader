import pandas as pd
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.widget_base import StatusProgressBar
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.sma.sma01_ui import Sma01Ui
from trade_monitor.tech.bb.bb_ui import BollingerBandUi


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


class TechUi():

    def __init__(self, ui) -> None:
        self.logger = ros_com.get_logger()

        # ---------- set Instrument ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        # ---------- set Granularity ----------
        utl.remove_all_items_of_comboBox(ui.comboBox_tech_gran)
        for obj in VALID_GRAN_LIST:
            ui.comboBox_tech_gran.addItem(obj.text)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        # ---------- set SMA ----------
        callback = self._on_tech_sma01_details_clicked
        ui.pushButton_tech_sma_mth01_details.clicked.connect(callback)

        # ---------- set Status Bar ----------
        self._sts_bar = StatusProgressBar(ui.statusbar)

        # ---------- set field ----------
        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]

        self._sma_mth01_ui = Sma01Ui()
        self._bb_ui = BollingerBandUi(ui, self._inst_param, self._gran_param, self._sts_bar)

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._bb_ui.update_inst_param(self._inst_param)

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._bb_ui.update_gran_param(self._gran_param)

    def _on_tech_sma01_details_clicked(self):
        self._sma_mth01_ui.set_data(self._inst_param, self._gran_param)
        self._sma_mth01_ui.show()
