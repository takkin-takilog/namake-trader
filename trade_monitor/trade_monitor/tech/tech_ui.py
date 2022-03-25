import pandas as pd
from .. import utility as utl
from .. import ros_common as ros_com
from ..widget_base import StatusProgressBar
from ..parameter import VALID_INST_LIST, VALID_GRAN_LIST
from .sma.sma_ui import SimpleMovingAverageUi
from .bb.bb_ui import BollingerBandUi


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

        # ---------- set Status Bar ----------
        sts_bar = StatusProgressBar(ui.statusbar)

        # ---------- instantiate ----------
        inst_param = VALID_INST_LIST[0]
        gran_param = VALID_GRAN_LIST[0]
        self._sma_ui = SimpleMovingAverageUi(ui, inst_param, gran_param, sts_bar)
        self._bb_ui = BollingerBandUi(ui, inst_param, gran_param, sts_bar)

        # ---------- set field ----------
        self._ui = ui
        self._inst_param = inst_param
        self._gran_param = gran_param
        self._sts_bar = sts_bar

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._sma_ui.update_inst_param(self._inst_param)
        self._bb_ui.update_inst_param(self._inst_param)

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._sma_ui.update_gran_param(self._gran_param)
        self._bb_ui.update_gran_param(self._gran_param)
