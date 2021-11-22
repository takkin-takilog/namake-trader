import pandas as pd
from trade_monitor import utility as utl
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import VALID_GRAN_LIST
from trade_monitor.tech.sma01_ui import Sma01Ui
from trade_monitor import ros_common as ros_com


pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


class TechUi():

    def __init__(self, ui) -> None:

        self.logger = ros_com.get_logger()

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_gran)
        for obj in VALID_GRAN_LIST:
            ui.comboBox_tech_gran.addItem(obj.text)

        callback = self._on_inst_currentIndexChanged
        ui.comboBox_tech_inst.currentIndexChanged.connect(callback)

        callback = self._on_gran_currentIndexChanged
        ui.comboBox_tech_gran.currentIndexChanged.connect(callback)

        callback = self._on_tech_sma01_details_clicked
        ui.pushButton_tech_sma_mth01_details.clicked.connect(callback)

        # ---------- set field ----------
        self._ui = ui
        self._inst_param = VALID_INST_LIST[0]
        self._gran_param = VALID_GRAN_LIST[0]

        self._init_ros_service()

        self._sma_mth01_ui = Sma01Ui()

    def _on_inst_currentIndexChanged(self, index):
        self._inst_param = VALID_INST_LIST[index]
        self._init_ros_service()

    def _on_gran_currentIndexChanged(self, index):
        self._gran_param = VALID_GRAN_LIST[index]
        self._init_ros_service()

    def _init_ros_service(self):
        ns = self._inst_param.namespace + "/" + self._gran_param.namespace + "/"

        # Create service client "tech_sma_monitor"
        """
        srv_type = TechSmaMntSrv
        srv_name = "tech_sma_monitor"
        fullname = ns + srv_name
        self._srv_sma_cli = ros_com.get_node().create_client(srv_type, fullname)
        """

        # Create service client "tech_sma_method01_monitor"
        """
        srv_type = TechSmaMth01MntSrv
        srv_name = "tech_sma_method01_monitor"
        fullname = ns + srv_name
        self._srv_sma_mth01_cli = ros_com.get_node().create_client(srv_type, fullname)
        """

        # Create service client "tech_macd_gdc_monitor"
        """
        srv_type = TechMacdMntSrv
        srv_name = "tech_macd_monitor"
        fullname = ns + srv_name
        self._srv_macd_cli = ros_com.get_node().create_client(srv_type, fullname)
        """

        # Create service client "tech_chart_monitor"
        """
        srv_type = TechChartMntSrv
        srv_name = "tech_chart_monitor"
        fullname = ns + srv_name
        self._srv_chart_cli = ros_com.get_node().create_client(srv_type, fullname)
        """

    def _on_tech_sma01_details_clicked(self):
        self._sma_mth01_ui.set_data(self._inst_param, self._gran_param)
        self._sma_mth01_ui.show()
