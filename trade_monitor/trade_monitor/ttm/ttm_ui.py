import pandas as pd
import datetime as dt

from PySide2.QtWidgets import QHeaderView
from PySide2.QtGui import QStandardItemModel, QStandardItem
from PySide2.QtCore import QItemSelectionModel

from trade_apl_msgs.srv import TtmMntSrv
from trade_apl_msgs.msg import TtmMsg
from trade_monitor import utilities as utl
from trade_monitor.utilities import INST_MSG_LIST
from trade_monitor.utilities import DT_FMT


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
        0: "None",
        1: "5",
        2: "10",
        3: "15",
        4: "20",
        5: "25",
        6: "L/D"
    }

    ID_NONE = 0         # Not Goto-Day
    ID_DAY_5 = 1        # Goto-Day(5 Day)
    ID_DAY_10 = 2       # Goto-Day(10 Day)
    ID_DAY_15 = 3       # Goto-Day(15 Day)
    ID_DAY_20 = 4       # Goto-Day(20 Day)
    ID_DAY_25 = 5       # Goto-Day(25 Day)
    ID_LAST_DAY = 6     # Goto-Day(Last Day)

    _TREEVIEW_HEADERS = [
        "Date",
        "Weekday",
        "Goto day",
        "Data type"
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

        # Create service client "ttm_monitor"
        srv_type = TtmMntSrv
        srv_name = "ttm_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = utl.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._qstd_itm_mdl = qstd_itm_mdl

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

            # Parameter data
            data = []
            for ttmmsg in rsp.ttmmsg_list:
                items = [
                    QStandardItem(ttmmsg.date),
                    QStandardItem(self._WEEKDAY_ID_DICT[ttmmsg.weekday_id]),
                    QStandardItem(self._GOTODAY_ID_DICT[ttmmsg.goto_id]),
                    QStandardItem(str(ttmmsg.data_type))
                ]
                self._qstd_itm_mdl.appendRow(items)

            header = self._ui.treeView_ttm.header()
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

    def _on_selection_ttm_changed(self, selected, _):

        qisr0 = selected.at(0)

        if qisr0 is not None:

            model_index = qisr0.indexes()[0]
            trg_date_str = self._qstd_itm_mdl.item(model_index.row()).text()
            utl.logger().debug("target_date: " + trg_date_str)
            trg_date = dt.datetime.strptime(trg_date_str, DT_FMT)

    def resize_chart_widget(self):
        pass

