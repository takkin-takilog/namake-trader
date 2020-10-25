from trade_apl_msgs.srv import TtmMntSrv
from trade_apl_msgs.msg import TtmMsg
from trade_monitor import utilities as utl
from trade_monitor.utilities import INST_MSG_LIST


class TtmUi():

    def __init__(self, ui) -> None:

        callback = self._on_fetch_ttm_clicked
        # ui.pushButton_fetch_ttm.clicked.connect(callback)

        # Create service client "gapfill_monitor"
        srv_type = TtmMntSrv
        srv_name = "ttm_monitor"
        srv_cli_list = []
        for obj in INST_MSG_LIST:
            fullname = obj.namespace + "/" + srv_name
            srv_cli = utl.get_node().create_client(srv_type, fullname)
            srv_cli_list.append(srv_cli)

        self._ui = ui
        self._srv_cli_list = srv_cli_list

    def _on_fetch_gapfill_clicked(self):
        pass

