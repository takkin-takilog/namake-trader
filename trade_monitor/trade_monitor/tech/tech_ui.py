from enum import Enum
import pandas as pd
import datetime as dt
from PySide2.QtCore import Qt
from trade_apl_msgs.srv import TechMntSrv
from trade_monitor.widget_base import PandasTreeView
from trade_monitor import utility as utl
from trade_monitor.constant import GranParam, InstParam
from trade_monitor.constant import FMT_YMDHMS
from trade_monitor.tech.constant import VALID_INST_LIST
from trade_monitor.tech.constant import ColNameOhlc
from trade_monitor.tech.constant import ColNameSma
# from trade_monitor.tech.widget import CandlestickChartView
# from trade_monitor.widget_base import CandlestickChartViewBarCategoryAxis as CandlestickChartView
from trade_monitor.widget_base import CandlestickChartViewDateTimeAxis as CandlestickChartView
from trade_monitor import ros_common as ros_com
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran
from trade_monitor.widget_base import BaseCandlestickChartView

pd.set_option("display.max_columns", 1000)
pd.set_option("display.max_rows", 300)
pd.set_option("display.width", 200)
# pd.options.display.float_format = '{:.3f}'.format


class TechUi():

    def __init__(self, ui) -> None:

        utl.remove_all_items_of_comboBox(ui.comboBox_tech_inst)
        for obj in VALID_INST_LIST:
            ui.comboBox_tech_inst.addItem(obj.text)

        callback = self._on_fetch_tech_clicked
        ui.pushButton_tech_fetch.clicked.connect(callback)

        chartview = CandlestickChartView(ui.widget_ChartView_tech)

        # Create service client "tech_monitor"
        srv_type = TechMntSrv
        srv_name = "tech_monitor"
        fullname = "usdjpy_d" + "/" + srv_name
        srv_cli = ros_com.get_node().create_client(srv_type, fullname)

        self._chartview = chartview

        self._ui = ui
        self._srv_cli = srv_cli
        self.logger = ros_com.get_logger()

        self._gran_param = GranParam.get_member_by_msgid(Gran.GRAN_D)
        self._inst_param = InstParam.get_member_by_msgid(Inst.INST_USD_JPY)

    def _on_fetch_tech_clicked(self):
        inst_idx = self._ui.comboBox_tech_inst.currentIndex()
        inst_param = VALID_INST_LIST[inst_idx]

        # fetch Tech data
        req = TechMntSrv.Request()

        if not self._srv_cli.service_is_ready():
            self.logger.error("service server [{}] not to become ready"
                              .format(inst_param.text))
        else:
            rsp = ros_com.call_servive_sync(self._srv_cli, req)

            # ---------- compose Table "OHLC" ----------
            tbl = []
            for rec in rsp.tbl_det:
                record = [dt.datetime.strptime(rec.datetime, FMT_YMDHMS),
                          rec.mid_o,
                          rec.mid_h,
                          rec.mid_l,
                          rec.mid_c,
                          rec.sma_s,
                          rec.sma_m,
                          rec.sma_l
                          ]
                tbl.append(record)

            columns = [ColNameOhlc.DATETIME.value,
                       BaseCandlestickChartView.CandleLabel.OP.value,
                       BaseCandlestickChartView.CandleLabel.HI.value,
                       BaseCandlestickChartView.CandleLabel.LO.value,
                       BaseCandlestickChartView.CandleLabel.CL.value,
                       ColNameSma.SMA_S.value,
                       ColNameSma.SMA_M.value,
                       ColNameSma.SMA_L.value
                       ]
            df_det = pd.DataFrame(tbl, columns=columns)

            index = ColNameOhlc.DATETIME.value
            df_det.set_index(index, inplace=True)

            df_det2 = df_det[-50:]

            self._chartview.set_max_y(120)
            self._chartview.set_min_y(100)
            self._chartview.update(df_det2, self._inst_param)

            self._df_det = df_det

            self.logger.debug("----- check Head & Tail DataFrame -----")
            self.logger.debug("  << ---------- df_det ---------- >>")
            """
            self.logger.debug("\n  << --- Head --- >>\n{}".format(self._df_det[:6]))
            self.logger.debug("\n  << --- Tail --- >>\n{}".format(self._df_det[-5:]))
            """
            self.logger.debug("\n  << --- Head --- >>\n{}".format(df_det2))

    def _get_dataframe(self):

        is_selected = self._pdtreeview.is_selected()
        dftv = self._pdtreeview.get_dataframe(is_selected=is_selected)

        date_list = sorted(dftv.index.to_list())
        df = self._df_base.loc[(date_list), :]

        return df
