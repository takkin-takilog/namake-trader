from typing import TypeVar
import datetime as dt
import pandas as pd
import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument as InstApi
from api_msgs.msg import Granularity as GranApi
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.srv import HistoricalCandlesSrv
from trade_manager_msgs.msg import Candle
from trade_manager_msgs.msg import CandleMnt
from trade_manager_msgs.msg import Instrument as InstTm
from trade_manager_msgs.msg import Granularity as GranTm

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class CandlesData():

    DT_LSB_DICT = {
        GranTm.GRAN_M1: dt.timedelta(minutes=1),    # 1 minute
        GranTm.GRAN_M2: dt.timedelta(minutes=2),    # 2 minutes
        GranTm.GRAN_M3: dt.timedelta(minutes=3),    # 3 minutes
        GranTm.GRAN_M4: dt.timedelta(minutes=4),    # 4 minutes
        GranTm.GRAN_M5: dt.timedelta(minutes=5),    # 5 minutes
        GranTm.GRAN_M10: dt.timedelta(minutes=10),  # 10 minutes
        GranTm.GRAN_M15: dt.timedelta(minutes=15),  # 15 minutes
        GranTm.GRAN_M30: dt.timedelta(minutes=30),  # 30 minutes
        GranTm.GRAN_H1: dt.timedelta(hours=1),      # 1 hour
        GranTm.GRAN_H2: dt.timedelta(hours=2),      # 2 hours
        GranTm.GRAN_H3: dt.timedelta(hours=3),      # 3 hours
        GranTm.GRAN_H4: dt.timedelta(hours=4),      # 4 hours
        GranTm.GRAN_H6: dt.timedelta(hours=6),      # 6 hours
        GranTm.GRAN_H8: dt.timedelta(hours=8),      # 8 hours
        GranTm.GRAN_H12: dt.timedelta(hours=12),    # 12 hours
        GranTm.GRAN_D: dt.timedelta(days=1),        # 1 Day
        GranTm.GRAN_W: dt.timedelta(weeks=1),       # 1 Week
    }

    COL_NAME_TIME = "time"
    COL_NAME_ASK_OP = "open(Ask)"
    COL_NAME_ASK_HI = "high(Ask)"
    COL_NAME_ASK_LO = "low(Ask)"
    COL_NAME_ASK_CL = "close(Ask)"
    COL_NAME_BID_OP = "open(Bid)"
    COL_NAME_BID_HI = "high(Bid)"
    COL_NAME_BID_LO = "low(Bid)"
    COL_NAME_BID_CL = "close(Bid)"
    COL_NAME_COMP = "complete"
    COL_NAME_MID_OP = "open(Mid)"
    COL_NAME_MID_HI = "high(Mid)"
    COL_NAME_MID_LO = "low(Mid)"
    COL_NAME_MID_CL = "close(Mid)"
    COL_NAME_HSC_OP = "half_spread_cost(open)"
    COL_NAME_HSC_HI = "half_spread_cost(high)"
    COL_NAME_HSC_LO = "half_spread_cost(low)"
    COL_NAME_HSC_CL = "half_spread_cost(close)"

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"
    TIMEOUT_SEC = 3.0
    MARGIN_SEC = dt.timedelta(seconds=2)

    def __init__(self,
                 node: Node,
                 srv_cli: Client,
                 inst_id: InstTm,
                 gran_id: GranTm,
                 data_length: int
                 ) -> None:

        minunit = self.DT_LSB_DICT[gran_id]
        self.__logger = node.get_logger()

        self.__logger.debug("----- Create CandlesData:Start -----")
        self.__logger.debug("- inst_id:[%d], gran_id:[%d]" % (inst_id, gran_id))

        dt_now = dt.datetime.now()
        dt_from = dt_now - minunit * data_length
        dt_to = dt_now

        interval = minunit
        dt_1h = dt.timedelta(hours=1)
        if dt_1h < interval:
            interval = dt_1h

        self.__srv_cli = srv_cli
        self.__inst_id = inst_id
        self.__gran_id = gran_id

        cnt = 0
        is_comp = False
        while cnt < 3:
            future = self.__request_async(dt_from, dt_to)
            rclpy.spin_until_future_complete(node, future)

            if future.done() and future.result() is not None:
                is_comp = True
                break
            cnt += 1

        if not is_comp:
            self.__logger.error("!!!!!!!!!! ROS Service Error !!!!!!!!!!")
            self.__logger.error("Service Name:[candles]")
        else:
            df = self.__get_df_from_future(future)

            df_comp = df[(df[self.COL_NAME_COMP])]
            df_prov = df[~(df[self.COL_NAME_COMP])]

            self.__interval = interval
            self.__next_update_time = self.__get_next_update_time(gran_id, dt_now)
            self.__df_comp = df_comp
            self.__df_prov = df_prov
            self.__future = None
            self.__timeout_end = 0.0

            self.__logger.debug("- last_update_time:[%s]" % (self.__next_update_time))
            self.__logger.debug("----- Create CandlesData:End -----")

    def __get_next_update_time(self,
                               gran_id: int,
                               dtin: dt.datetime
                               ) -> dt.datetime:

        if gran_id == GranTm.GRAN_M1:
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, dtin.minute)
        elif gran_id == GranTm.GRAN_M2:
            tmp = dtin.minute - dtin.minute % 2
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M3:
            tmp = dtin.minute - dtin.minute % 3
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M4:
            tmp = dtin.minute - dtin.minute % 4
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M5:
            tmp = dtin.minute - dtin.minute % 5
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M10:
            tmp = dtin.minute - dtin.minute % 10
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M15:
            tmp = dtin.minute - dtin.minute % 15
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranTm.GRAN_M30:
            tmp = dtin.minute - dtin.minute % 30
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif ((gran_id == GranTm.GRAN_H1) or (gran_id == GranTm.GRAN_H2) or
              (gran_id == GranTm.GRAN_H3) or (gran_id == GranTm.GRAN_H4) or
              (gran_id == GranTm.GRAN_H6) or (gran_id == GranTm.GRAN_H8) or
              (gran_id == GranTm.GRAN_H12) or (gran_id == GranTm.GRAN_D) or
                (gran_id == GranTm.GRAN_W)):
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour)
        else:
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour)

        return dtout

    @property
    def dataframe(self) -> pd.DataFrame:
        return self.__df_comp

    def update_not_complete_data(self) -> None:

        if self.__future is None:

            dt_now = dt.datetime.now()
            """
            self.__logger.debug("[%s]update_not_complete_data[inst:%d][gran:%d]" % (
                dt_now, self.__inst_id, self.__gran_id))
            """
            target_time = dt_now - self.__next_update_time - self.MARGIN_SEC
            if self.__interval < target_time:

                # dt_from = self.__next_update_time
                dt_from = self.__df_comp.index[-1] + self.__interval
                dt_to = dt_now

                self.__future = self.__request_async(dt_from, dt_to)

                self.__timeout_end = time.monotonic() + self.TIMEOUT_SEC

        else:
            if self.__future.done() and self.__future.result() is not None:

                df = self.__get_df_from_future(self.__future)
                self.__update_df(df)
                self.__next_update_time += self.__interval
                self.__future = None

                self.__logger.debug("<Update> inst_id:[%d], gran_id:[%d] last_update_time:[%s]"
                                    % (self.__inst_id, self.__gran_id, self.__next_update_time))
            else:
                if time.monotonic() >= self.__timeout_end:
                    self.__future = None

    def __request_async(self,
                        dt_from: dt.datetime,
                        dt_to: dt.datetime
                        ) -> Future:

        req = CandlesSrv.Request()

        req.inst_msg.inst_id = self.__inst_id
        req.gran_msg.gran_id = self.__gran_id

        req.dt_from = dt_from.strftime(self.DT_FMT)
        req.dt_to = dt_to.strftime(self.DT_FMT)

        future = self.__srv_cli.call_async(req)

        return future

    def __get_df_from_future(self, future: Future) -> pd.DataFrame:

        df = pd.DataFrame()
        rsp = future.result()
        if rsp.result is True:

            data = []
            for cndl_msg in rsp.cndl_msg_list:
                dt_ = dt.datetime.strptime(cndl_msg.time, self.DT_FMT)
                hsc_o = (cndl_msg.ask_o - cndl_msg.bid_o) / 2
                hsc_h = (cndl_msg.ask_h - cndl_msg.bid_h) / 2
                hsc_l = (cndl_msg.ask_l - cndl_msg.bid_l) / 2
                hsc_c = (cndl_msg.ask_c - cndl_msg.bid_c) / 2
                data.append([dt_,
                             cndl_msg.ask_o,
                             cndl_msg.ask_h,
                             cndl_msg.ask_l,
                             cndl_msg.ask_c,
                             cndl_msg.bid_o,
                             cndl_msg.bid_h,
                             cndl_msg.bid_l,
                             cndl_msg.bid_c,
                             cndl_msg.bid_o + hsc_o,
                             cndl_msg.bid_h + hsc_h,
                             cndl_msg.bid_l + hsc_l,
                             cndl_msg.bid_c + hsc_c,
                             hsc_o,
                             hsc_h,
                             hsc_l,
                             hsc_c,
                             cndl_msg.is_complete
                             ])

            df = pd.DataFrame(data)

            if not df.empty:
                df.columns = [self.COL_NAME_TIME,
                              self.COL_NAME_ASK_OP,
                              self.COL_NAME_ASK_HI,
                              self.COL_NAME_ASK_LO,
                              self.COL_NAME_ASK_CL,
                              self.COL_NAME_BID_OP,
                              self.COL_NAME_BID_HI,
                              self.COL_NAME_BID_LO,
                              self.COL_NAME_BID_CL,
                              self.COL_NAME_MID_OP,
                              self.COL_NAME_MID_HI,
                              self.COL_NAME_MID_LO,
                              self.COL_NAME_MID_CL,
                              self.COL_NAME_HSC_OP,
                              self.COL_NAME_HSC_HI,
                              self.COL_NAME_HSC_LO,
                              self.COL_NAME_HSC_CL,
                              self.COL_NAME_COMP
                              ]

                TIME = self.COL_NAME_TIME
                if GranTm.GRAN_D <= self.__gran_id:
                    df[TIME] = df[TIME].apply(
                        lambda d: dt.datetime(d.year, d.month, d.day))

                df = df.set_index(TIME)

        return df

    def __update_df(self, df: pd.DataFrame):

        if not df.empty:

            df_comp = df[(df[self.COL_NAME_COMP])]
            df_prov = df[~(df[self.COL_NAME_COMP])]

            if not df_comp.empty:
                # "df_comp" deal with FIFO
                self.__df_comp = self.__df_comp.append(df_comp)
                droplist = self.__df_comp.index[range(0, len(df_comp))]
                self.__df_comp.drop(droplist, inplace=True)

            if not df_prov.empty:
                self.__df_prov = df_prov
            else:
                self.__df_prov = self.__df_prov[:0]  # All data delete

            self.__logger.debug("- df_comp:%s" % (df_comp.index.tolist()))
            self.__logger.debug("- df_prov:%s" % (df_prov.index.tolist()))


class CandlestickManager(Node):

    INST_ID_LIST = [
        InstApi.INST_USD_JPY,
        InstApi.INST_EUR_JPY,
        InstApi.INST_EUR_USD,
    ]

    GRAN_ID_LIST = [
        GranApi.GRAN_D,
        GranApi.GRAN_H4,
        GranApi.GRAN_H1,
        GranApi.GRAN_M10
    ]

    DATA_LENGTH_DICT = {
        GranApi.GRAN_M1: 60 * 24 * 10,   # 10 Days
        GranApi.GRAN_M2: 30 * 24 * 10,   # 10 Days
        GranApi.GRAN_M3: 20 * 24 * 10,   # 10 Days
        GranApi.GRAN_M4: 15 * 24 * 10,   # 10 Days
        GranApi.GRAN_M5: 12 * 24 * 10,   # 10 Days
        GranApi.GRAN_M10: 6 * 24 * 100,  # 1 Year
        GranApi.GRAN_M15: 4 * 24 * 10,   # 10 Days
        GranApi.GRAN_M30: 2 * 24 * 10,   # 10 Days
        GranApi.GRAN_H1: 24 * 30,        # 1 Months
        GranApi.GRAN_H2: 12 * 30,        # 1 Months
        GranApi.GRAN_H3: 8 * 30,         # 1 Months
        GranApi.GRAN_H4: 6 * 30,         # 1 Months
        GranApi.GRAN_H6: 4 * 30,         # 1 Months
        GranApi.GRAN_H8: 3 * 30 * 3,     # 3 Months
        GranApi.GRAN_H12: 2 * 30 * 3,    # 3 Months
        GranApi.GRAN_D: 365,             # 1 Year
        GranApi.GRAN_W: 4 * 12,          # 1 Year
    }

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

    def __init__(self) -> None:
        super().__init__("candlestick_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Create service server "CandlesMonitor"
        srv_type = CandlesMntSrv
        srv_name = "candles_monitor"
        callback = self.__on_recv_candlesnt_mnt
        self.__candles_mnt_srv = self.create_service(srv_type,
                                                     srv_name,
                                                     callback)

        try:
            # Create service client "Candles"
            srv_type = CandlesSrv
            srv_name = "candles"
            cli_cdl = self.__create_service_client(srv_type, srv_name)
        except RuntimeError as err:
            self.__logger.error(err)
            self.destroy_node()
            rclpy.shutdown()

        # initialize "data_map"
        self.__init_data_map(cli_cdl)

        # Create service server "HistoricalCandles"
        srv_type = HistoricalCandlesSrv
        srv_name = "historical_candles"
        callback = self.__on_recv_historical_candles
        self.__hc_srv = self.create_service(srv_type,
                                            srv_name,
                                            callback)

        # Timer(1s)
        self.__timer_1s = self.create_timer(timer_period_sec=1.0,
                                            callback=self.__on_timeout_1s)

        self.__cli_cdl = cli_cdl

        """
        dt_from = dt.datetime(2020, 5, 1)
        dt_to = dt.datetime(2020, 5, 5)

        print("1----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, dt_from, dt_to)
        print(df)
        print("2----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, dt_from)
        print(df)
        print("3----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D, None, dt_to)
        print(df)
        print("4----------------------------")
        df = self.get_dataframe(InstApi.INST_USD_JPY, GranApi.GRAN_D)
        print(df)
        """

    def __init_data_map(self, cli: Client) -> None:

        obj_map_dict = {}
        for inst_id in self.INST_ID_LIST:
            gran_dict = {}
            for gran_id in self.GRAN_ID_LIST:
                data_len = self.DATA_LENGTH_DICT[gran_id]
                obj = CandlesData(self, cli, inst_id, gran_id, data_len)
                tmp = {gran_id: obj}
                gran_dict.update(tmp)
            tmp = {inst_id: gran_dict}
            obj_map_dict.update(tmp)

        # type: Dict[InstrumentMnt][GranularityMnt]
        self.__obj_map_dict = obj_map_dict

    def __get_dataframe(self,
                        inst_id: InstApi,
                        gran_id: GranApi,
                        dt_from: dt.datetime=None,
                        dt_to: dt.datetime=None
                        ) -> pd.DataFrame:

        df = self.__obj_map_dict[inst_id][gran_id].dataframe

        if dt_from is not None:
            dftmp = df[(dt_from <= df.index)]
        else:
            dftmp = df

        if dt_to is not None:
            dftmp = dftmp[(dftmp.index <= dt_to)]

        return dftmp

    def __create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for service.")
            self.__logger.info("Waiting for [%s] service..." % (srv_name))
        return cli

    def __on_recv_historical_candles(self,
                                     req: SrvTypeRequest,
                                     rsp: SrvTypeResponse
                                     ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[historical_candles]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- gran_msg.gran_id:[%d]" % (req.gran_msg.gran_id))
        logger.debug("- inst_msg.inst_id:[%d]" % (req.inst_msg.inst_id))
        logger.debug("- dt_from:[%s]" % (req.dt_from))
        logger.debug("- dt_to:[%s]" % (req.dt_to))
        dbg_tm_start = dt.datetime.now()

        DT_FMT = CandlesData.DT_FMT

        if req.dt_from == "":
            dt_from = None
        else:
            dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)

        if req.dt_to == "":
            dt_to = None
        else:
            dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)

        df = self.__get_dataframe(req.inst_msg.inst_id,
                                  req.gran_msg.gran_id,
                                  dt_from,
                                  dt_to)

        rsp.cndl_msg_list = []
        if not df.empty:
            for time, sr in df.iterrows():
                msg = Candle()
                msg.ask_o = sr[CandlesData.COL_NAME_ASK_OP]
                msg.ask_h = sr[CandlesData.COL_NAME_ASK_HI]
                msg.ask_l = sr[CandlesData.COL_NAME_ASK_LO]
                msg.ask_c = sr[CandlesData.COL_NAME_ASK_CL]
                msg.mid_o = sr[CandlesData.COL_NAME_MID_OP]
                msg.mid_h = sr[CandlesData.COL_NAME_MID_HI]
                msg.mid_l = sr[CandlesData.COL_NAME_MID_LO]
                msg.mid_c = sr[CandlesData.COL_NAME_MID_CL]
                msg.bid_o = sr[CandlesData.COL_NAME_BID_OP]
                msg.bid_h = sr[CandlesData.COL_NAME_BID_HI]
                msg.bid_l = sr[CandlesData.COL_NAME_BID_LO]
                msg.bid_c = sr[CandlesData.COL_NAME_BID_CL]
                msg.time = time.strftime(DT_FMT)
                msg.is_complete = sr[CandlesData.COL_NAME_COMP]
                rsp.cndl_msg_list.append(msg)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- cndl_msg_list(length):[%d]" % (len(rsp.cndl_msg_list)))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[historical_candles]:End ==========")

        return rsp

    def __on_recv_candlesnt_mnt(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:
        logger = self._logger

        logger.debug("========== Service[candles_monitor]:Start ==========")
        logger.debug("<Request>")
        logger.debug("- gran_msg.gran_id:[%d]" % (req.gran_msg.gran_id))
        logger.debug("- inst_msg.inst_id:[%d]" % (req.inst_msg.inst_id))
        logger.debug("- dt_from:[%s]" % (req.dt_from))
        logger.debug("- dt_to:[%s]" % (req.dt_to))
        dbg_tm_start = dt.datetime.now()

        DT_FMT = CandlesData.DT_FMT
        INF = "inf"

        dt_from = None
        if req.dt_from != INF:
            dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)

        dt_to = None
        if req.dt_to != INF:
            dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)

        df = self.__get_dataframe(req.inst_msg.inst_id,
                                  req.gran_msg.gran_id,
                                  dt_from,
                                  dt_to)

        rsp.cndl_msg_list = []
        if not df.empty:
            for time, sr in df.iterrows():
                msg = CandleMnt()
                msg.ask_o = sr[CandlesData.COL_NAME_ASK_OP]
                msg.ask_h = sr[CandlesData.COL_NAME_ASK_HI]
                msg.ask_l = sr[CandlesData.COL_NAME_ASK_LO]
                msg.ask_c = sr[CandlesData.COL_NAME_ASK_CL]
                msg.mid_o = sr[CandlesData.COL_NAME_MID_OP]
                msg.mid_h = sr[CandlesData.COL_NAME_MID_HI]
                msg.mid_l = sr[CandlesData.COL_NAME_MID_LO]
                msg.mid_c = sr[CandlesData.COL_NAME_MID_CL]
                msg.bid_o = sr[CandlesData.COL_NAME_BID_OP]
                msg.bid_h = sr[CandlesData.COL_NAME_BID_HI]
                msg.bid_l = sr[CandlesData.COL_NAME_BID_LO]
                msg.bid_c = sr[CandlesData.COL_NAME_BID_CL]
                msg.time = time.strftime(DT_FMT)
                msg.is_complete = sr[CandlesData.COL_NAME_COMP]
                rsp.cndl_msg_list.append(msg)

        dbg_tm_end = dt.datetime.now()
        logger.debug("<Response>")
        logger.debug("- cndl_msg_list(length):[%d]" % (len(rsp.cndl_msg_list)))
        logger.debug("[Performance]")
        logger.debug("- Response time:[%s]" % (dbg_tm_end - dbg_tm_start))
        logger.debug("========== Service[order_create]:End ==========")

        return rsp

    def __on_timeout_1s(self) -> None:

        for inst_id in self.INST_ID_LIST:
            for gran_id in self.GRAN_ID_LIST:
                obj = self.__obj_map_dict[inst_id][gran_id]
                obj.update_not_complete_data()


def main(args=None):
    rclpy.init(args=args)
    cm = CandlestickManager()
    try:
        rclpy.spin(cm)
    except KeyboardInterrupt:
        pass

    cm.destroy_node()
    rclpy.shutdown()
