from typing import TypeVar
import datetime as dt
import pandas as pd
import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from api_msgs.srv import CandlesSrv
from api_msgs.msg import Instrument as Inst
from api_msgs.msg import Granularity as Gran
from trade_manager_msgs.srv import CandlesMntSrv
from trade_manager_msgs.msg import CandleMnt
from trade_manager_msgs.msg import InstrumentMnt as InstMnt
from trade_manager_msgs.msg import GranularityMnt as GranMnt

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")

# pd.set_option('display.max_rows', 500)


class CandlesData():

    DT_LSB_DICT = {
        GranMnt.GRAN_M1: dt.timedelta(minutes=1),  # 1 minute
        GranMnt.GRAN_M2: dt.timedelta(minutes=2),  # 2 minutes
        GranMnt.GRAN_M3: dt.timedelta(minutes=3),  # 3 minutes
        GranMnt.GRAN_M4: dt.timedelta(minutes=4),  # 4 minutes
        GranMnt.GRAN_M5: dt.timedelta(minutes=5),  # 5 minutes
        GranMnt.GRAN_M10: dt.timedelta(minutes=10),  # 10 minutes
        GranMnt.GRAN_M15: dt.timedelta(minutes=15),  # 15 minutes
        GranMnt.GRAN_M30: dt.timedelta(minutes=30),  # 30 minutes
        GranMnt.GRAN_H1: dt.timedelta(hours=1),  # 1 hour
        GranMnt.GRAN_H2: dt.timedelta(hours=2),  # 2 hours
        GranMnt.GRAN_H3: dt.timedelta(hours=3),  # 3 hours
        GranMnt.GRAN_H4: dt.timedelta(hours=4),  # 4 hours
        GranMnt.GRAN_H6: dt.timedelta(hours=6),  # 6 hours
        GranMnt.GRAN_H8: dt.timedelta(hours=8),  # 8 hours
        GranMnt.GRAN_H12: dt.timedelta(hours=12),  # 12 hours
        GranMnt.GRAN_D: dt.timedelta(days=1),  # 1 Day
        GranMnt.GRAN_W: dt.timedelta(weeks=1),  # 1 Week
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

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"
    TIMEOUT_SEC = 3.0
    MARGIN_SEC = dt.timedelta(seconds=1)

    def __init__(self,
                 node: Node,
                 srv_cli: Client,
                 inst_id: InstMnt,
                 gran_id: GranMnt,
                 data_length: int
                 ) -> None:

        minunit = self.DT_LSB_DICT[gran_id]
        self.__logger = node.get_logger()

        self.__logger.info("===== Create CandlesData object")
        self.__logger.info("inst_id:[%d], gran_id:[%d]" % (inst_id, gran_id))

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

        future = self.__request_async(dt_from, dt_to)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        df = self.__get_df_from_future(future)

        assert not df.empty, "ROSServiceError"

        df_comp = df[(df[self.COL_NAME_COMP])]
        df_prov = df[~(df[self.COL_NAME_COMP])]

        self.__interval = interval
        self.__last_update_time = self.__get_last_update_time(gran_id, dt_now)
        self.__df_comp = df_comp
        self.__df_prov = df_prov
        self.__future = None
        self.__timeout_end = 0.0

        self.__logger.debug("last_update_time:%s" % (self.__last_update_time))

    def __get_last_update_time(self,
                               gran_id: int,
                               dtin: dt.datetime
                               ) -> dt.datetime:

        if gran_id == GranMnt.GRAN_M1:
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, dtin.minute)
        elif gran_id == GranMnt.GRAN_M2:
            tmp = dtin.minute - dtin.minute % 2
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M3:
            tmp = dtin.minute - dtin.minute % 3
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M4:
            tmp = dtin.minute - dtin.minute % 4
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M5:
            tmp = dtin.minute - dtin.minute % 5
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M10:
            tmp = dtin.minute - dtin.minute % 10
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M15:
            tmp = dtin.minute - dtin.minute % 15
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif gran_id == GranMnt.GRAN_M30:
            tmp = dtin.minute - dtin.minute % 30
            dtout = dt.datetime(dtin.year, dtin.month,
                                dtin.day, dtin.hour, tmp)
        elif ((gran_id == GranMnt.GRAN_H1) or (gran_id == GranMnt.GRAN_H2) or
              (gran_id == GranMnt.GRAN_H3) or (gran_id == GranMnt.GRAN_H4) or
              (gran_id == GranMnt.GRAN_H6) or (gran_id == GranMnt.GRAN_H8) or
              (gran_id == GranMnt.GRAN_H12) or (gran_id == GranMnt.GRAN_D) or
                (gran_id == GranMnt.GRAN_W)):
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
            target_time = dt_now - self.__last_update_time - self.MARGIN_SEC
            if self.__interval < target_time:

                #dt_from = self.__last_update_time
                dt_from = self.__df_comp.index[-1] + self.__interval
                dt_to = dt_now

                self.__future = self.__request_async(dt_from, dt_to)

                self.__timeout_end = time.monotonic() + self.TIMEOUT_SEC

        else:
            if self.__future.done() and self.__future.result() is not None:

                df = self.__get_df_from_future(self.__future)
                self.__update_df(df)
                self.__future = None
            else:
                if time.monotonic() >= self.__timeout_end:
                    self.__future = None

    def __request_async(self,
                        dt_from: dt.datetime,
                        dt_to: dt.datetime
                        ) -> Future:

        req = CandlesSrv.Request()

        req.inst_msg.instrument_id = self.__inst_id
        req.gran_msg.granularity_id = self.__gran_id

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
                data.append([dt_,
                             cndl_msg.ask_o,
                             cndl_msg.ask_h,
                             cndl_msg.ask_l,
                             cndl_msg.ask_c,
                             cndl_msg.bid_o,
                             cndl_msg.bid_h,
                             cndl_msg.bid_l,
                             cndl_msg.bid_c,
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
                              self.COL_NAME_COMP
                              ]

                TIME = self.COL_NAME_TIME
                if GranMnt.GRAN_D <= self.__gran_id:
                    df[TIME] = df[TIME].apply(
                        lambda d: dt.datetime(d.year, d.month, d.day))

                df = df.set_index(TIME)

        return df

    def __update_df(self, df: pd.DataFrame):

        if not df.empty:

            self.__last_update_time += self.__interval
            self.__logger.debug("update<last_update_time>:%s" % (self.__last_update_time))

            df_comp = df[(df[self.COL_NAME_COMP])]
            df_prov = df[~(df[self.COL_NAME_COMP])]

            self.__logger.debug("df_comp:%s" % (df_comp.index.tolist()))
            self.__logger.debug("df_prov:%s" % (df_prov.index.tolist()))

            if not df_comp.empty:
                self.__df_comp.append(df_comp)
                #dftmp = self.__df_comp.append(df_comp)
                #self.__df_comp = dftmp.groupby(dftmp.index).last()
            if not df_prov.empty:
                self.__df_prov = df_prov
            else:
                self.__df_prov = self.__df_prov[:0]  # All data delete


class CandlestickManager(Node):

    DATA_LENGTH_DICT = {
        Gran.GRAN_M1: 60 * 24 * 10,  # 10 Days
        Gran.GRAN_M2: 30 * 24 * 10,  # 10 Days
        Gran.GRAN_M3: 20 * 24 * 10,  # 10 Days
        Gran.GRAN_M4: 15 * 24 * 10,  # 10 Days
        Gran.GRAN_M5: 12 * 24 * 10,  # 10 Days
        Gran.GRAN_M10: 6 * 24 * 10,  # 10 Days
        Gran.GRAN_M15: 4 * 24 * 10,  # 10 Days
        Gran.GRAN_M30: 2 * 24 * 10,  # 10 Days
        Gran.GRAN_H1: 24 * 30,  # 1 Months
        Gran.GRAN_H2: 12 * 30,  # 1 Months
        Gran.GRAN_H3: 8 * 30,  # 1 Months
        Gran.GRAN_H4: 6 * 30,  # 1 Months
        Gran.GRAN_H6: 4 * 30,  # 1 Months
        Gran.GRAN_H8: 3 * 30 * 3,  # 3 Months
        Gran.GRAN_H12: 2 * 30 * 3,  # 3 Months
        Gran.GRAN_D: 365,  # 1 Year
        Gran.GRAN_W: 4 * 12,  # 1 Year
    }

    DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

    def __init__(self) -> None:
        super().__init__("candlestick_manager")

        # Set logger lebel
        self.__logger = super().get_logger()
        self.__logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.__inst_id_list = [Inst.INST_USD_JPY]
        self.__gran_id_list = [Gran.GRAN_D, Gran.GRAN_H4, Gran.GRAN_H1,
                               Gran.GRAN_M1]

        # Create service client "Candles"
        srv_type = CandlesSrv
        srv_name = "candles"
        cli_cdl = self.__create_service_client(srv_type, srv_name)

        # initialize "data_map"
        self.__init_data_map(cli_cdl)

        # Create service server "CandlesMonitor"
        srv_type = CandlesMntSrv
        srv_name = "candles_monitor"
        callback = self.__on_recv_candlesnt_mnt
        self.__candles_mnt_srv = self.create_service(srv_type,
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
        df = self.get_dataframe(Inst.INST_USD_JPY, Gran.GRAN_D, dt_from, dt_to)
        print(df)
        print("2----------------------------")
        df = self.get_dataframe(Inst.INST_USD_JPY, Gran.GRAN_D, dt_from)
        print(df)
        print("3----------------------------")
        df = self.get_dataframe(Inst.INST_USD_JPY, Gran.GRAN_D, None, dt_to)
        print(df)
        print("4----------------------------")
        df = self.get_dataframe(Inst.INST_USD_JPY, Gran.GRAN_D)
        print(df)
        """

    def __init_data_map(self, cli: Client) -> None:

        obj_map_dict = {}
        for inst_id in self.__inst_id_list:
            gran_dict = {}
            for gran_id in self.__gran_id_list:
                data_length = self.DATA_LENGTH_DICT[gran_id]
                obj = CandlesData(self, cli, inst_id, gran_id, data_length)
                tmp = {gran_id: obj}
                gran_dict.update(tmp)
            tmp = {inst_id: gran_dict}
            obj_map_dict.update(tmp)

        # type: Dict[InstrumentMnt][GranularityMnt]
        self.__obj_map_dict = obj_map_dict

    def __get_dataframe(self,
                        inst_id: Inst,
                        gran_id: Gran,
                        dt_from: dt.datetime=None,
                        dt_to: dt.datetime=None
                        ) -> pd.DataFrame:

        df = self.__obj_map_dict[inst_id][gran_id].dataframe

        if ((dt_from is not None) and (dt_to is not None)):
            dftmp = df[((dt_from <= df.index) & (df.index <= dt_to))]
        elif dt_from is not None:
            dftmp = df[(dt_from <= df.index)]
        elif (dt_to is not None):
            dftmp = df[(df.index <= dt_to)]
        else:
            dftmp = df

        return dftmp

    def __create_service_client(self, srv_type: int, srv_name: str) -> Client:
        # Create service client
        cli = self.create_client(srv_type, srv_name)
        # Wait for a service server
        while not cli.wait_for_service(timeout_sec=1.0):
            self.__logger.info("Waiting for \"" + srv_name + "\" service...")
        return cli

    def __on_recv_candlesnt_mnt(self,
                                req: SrvTypeRequest,
                                rsp: SrvTypeResponse
                                ) -> SrvTypeResponse:

        DT_FMT = CandlesData.DT_FMT

        dt_from = None
        if req.dt_from is not None:
            dt_from = dt.datetime.strptime(req.dt_from, DT_FMT)

        dt_to = None
        if req.dt_to is not None:
            dt_to = dt.datetime.strptime(req.dt_to, DT_FMT)

        df = self.__get_dataframe(req.inst_msg.instrument_id,
                                  req.gran_msg.granularity_id,
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
                msg.bid_o = sr[CandlesData.COL_NAME_BID_OP]
                msg.bid_h = sr[CandlesData.COL_NAME_BID_HI]
                msg.bid_l = sr[CandlesData.COL_NAME_BID_LO]
                msg.bid_c = sr[CandlesData.COL_NAME_BID_CL]
                msg.time = time.strftime(DT_FMT)
                msg.is_complete = sr[CandlesData.COL_NAME_COMP]
                rsp.cndl_msg_list.append(msg)

        return rsp

    def __on_timeout_1s(self) -> None:

        for inst_id in self.__inst_id_list:
            for gran_id in self.__gran_id_list:
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
