import math
import pandas as pd
from trade_monitor.util import INST_MSG_LIST
from trade_monitor import util as utl
from trade_apl_msgs.msg import GapFillMsg

COL_NAME_DATE = "date"
COL_NAME_GPA_DIR = "gap dir"
COL_NAME_GPA_CLOSE_PRICE = "gap close price"
COL_NAME_GPA_OPEN_PRICE = "gap open price"
COL_NAME_GPA_PRICE_MID = "gap price(mid)"
COL_NAME_GPA_PRICE_REAL = "gap price(real)"
COL_NAME_VALID_FLAG = "valid flag"
COL_NAME_SUCCESS_FLAG = "success flag"
COL_NAME_GAP_FILLED_TIME = "gap filled time"
COL_NAME_MAX_OPEN_RANGE = "max open range"
COL_NAME_END_CLOSE_PRICE = "end close price"

COL_GPA_PRICE_TH = "gap price thresh"


class HeatMapManager():

    GAP_DIR_ALL = 1
    GAP_DIR_UP = 2
    GAP_DIR_DOWN = 3

    def __init__(self, sts_bar):
        self.__sts_bar = sts_bar

    def set_param(self, df_param: pd.DataFrame, inst_idx):

        df_valid = df_param[df_param[COL_NAME_VALID_FLAG]]

        max_open_price_max = df_valid[COL_NAME_MAX_OPEN_RANGE].max()

        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        margin = 5
        max_open_pips_max = utl.roundi(max_open_price_max * lsb) + margin
        # print("---------- max_open_pips_max: {}" .format(max_open_pips_max))
        hmap_col = [COL_NAME_DATE] + list(range(1, max_open_pips_max + 1))
        # print("---------- hmap_col: {}" .format(hmap_col))

        roslist = []
        for date, is_succ, mop, gpr in zip(df_valid.index,
                                           df_valid[COL_NAME_SUCCESS_FLAG],
                                           df_valid[COL_NAME_MAX_OPEN_RANGE],
                                           df_valid[COL_NAME_GPA_PRICE_REAL]):
            if is_succ:
                max_open_pips = utl.roundi(mop * lsb)
            else:
                max_open_pips = max_open_pips_max

            row_left = list(range(-1, -max_open_pips - 1, -1))
            gap_pips = utl.roundi(gpr * lsb)

            row_right = [gap_pips] * (max_open_pips_max - max_open_pips)
            roslist.append([date] + row_left + row_right)

        df_htbl = pd.DataFrame(roslist, columns=hmap_col)
        df_htbl.set_index(COL_NAME_DATE, inplace=True)

        df_hmap_base = self.__make_basemap(df_valid, df_htbl, inst_idx)
        df_hmap = df_hmap_base.sum(level=COL_GPA_PRICE_TH).sort_index()

        zero_idx = list(range(1, df_hmap.index[0]))
        df_tmp = pd.DataFrame(index=zero_idx,
                              columns=df_hmap.columns).fillna(0)
        df_hmap = pd.concat([df_tmp, df_hmap])

        df_hmap_zero = pd.DataFrame(index=df_hmap.index,
                                    columns=df_hmap.columns).fillna(0)

        date_list = df_valid.index.tolist()

        self.__df_param_mst = df_valid
        self.__df_param = df_valid
        self.__df_htbl = df_htbl
        self.__inst_idx = inst_idx
        self.__df_hmap_base = df_hmap_base
        self.__df_hmap_mst = df_hmap
        self.__df_hmap = df_hmap
        self.__df_hmap_deci = df_hmap
        self.__df_hmap_zero = df_hmap_zero
        self.__date_list = date_list
        self.__decimate_value = 0

        self.__date_step = len(df_valid)
        self.__date_pos = 0
        self.__gap_dir = self.GAP_DIR_ALL

    def reset_hmap(self, deci: int):
        self.__df_param = self.__df_param_mst
        self.__df_hmap = self.__df_hmap_mst

        self.__date_step = len(self.__df_param)
        df_new = self.__decimate_hmap(self.__df_hmap, deci)
        self.__df_hmap_deci = df_new
        self.__decimate_value = deci
        self.__date_list = self.__df_param.index.tolist()

        return df_new

    def tuned_hmap(self):
        start = self.__date_pos
        end = self.__date_pos + self.__date_step
        print("----- tuned_hmap -----")
        print("date_step: {}" .format(self.__date_step))
        print("start:{}, end:{}" .format(start, end))
        date_list = self.__df_param.index[start:end].tolist()
        df_hmap_mst = self.__df_hmap_base.loc[(date_list), :]
        print("df_hmap_mst.shape:{}" .format(df_hmap_mst.shape))

        df_hmap = df_hmap_mst.sum(level=COL_GPA_PRICE_TH)
        df_hmap = pd.concat([df_hmap, self.__df_hmap_zero]).sum(level=0)
        df_hmap.sort_index(inplace=True)
        print("df_hmap.shape:{}" .format(df_hmap.shape))

        deci = self.__decimate_value
        print("deci:{}" .format(deci))
        df_new = self.__decimate_hmap(df_hmap, deci)
        self.__df_hmap_deci = df_new
        self.__date_list = date_list

        return df_new

    def switch_dir_all(self):
        self.__gap_dir = self.GAP_DIR_ALL
        self.__update_param()

    def switch_dir_up(self):
        self.__gap_dir = self.GAP_DIR_UP
        self.__update_param()

    def switch_dir_down(self):
        self.__gap_dir = self.GAP_DIR_DOWN
        self.__update_param()

    def __update_param(self):

        df = self.__df_param_mst

        if self.__gap_dir == self.GAP_DIR_UP:
            df_param = df[df[COL_NAME_GPA_DIR] == GapFillMsg.GAP_DIR_UP]
        elif self.__gap_dir == self.GAP_DIR_DOWN:
            df_param = df[df[COL_NAME_GPA_DIR] == GapFillMsg.GAP_DIR_DOWN]
        else:
            df_param = df

        self.__date_pos = 0
        if len(df_param) < self.__date_step:
            self.__date_step = len(df_param)

        self.__df_param = df_param

    def __decimate_hmap(self, df_hmap: pd.DataFrame, deci: int):

        if deci < 2:
            return df_hmap

        col_min = df_hmap.columns[0]
        col_max = df_hmap.columns[-1]
        row_min = df_hmap.index[0]
        row_max = df_hmap.index[-1]

        rng_y = list(range(((row_min - 1) // deci) * deci + deci,
                           row_max + deci,
                           deci))

        rng_x = list(range(((col_min - 1) // deci) * deci + deci,
                           col_max + deci,
                           deci))

        self.__sts_bar.set_label_text("Generating Heat Map : [1/3]")
        self.__sts_bar.set_bar_range(0, len(rng_y) * len(rng_x))

        new_y_map = []
        cnt = 0
        for y in rng_y:
            str_y = y - deci + 1
            str_y = utl.limit(str_y, row_min, row_max)
            end_y = utl.limit(y, row_min, row_max) + 1
            y_rng = range(str_y,  end_y, 1)
            new_x_list = [y]
            for x in rng_x:
                str_x = x - deci + 1
                str_x = utl.limit(str_x, col_min, col_max)
                end_x = utl.limit(x, col_min, col_max) + 1
                x_rng = range(str_x,  end_x, 1)
                new_x_list.append(df_hmap.loc[y_rng][x_rng].max().max())
                cnt = cnt + 1
                self.__sts_bar.set_bar_value(cnt)
            new_y_map.append(new_x_list)

        idx = "Y"
        columns = [idx] + list(rng_x)
        df_new = pd.DataFrame(new_y_map, columns=columns)
        df_new.set_index(idx, inplace=True)

        return df_new

    def __make_basemap(self,
                       df_param: pd.DataFrame,
                       df_htbl: pd.DataFrame,
                       inst_idx: int
                       ):

        label = COL_NAME_GPA_PRICE_MID
        gap_price_real_max = df_param[label].max()
        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        margin = 5
        gap_pips_max = utl.roundi(gap_price_real_max * lsb) + margin

        df_base = pd.DataFrame()
        for date, htbl in df_htbl.iterrows():
            gap_pips = utl.roundi(df_param.loc[date][label] * lsb)
            collist = [htbl.to_list()] * (gap_pips_max - (gap_pips - 1))
            gpt_col = list(range(gap_pips, gap_pips_max + 1))

            df = pd.DataFrame(collist,
                              columns=df_htbl.columns)
            df[COL_GPA_PRICE_TH] = gpt_col
            df[COL_NAME_DATE] = date
            df.set_index([COL_NAME_DATE, COL_GPA_PRICE_TH], inplace=True)
            df_base = pd.concat([df_base, df])

        return df_base

    @property
    def date_list(self):
        return self.__date_list

    @property
    def shape(self):
        return self.__df_hmap_zero.shape

    @property
    def data_len(self):
        return len(self.__df_param)

    @property
    def date_step(self):
        return self.__date_step

    @date_step.setter
    def date_step(self, value):
        self.__date_step = value
        print("date_step: {}" .format(value))

    @property
    def date_pos(self):
        return self.__date_pos

    @date_pos.setter
    def date_pos(self, value):
        self.__date_pos = value
