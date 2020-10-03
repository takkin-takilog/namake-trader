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

    GAP_UPPER_LIMIT = 500   # pips

    def __init__(self, sts_bar):
        self._sts_bar = sts_bar

    def set_param(self, df_param: pd.DataFrame, inst_idx):

        df_valid = df_param[df_param[COL_NAME_VALID_FLAG]]

        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        gap_upper_limit = self.GAP_UPPER_LIMIT / lsb
        flg = df_valid[COL_NAME_GPA_PRICE_MID] < gap_upper_limit
        df_valid = df_valid[flg]

        flg = df_valid[COL_NAME_SUCCESS_FLAG]
        df_succ = df_valid[flg]
        max_open_price_max = df_succ[COL_NAME_MAX_OPEN_RANGE].max()

        margin = 5
        max_open_pips_max = utl.roundi(max_open_price_max * lsb) + margin
        hmap_col = [COL_NAME_DATE] + list(range(1, max_open_pips_max + 1))

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

        df_hmap_base = self._make_basemap(df_valid, df_htbl, inst_idx)
        df_hmap = df_hmap_base.sum(level=COL_GPA_PRICE_TH).sort_index()

        zero_idx = list(range(1, df_hmap.index[0]))
        df_tmp = pd.DataFrame(index=zero_idx,
                              columns=df_hmap.columns).fillna(0)
        df_hmap = pd.concat([df_tmp, df_hmap])

        df_hmap_zero = pd.DataFrame(index=df_hmap.index,
                                    columns=df_hmap.columns).fillna(0)

        date_list = df_valid.index.tolist()

        self._df_param_mst = df_valid
        self._df_param = df_valid
        self._df_hmap_base = df_hmap_base
        self._df_hmap_mst = df_hmap
        self._df_hmap = df_hmap
        self._df_hmap_deci = df_hmap
        self._df_hmap_zero = df_hmap_zero
        self._date_list = date_list
        self._decimate_value = 0

        self._date_step = len(df_valid)
        self._date_pos = 0
        self._gap_dir = self.GAP_DIR_ALL

    def reset_hmap(self, deci: int):
        self._df_param = self._df_param_mst
        self._df_hmap = self._df_hmap_mst

        self._date_step = len(self._df_param)
        df_new = self._decimate_hmap(self._df_hmap, deci)
        self._df_hmap_deci = df_new
        self._decimate_value = deci
        self._date_list = self._df_param.index.tolist()

        return df_new

    def tuned_hmap(self):
        start = self._date_pos
        end = self._date_pos + self._date_step
        date_list = self._df_param.index[start:end].tolist()
        df_hmap_base = self._df_hmap_base.loc[(date_list), :]

        df_hmap = df_hmap_base.sum(level=COL_GPA_PRICE_TH)
        df_hmap = pd.concat([df_hmap, self._df_hmap_zero]).sum(level=0)
        df_hmap.sort_index(inplace=True)

        deci = self._decimate_value
        df_new = self._decimate_hmap(df_hmap, deci)
        self._df_hmap_deci = df_new
        self._date_list = date_list

        return df_new

    def switch_dir_all(self):
        self._gap_dir = self.GAP_DIR_ALL
        self._update_param()

    def switch_dir_up(self):
        self._gap_dir = self.GAP_DIR_UP
        self._update_param()

    def switch_dir_down(self):
        self._gap_dir = self.GAP_DIR_DOWN
        self._update_param()

    def _update_param(self):

        df = self._df_param_mst

        if self._gap_dir == self.GAP_DIR_UP:
            df_param = df[df[COL_NAME_GPA_DIR] == GapFillMsg.GAP_DIR_UP]
        elif self._gap_dir == self.GAP_DIR_DOWN:
            df_param = df[df[COL_NAME_GPA_DIR] == GapFillMsg.GAP_DIR_DOWN]
        else:
            df_param = df

        self._date_pos = 0
        if len(df_param) < self._date_step:
            self._date_step = len(df_param)

        self._df_param = df_param

    def _decimate_hmap(self, df_hmap: pd.DataFrame, deci: int):

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

        self._sts_bar.set_label_text("Generating Heat Map : [1/3]")
        self._sts_bar.set_bar_range(0, len(rng_y) * len(rng_x))

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
                self._sts_bar.set_bar_value(cnt)
            new_y_map.append(new_x_list)

        idx = "Y"
        columns = [idx] + list(rng_x)
        df_new = pd.DataFrame(new_y_map, columns=columns)
        df_new.set_index(idx, inplace=True)

        return df_new

    def _make_basemap(self,
                       df_param: pd.DataFrame,
                       df_htbl: pd.DataFrame,
                       inst_idx: int
                       ):

        label = COL_NAME_GPA_PRICE_MID
        gap_price_real_max = df_param[label].max()
        decimal_digit = INST_MSG_LIST[inst_idx].decimal_digit
        lsb = math.pow(10, decimal_digit)

        margin = 10
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
        return self._date_list

    @property
    def shape(self):
        return self._df_hmap_zero.shape

    @property
    def data_len(self):
        return len(self._df_param)

    @property
    def date_step(self):
        return self._date_step

    @date_step.setter
    def date_step(self, value):
        self._date_step = value

    @property
    def date_pos(self):
        return self._date_pos

    @date_pos.setter
    def date_pos(self, value):
        self._date_pos = value
