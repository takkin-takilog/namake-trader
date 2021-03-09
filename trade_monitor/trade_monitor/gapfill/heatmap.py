from enum import IntEnum, auto
import pandas as pd
from trade_apl_msgs.msg import GapFillTblGapRecMsg as GapMsg
from trade_monitor import utility as utl
from trade_monitor import ros_common as ros_com
from trade_monitor.gapfill.constant import VALID_INST_LIST
from trade_monitor.gapfill.constant import ColNameGap as ColNmGap


class _GapDir(IntEnum):
    """
    Gap direction.
    """
    ALL = auto()
    UP = auto()
    DOWN = auto()


class HeatMap():

    _COL_GPA_PRICE_TH = "gap price thresh"
    _GAP_UPPER_LIMIT_RAW = 500
    _MAX_OPEN_MARGIN_RAW = 5
    _GAP_MARGIN_RAW = 10

    def __init__(self, sts_bar):
        self._sts_bar = sts_bar
        self._logger = ros_com.get_logger()

    def set_param(self, df_gap: pd.DataFrame, inst_idx):

        df_valid = df_gap[df_gap[ColNmGap.VALID_FLAG.value]]
        inst_param = VALID_INST_LIST[inst_idx]

        gap_upper_limit = inst_param.convert_raw2phy(self._GAP_UPPER_LIMIT_RAW)
        flg = df_valid[ColNmGap.GAP_PRICE_MID.value] < gap_upper_limit
        df_valid = df_valid[flg]

        flg = df_valid[ColNmGap.SUCCESS_FLAG.value]
        df_succ = df_valid[flg]
        max_open_price_max = df_succ[ColNmGap.MAX_OPEN_RANGE.value].max()

        max_open_max_raw = inst_param.convert_phy2raw(max_open_price_max)
        max_open_max_raw += self._MAX_OPEN_MARGIN_RAW

        hmap_col = [ColNmGap.DATE.value] + list(range(1, max_open_max_raw + 1))

        roslist = []
        for date, is_succ, mop, gpr in zip(df_valid.index,
                                           df_valid[ColNmGap.SUCCESS_FLAG.value],
                                           df_valid[ColNmGap.MAX_OPEN_RANGE.value],
                                           df_valid[ColNmGap.GAP_PRICE_REAL.value]):
            if is_succ:
                max_open_raw = inst_param.convert_phy2raw(mop)
            else:
                max_open_raw = max_open_max_raw

            row_left = list(range(-1, -max_open_raw - 1, -1))
            gap_raw = inst_param.convert_phy2raw(gpr)

            row_right = [gap_raw] * (max_open_max_raw - max_open_raw)
            roslist.append([date] + row_left + row_right)

        df_htbl = pd.DataFrame(roslist, columns=hmap_col)
        df_htbl.set_index(ColNmGap.DATE.value, inplace=True)

        df_hmap_base = self._make_basemap(df_valid, df_htbl, inst_idx)
        df_hmap = df_hmap_base.sum(level=self._COL_GPA_PRICE_TH).sort_index()

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
        self._gap_dir = _GapDir.ALL

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

        df_hmap = df_hmap_base.sum(level=self._COL_GPA_PRICE_TH)
        df_hmap = pd.concat([df_hmap, self._df_hmap_zero]).sum(level=0)
        df_hmap.sort_index(inplace=True)

        deci = self._decimate_value
        df_new = self._decimate_hmap(df_hmap, deci)
        self._df_hmap_deci = df_new
        self._date_list = date_list

        return df_new

    def switch_dir_all(self):
        self._gap_dir = _GapDir.ALL
        self._update_param()

    def switch_dir_up(self):
        self._gap_dir = _GapDir.UP
        self._update_param()

    def switch_dir_down(self):
        self._gap_dir = _GapDir.DOWN
        self._update_param()

    def _update_param(self):

        df = self._df_param_mst

        if self._gap_dir == _GapDir.UP:
            df_param = df[df[ColNmGap.GAP_DIR.value] == GapMsg.GAP_DIR_UP]
        elif self._gap_dir == _GapDir.DOWN:
            df_param = df[df[ColNmGap.GAP_DIR.value] == GapMsg.GAP_DIR_DOWN]
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
            y_rng = range(str_y, end_y, 1)
            new_x_list = [y]
            for x in rng_x:
                str_x = x - deci + 1
                str_x = utl.limit(str_x, col_min, col_max)
                end_x = utl.limit(x, col_min, col_max) + 1
                x_rng = range(str_x, end_x, 1)
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

        label = ColNmGap.GAP_PRICE_MID.value
        gap_price_real_max = df_param[label].max()
        inst_param = VALID_INST_LIST[inst_idx]

        gap_max_raw = inst_param.convert_phy2raw(gap_price_real_max)
        gap_max_raw += self._GAP_MARGIN_RAW

        df_base = pd.DataFrame()
        for date, htbl in df_htbl.iterrows():
            gap_pips_raw = inst_param.convert_phy2raw(df_param.loc[date][label])

            collist = [htbl.to_list()] * (gap_max_raw - (gap_pips_raw - 1))
            gpt_col = list(range(gap_pips_raw, gap_max_raw + 1))

            df = pd.DataFrame(collist,
                              columns=df_htbl.columns)
            df[self._COL_GPA_PRICE_TH] = gpt_col
            df[ColNmGap.DATE.value] = date
            df.set_index([ColNmGap.DATE.value, self._COL_GPA_PRICE_TH], inplace=True)
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
