import pandas as pd
from enum import Enum, IntEnum


class ColumnName(Enum):
    """
    Pandas dataframe column name.
    """
    DATE = "Date"
    TIME = "Time"
    CDL_O = "O"
    CDL_H = "H"
    CDL_L = "L"
    CDL_C = "C"
    WEEKDAY_ID = "Weekday_id"
    GOTO_ID = "Goto_id"
    IS_GOTO = "Is_Goto"
    GAP_TYP = "Gap_type"
    DATA_TYP = "Data_type"
    MONTH = "Month"


class GapType(IntEnum):
    """
    Gap type.
    """
    HO = 1    # High - Open price
    LO = 2    # Low - Open price
    CO = 3    # Close - Open price


class DataType(IntEnum):
    """
    Data type.
    """
    HO_MEAN = 1   # Mean of High - Open price
    HO_STD = 2    # Std of High - Open price
    LO_MEAN = 3   # Mean of Low - Open price
    LO_STD = 4    # Std of Low - Open price
    CO_MEAN = 5   # Mean of Close - Open price
    CO_STD = 6    # Std of Close - Open price
    CO_CSUM = 7   # Cumsum of Close - Open price


def convert_base2weekgoto(df_base: pd.DataFrame) -> pd.DataFrame:

    level = [ColumnName.WEEKDAY_ID.value,
             ColumnName.IS_GOTO.value,
             ColumnName.GAP_TYP.value
             ]
    df = _make_statistics_dataframe(df_base, level)

    return df


def convert_base2monthgoto(df_base: pd.DataFrame) -> pd.DataFrame:

    # level = [ColumnName.MONTH, ColumnName.GOTO_ID, ColumnName.GAP_TYP]
    level = [ColumnName.GOTO_ID.value,
             ColumnName.GAP_TYP.value
             ]
    df = _make_statistics_dataframe(df_base, level)

    return df


def _make_statistics_dataframe(df_base: pd.DataFrame,
                               level: list
                               ) -> pd.DataFrame:

    # ----- make DataFrame "Mean" -----
    df_mean = df_base.mean(level=level).sort_index()
    df_mean.reset_index(ColumnName.GAP_TYP.value, inplace=True)

    df_mean[ColumnName.DATA_TYP.value] = 0
    cond = df_mean[ColumnName.GAP_TYP.value] == GapType.HO.value
    df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.HO_MEAN.value
    cond = df_mean[ColumnName.GAP_TYP.value] == GapType.LO.value
    df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.LO_MEAN.value
    cond = df_mean[ColumnName.GAP_TYP.value] == GapType.CO.value
    df_mean.loc[cond, ColumnName.DATA_TYP.value] = DataType.CO_MEAN.value

    df_mean.drop(columns=ColumnName.GAP_TYP.value, inplace=True)
    index = ColumnName.DATA_TYP.value
    df_mean.set_index(index, append=True, inplace=True)

    # ----- make DataFrame "Std" -----
    df_std = df_base.std(level=level).sort_index()
    df_std.reset_index(ColumnName.GAP_TYP.value, inplace=True)

    df_std[ColumnName.DATA_TYP.value] = 0
    cond = df_std[ColumnName.GAP_TYP.value] == GapType.HO.value
    df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.HO_STD.value
    cond = df_std[ColumnName.GAP_TYP.value] == GapType.LO.value
    df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.LO_STD.value
    cond = df_std[ColumnName.GAP_TYP.value] == GapType.CO.value
    df_std.loc[cond, ColumnName.DATA_TYP.value] = DataType.CO_STD.value

    df_std.drop(columns=ColumnName.GAP_TYP.value, inplace=True)
    index = ColumnName.DATA_TYP.value
    df_std.set_index(index, append=True, inplace=True)

    # ----- make DataFrame "Cumulative Sum" -----
    cond = df_mean.index.get_level_values(ColumnName.DATA_TYP.value) == DataType.CO_MEAN.value
    df_csum = df_mean[cond].rename(index={DataType.CO_MEAN.value: DataType.CO_CSUM.value},
                                   level=ColumnName.DATA_TYP.value)
    df_csum = df_csum.cumsum(axis=1)

    # concat "df_mean" and "df_std" and "df_csum"
    return pd.concat([df_mean, df_std, df_csum]).sort_index()
