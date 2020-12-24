import pandas as pd

# define column items
COL_DATE = "Date"
COL_TIME = "Time"
COL_O = "O"
COL_H = "H"
COL_L = "L"
COL_C = "C"
COL_WEEKDAY_ID = "Weekday_id"
COL_GOTO_ID = "Goto_id"
COL_IS_GOTO = "Is_Goto"
COL_GAP_TYP = "Gap_type"
COL_DATA_TYP = "Data_type"
COL_MONTH = "Month"

# define Week-day param
WEEKDAY_ID_MON = 0
WEEKDAY_ID_TUE = 1
WEEKDAY_ID_WED = 2
WEEKDAY_ID_THU = 3
WEEKDAY_ID_FRI = 4
WEEKDAY_ID_SAT = 5
WEEKDAY_ID_SUN = 6

WEEKDAY_ID_DICT = {
    WEEKDAY_ID_MON: "Mon",
    WEEKDAY_ID_TUE: "Tue",
    WEEKDAY_ID_WED: "Wed",
    WEEKDAY_ID_THU: "Thu",
    WEEKDAY_ID_FRI: "Fri",
    WEEKDAY_ID_SAT: "Sat",
    WEEKDAY_ID_SUN: "Sun"
}

# define Goto-day param
GOTODAY_ID_NON = 0
GOTODAY_ID_05D = 1
GOTODAY_ID_10D = 2
GOTODAY_ID_15D = 3
GOTODAY_ID_20D = 4
GOTODAY_ID_25D = 5
GOTODAY_ID_LSD = 6

GOTODAY_ID_DICT = {
    GOTODAY_ID_NON: "-",
    GOTODAY_ID_05D: "5",
    GOTODAY_ID_10D: "10",
    GOTODAY_ID_15D: "15",
    GOTODAY_ID_20D: "20",
    GOTODAY_ID_25D: "25",
    GOTODAY_ID_LSD: "L/D"
}

# define data type
GAP_TYP_HO = 1    # High - Open price
GAP_TYP_LO = 2    # Low - Open price
GAP_TYP_CO = 3    # Close - Open price

DATA_TYP_HO_MEAN = 1   # Mean of High - Open price
DATA_TYP_HO_STD = 2    # Std of High - Open price
DATA_TYP_LO_MEAN = 3   # Mean of Low - Open price
DATA_TYP_LO_STD = 4    # Std of Low - Open price
DATA_TYP_CO_MEAN = 5   # Mean of Close - Open price
DATA_TYP_CO_STD = 6    # Std of Close - Open price
DATA_TYP_CO_CSUM = 7   # Cumsum of Close - Open price


def convert_base2weekgoto(df_base: pd.DataFrame) -> pd.DataFrame:

    level = [COL_WEEKDAY_ID, COL_IS_GOTO, COL_GAP_TYP]
    df = _make_statistics_dataframe(df_base, level)

    return df


def convert_base2monthgoto(df_base: pd.DataFrame) -> pd.DataFrame:

    # level = [COL_MONTH, COL_GOTO_ID, COL_GAP_TYP]
    level = [COL_GOTO_ID, COL_GAP_TYP]
    df = _make_statistics_dataframe(df_base, level)

    return df


def _make_statistics_dataframe(df_base: pd.DataFrame,
                               level: list
                               ) -> pd.DataFrame:

    # ----- make DataFrame "Mean" -----
    df_mean = df_base.mean(level=level).sort_index()
    df_mean.reset_index(COL_GAP_TYP, inplace=True)

    df_mean[COL_DATA_TYP] = 0
    cond = df_mean[COL_GAP_TYP] == GAP_TYP_HO
    df_mean.loc[cond, COL_DATA_TYP] = DATA_TYP_HO_MEAN
    cond = df_mean[COL_GAP_TYP] == GAP_TYP_LO
    df_mean.loc[cond, COL_DATA_TYP] = DATA_TYP_LO_MEAN
    cond = df_mean[COL_GAP_TYP] == GAP_TYP_CO
    df_mean.loc[cond, COL_DATA_TYP] = DATA_TYP_CO_MEAN

    df_mean.drop(columns=COL_GAP_TYP, inplace=True)
    index = COL_DATA_TYP
    df_mean.set_index(index, append=True, inplace=True)

    # ----- make DataFrame "Std" -----
    df_std = df_base.std(level=level).sort_index()
    df_std.reset_index(COL_GAP_TYP, inplace=True)

    df_std[COL_DATA_TYP] = 0
    cond = df_std[COL_GAP_TYP] == GAP_TYP_HO
    df_std.loc[cond, COL_DATA_TYP] = DATA_TYP_HO_STD
    cond = df_std[COL_GAP_TYP] == GAP_TYP_LO
    df_std.loc[cond, COL_DATA_TYP] = DATA_TYP_LO_STD
    cond = df_std[COL_GAP_TYP] == GAP_TYP_CO
    df_std.loc[cond, COL_DATA_TYP] = DATA_TYP_CO_STD

    df_std.drop(columns=COL_GAP_TYP, inplace=True)
    index = COL_DATA_TYP
    df_std.set_index(index, append=True, inplace=True)

    # ----- make DataFrame "Cumulative Sum" -----
    cond = df_mean.index.get_level_values(COL_DATA_TYP) == DATA_TYP_CO_MEAN
    df_csum = df_mean[cond].rename(index={DATA_TYP_CO_MEAN: DATA_TYP_CO_CSUM},
                                   level=COL_DATA_TYP)
    df_csum = df_csum.cumsum(axis=1)

    # concat "df_mean" and "df_std" and "df_csum"
    return pd.concat([df_mean, df_std, df_csum]).sort_index()
