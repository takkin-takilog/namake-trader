from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from trade_monitor.constant import InstParam, GranParam

VALID_INST_LIST = [
    InstParam.USDJPY,
    InstParam.EURJPY,
    InstParam.EURUSD
]

VALID_GRAN_LIST = [
    GranParam.D,
    GranParam.H1,
    GranParam.M10,
    GranParam.M1,
]


class ColOhlc(Enum):
    """
    Pandas OHLC dataframe column name.
    """
    DATETIME = "datetime"
    # ---------- OHLC ----------
    MID_O = "mid_o"
    MID_H = "mid_h"
    MID_L = "mid_l"
    MID_C = "mid_c"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColTrnd(Enum):
    """
    Pandas Trend dataframe column name.
    """
    # ---------- MA(Moving Average) ----------
    SMA_S = "sma_s"
    SMA_M = "sma_m"
    SMA_L = "sma_l"
    EMA_S = "ema_s"
    EMA_M = "ema_m"
    EMA_L = "ema_l"
    WMA_S = "wma_s"
    WMA_M = "wma_m"
    WMA_L = "wma_l"
    # ---------- Ichimoku Kinko ----------
    ICHMK_BASE = "ichmk_base"
    ICHMK_CONV = "ichmk_conv"
    ICHMK_SPNA = "ichmk_sapn_a"
    ICHMK_SPNB = "ichmk_sapn_b"
    ICHMK_LAG = "ichmk_lag"
    # ---------- Bollinger Bands ----------
    BLNGR_BASE = "blngr_base"
    BLNGR_PS1 = "blngr_ps1"
    BLNGR_PS2 = "blngr_ps2"
    BLNGR_PS3 = "blngr_ps3"
    BLNGR_NS1 = "blngr_ns1"
    BLNGR_NS2 = "blngr_ns2"
    BLNGR_NS3 = "blngr_ns3"

    @classmethod
    def to_list_all(cls):
        return [m.value for m in cls]

    @classmethod
    def to_list_sma(cls):
        return [ColTrnd.SMA_S.value,
                ColTrnd.SMA_M.value,
                ColTrnd.SMA_L.value
                ]

    @classmethod
    def to_list_ema(cls):
        return [ColTrnd.EMA_S.value,
                ColTrnd.EMA_M.value,
                ColTrnd.EMA_L.value
                ]

    @classmethod
    def to_list_wma(cls):
        return [ColTrnd.WMA_S.value,
                ColTrnd.WMA_M.value,
                ColTrnd.WMA_L.value
                ]

    @classmethod
    def to_list_ichmk(cls):
        return [ColTrnd.ICHMK_BASE.value,
                ColTrnd.ICHMK_CONV.value,
                ColTrnd.ICHMK_SPNA.value,
                ColTrnd.ICHMK_SPNB.value,
                ColTrnd.ICHMK_LAG.value
                ]

    @classmethod
    def to_list_bb(cls):
        return [ColTrnd.BLNGR_BASE.value,
                ColTrnd.BLNGR_PS1.value,
                ColTrnd.BLNGR_PS2.value,
                ColTrnd.BLNGR_PS3.value,
                ColTrnd.BLNGR_NS1.value,
                ColTrnd.BLNGR_NS2.value,
                ColTrnd.BLNGR_NS3.value,
                ]


class ColOsci(Enum):
    """
    Pandas Oscillator dataframe column name.
    """
    # ---------- RSI ----------
    RSI_SMA = "rsi_sma"
    RSI_EMA = "rsi_ema"
    # ---------- MACD ----------
    MACD_MACD = "macd_macd"
    MACD_SIG = "macd_sig"
    # ---------- Stochastics ----------
    STCHA_K = "stcha_k"
    STCHA_D = "stcha_d"
    STCHA_SD = "stcha_sd"

    @classmethod
    def to_list_all(cls):
        return [m.value for m in cls]

    @classmethod
    def to_list_rsi(cls):
        return [ColOsci.RSI_SMA.value,
                ColOsci.RSI_EMA.value
                ]

    @classmethod
    def to_list_macd(cls):
        return [ColOsci.MACD_MACD.value,
                ColOsci.MACD_SIG.value
                ]

    @classmethod
    def to_list_stochastic(cls):
        return [ColOsci.STCHA_K.value,
                ColOsci.STCHA_D.value,
                ColOsci.STCHA_SD.value
                ]


class ColSma(Enum):
    """
    Pandas SMA(Simple Moving Average) dataframe column name.
    """
    DATETIME = "datetime"
    CRS_TYP = "cross_type"
    CRS_LVL = "cross_level"
    ANG_S = "angle_s"
    ANG_M = "angle_m"
    ANG_L = "angle_l"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColMacdGdc(Enum):
    """
    Pandas MACD(Golden/Death cross) dataframe column name.
    """
    DATETIME = "datetime"
    SIG_TYP = "signal_type"
    SIG_VAL = "signal_value"
    MACD_SLP = "macd_slope"
    EXIT_COND = "exit_cond"
    EXIT_DATETIME = "exit_datetime"
    PL_PRICE = "profit_loss_price"
    MAX_LOSS_PRICE = "max_loss_price"
    ADD_ENTRY_ID = "add_entry_id"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColMacdZlc(Enum):
    """
    Pandas MACD(Zero line cross) dataframe column name.
    """
    DATETIME = "datetime"
    ADD_ENTRY_ID = "add_entry_id"
    SIG_TYP = "signal_type"
    EXIT_COND = "exit_cond"
    EXIT_DATETIME = "exit_datetime"
    PL_PRICE = "profit_loss_price"
    MAX_LOSS_PRICE = "max_loss_price"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class ColLine(Enum):
    """
    Line Chart dataframe column name.
    """
    DATA_TYP = "data_type"
    PEN = "pen"
    SERIES = "series"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]


class OsciTyp(Enum):
    """
    Oscillator type.
    """
    RSI = "RSI"
    MACD = "MACD"
    STOCHASTICS = "Stochastics"
