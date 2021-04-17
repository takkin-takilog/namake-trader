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


class ColNameOhlc(Enum):
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


class ColNameTrnd(Enum):
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
        return [ColNameTrnd.SMA_S.value,
                ColNameTrnd.SMA_M.value,
                ColNameTrnd.SMA_L.value
                ]

    @classmethod
    def to_list_ema(cls):
        return [ColNameTrnd.EMA_S.value,
                ColNameTrnd.EMA_M.value,
                ColNameTrnd.EMA_L.value
                ]

    @classmethod
    def to_list_wma(cls):
        return [ColNameTrnd.WMA_S.value,
                ColNameTrnd.WMA_M.value,
                ColNameTrnd.WMA_L.value
                ]

    @classmethod
    def to_list_ichmk(cls):
        return [ColNameTrnd.ICHMK_BASE.value,
                ColNameTrnd.ICHMK_CONV.value,
                ColNameTrnd.ICHMK_SPNA.value,
                ColNameTrnd.ICHMK_SPNB.value,
                ColNameTrnd.ICHMK_LAG.value
                ]

    @classmethod
    def to_list_bb(cls):
        return [ColNameTrnd.BLNGR_BASE.value,
                ColNameTrnd.BLNGR_PS1.value,
                ColNameTrnd.BLNGR_PS2.value,
                ColNameTrnd.BLNGR_PS3.value,
                ColNameTrnd.BLNGR_NS1.value,
                ColNameTrnd.BLNGR_NS2.value,
                ColNameTrnd.BLNGR_NS3.value,
                ]

class ColNameOsci(Enum):
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
        return [ColNameOsci.RSI_SMA.value,
                ColNameOsci.RSI_EMA.value
                ]

    @classmethod
    def to_list_macd(cls):
        return [ColNameOsci.MACD_MACD.value,
                ColNameOsci.MACD_SIG.value
                ]

    @classmethod
    def to_list_stochastic(cls):
        return [ColNameOsci.STCHA_K.value,
                ColNameOsci.STCHA_D.value,
                ColNameOsci.STCHA_SD.value
                ]

class ColNameSma(Enum):
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


class ColNameLine(Enum):
    """
    Line Chart dataframe column name.
    """
    DATA_TYP = "data_type"
    PEN = "pen"
    SERIES = "series"

    @classmethod
    def to_list(cls):
        return [m.value for m in cls]
