from PySide2.QtCore import Qt, QRectF
from PySide2.QtGui import QImage, QPixmap, QBrush, QIcon, QPainter
from PySide2.QtGui import QLinearGradient, QColor
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran

DT_FMT = "%Y-%m-%dT%H:%M:00.000000000Z"

# Candlestick data frame column name
COL_NAME_TIME = "time"
COL_NAME_ASK_OP = "open(Ask)"
COL_NAME_ASK_HI = "high(Ask)"
COL_NAME_ASK_LO = "low(Ask)"
COL_NAME_ASK_CL = "close(Ask)"
COL_NAME_MID_OP = "open(Mid)"
COL_NAME_MID_HI = "high(Mid)"
COL_NAME_MID_LO = "low(Mid)"
COL_NAME_MID_CL = "close(Mid)"
COL_NAME_BID_OP = "open(Bid)"
COL_NAME_BID_HI = "high(Bid)"
COL_NAME_BID_LO = "low(Bid)"
COL_NAME_BID_CL = "close(Bid)"
COL_NAME_COMP = "complete"

CANDLE_COL_NAME_LIST = [COL_NAME_TIME,
                        COL_NAME_ASK_OP,
                        COL_NAME_ASK_HI,
                        COL_NAME_ASK_LO,
                        COL_NAME_ASK_CL,
                        COL_NAME_MID_OP,
                        COL_NAME_MID_HI,
                        COL_NAME_MID_LO,
                        COL_NAME_MID_CL,
                        COL_NAME_BID_OP,
                        COL_NAME_BID_HI,
                        COL_NAME_BID_LO,
                        COL_NAME_BID_CL,
                        COL_NAME_COMP
                        ]


class MsgGranDict():

    def __init__(self,
                 msg_id: int,
                 text: str
                 ) -> None:
        self._msg_id = msg_id
        self._text = text

    @property
    def msg_id(self) -> int:
        return self._msg_id

    @property
    def text(self) -> str:
        return self._text


class MsgInstDict():

    def __init__(self,
                 msg_id: int,
                 namespace: str,
                 text: str,
                 decimal_digit: int,
                 min_unit: str
                 ) -> None:
        self._msg_id = msg_id
        self._namespace = namespace
        self._text = text
        self._decimal_digit = decimal_digit
        self._min_unit = min_unit

    @property
    def msg_id(self) -> int:
        return self._msg_id

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def text(self) -> str:
        return self._text

    @property
    def decimal_digit(self) -> int:
        return self._decimal_digit

    @property
    def min_unit(self) -> str:
        return self._min_unit


class GradientManager():

    _RBG_MAX = 255

    def __init__(self) -> None:
        self._grad = 0
        self._slope = 0
        self._intercept = 0
        self._intensityMax = 1
        self._intensityMin = 0
        self._image = QImage()

    @property
    def intensityMax(self):
        return self._intensityMax

    @property
    def intensityMin(self):
        return self._intensityMin

    def setGradient(self, grad: QLinearGradient) -> None:
        self._grad = grad
        self.updateColorTable(self._intensityMax,
                              self._intensityMin)

    @staticmethod
    def generateIcon(grad: QLinearGradient,
                     width: int,
                     height: int
                     ) -> QIcon:

        grad.setFinalStop(0, height)
        pm = QPixmap(width, height)
        pmp = QPainter(pm)
        pmp.setBrush(QBrush(grad))
        pmp.setPen(Qt.NoPen)
        pmp.drawRect(0, 0, width, height)
        pmp.end()

        return QIcon(pm)

    def updateColorTable(self, max_, min_=None):

        if min_ is None:
            min_ = -max_

        self._grad.setStart(0, 0)
        self._grad.setFinalStop(0, self._RBG_MAX)
        # create image and fill it with gradient
        image = QImage(1, self._RBG_MAX + 1, QImage.Format_RGB32)
        painter = QPainter(image)
        painter.fillRect(image.rect(), self._grad)
        painter.end()

        self._slope = self._RBG_MAX / (min_ - max_)
        self._intercept = self._RBG_MAX * max_ / (max_ - min_)

        self._intensityMax = max_
        self._intensityMin = min_
        self._image = image

    def convertValueToColor(self, value) -> QColor:
        calcf = self._slope * value + self._intercept
        calcf = self._limit_rgb(calcf)
        return self._image.pixelColor(0, calcf)

    def convertValueToIntensity(self, value):
        calcf = self._slope * value + self._intercept
        return int(calcf)

    def setRect(self, rect: QRectF):
        self._grad.setStart(rect.topLeft())
        self._grad.setFinalStop(0, rect.bottom())

    def getGradient(self) -> QLinearGradient:
        return self._grad

    def _limit_rgb(self, val_in: float):

        if self._RBG_MAX < val_in:
            val_out = self._RBG_MAX
        elif val_in < 0:
            val_out = 0
        else:
            val_out = val_in

        return val_out


INST_MSG_LIST = [
    MsgInstDict(Inst.INST_USD_JPY, "usdjpy", "USD/JPY", 3, "0.001"),
    MsgInstDict(Inst.INST_EUR_JPY, "eurjpy", "EUR/JPY", 3, "0.001"),
    MsgInstDict(Inst.INST_EUR_USD, "eurusd", "EUR/USD", 5, "0.00001"),
]

GRAN_MSG_LIST = [
    MsgGranDict(Gran.GRAN_D, "日足"),
    MsgGranDict(Gran.GRAN_H4, "４時間足"),
    MsgGranDict(Gran.GRAN_H1, "１時間足"),
    MsgGranDict(Gran.GRAN_M10, "１０分足"),
]

SPREAD_MSG_LIST = [
    "Mid",
    "Ask",
    "Bid"
]


def remove_all_items_of_comboBox(combo_box):

    idx = combo_box.currentIndex()
    while -1 < idx:
        combo_box.removeItem(idx)
        idx = combo_box.currentIndex()


def limit(val, min_val, max_val):

    if val < min_val:
        ret_val = min_val
    elif max_val < val:
        ret_val = max_val
    else:
        ret_val = val

    return ret_val


def roundi(a: float):

    if 0 < a:
        ans = int(a + 0.5)
    else:
        ans = int(a - 0.5)

    return ans
