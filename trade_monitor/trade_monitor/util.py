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
        self.__msg_id = msg_id
        self.__text = text

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text


class MsgInstDict():

    def __init__(self,
                 msg_id: int,
                 text: str,
                 decimal_digit: int,
                 min_unit: str
                 ) -> None:
        self.__msg_id = msg_id
        self.__text = text
        self.__decimal_digit = decimal_digit
        self.__min_unit = min_unit

    @property
    def msg_id(self) -> int:
        return self.__msg_id

    @property
    def text(self) -> str:
        return self.__text

    @property
    def decimal_digit(self) -> int:
        return self.__decimal_digit

    @property
    def min_unit(self) -> str:
        return self.__min_unit


class GradientManager():

    __RBG_MAX = 255

    def __init__(self) -> None:
        self.__grad = 0
        self.__slope = 0
        self.__intercept = 0
        self.__intensityMax = 1
        self.__intensityMin = 0
        self.__image = QImage()

    @property
    def intensityMax(self):
        return self.__intensityMax

    @property
    def intensityMin(self):
        return self.__intensityMin

    def setGradient(self, grad: QLinearGradient) -> None:
        self.__grad = grad
        self.updateColorTable(self.__intensityMax,
                              self.__intensityMin)

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

        self.__grad.setStart(0, 0)
        self.__grad.setFinalStop(0, self.__RBG_MAX)
        # create image and fill it with gradient
        image = QImage(1, self.__RBG_MAX + 1, QImage.Format_RGB32)
        painter = QPainter(image)
        painter.fillRect(image.rect(), self.__grad)
        painter.end()

        self.__slope = self.__RBG_MAX / (min_ - max_)
        self.__intercept = self.__RBG_MAX * max_ / (max_ - min_)

        self.__intensityMax = max_
        self.__intensityMin = min_
        self.__image = image

        """
        print("w:{}, h:{}" .format(image.width(), image.height()))
        for i in range(self.__RBG_MAX + 1):
            print("[{}]:{}" .format(i, image.pixelColor(0, i)))
        """

    def convertValueToColor(self, value) -> QColor:
        calcf = self.__slope * value + self.__intercept
        return self.__image.pixelColor(0, calcf)

    def convertValueToIntensity(self, value):
        calcf = self.__slope * value + self.__intercept
        print("calcf: {}" .format(calcf))
        return int(calcf)

    def setRect(self, rect: QRectF):
        self.__grad.setStart(rect.topLeft())
        self.__grad.setFinalStop(0, rect.bottom())

    def getGradient(self) -> QLinearGradient:
        return self.__grad


INST_MSG_LIST = [
    MsgInstDict(Inst.INST_USD_JPY, "USD/JPY", 3, "0.001"),
    MsgInstDict(Inst.INST_EUR_JPY, "EUR/JPY", 3, "0.001"),
    MsgInstDict(Inst.INST_EUR_USD, "EUR/USD", 5, "0.00001"),
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
