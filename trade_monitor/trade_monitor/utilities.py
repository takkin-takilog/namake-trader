from typing import TypeVar
from PySide2.QtCore import Qt, QRectF
from PySide2.QtGui import QImage, QPixmap, QBrush, QIcon, QPainter
from PySide2.QtGui import QLinearGradient, QColor
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
from rclpy.client import Client
from trade_manager_msgs.msg import Instrument as Inst
from trade_manager_msgs.msg import Granularity as Gran

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")

FMT_DTTM_API = "%Y-%m-%dT%H:%M:00.000000000Z"
FMT_DATE_YMD = "%Y-%m-%d"
FMT_TIME_HM = "%H:%M"
FMT_TIME_HMS = "%H:%M:%S"

FMT_QT_DATE_YMD = "yyyy-MM-dd"

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

GRAN_FREQ_DICT = {
    Gran.GRAN_M1: "1min",
    Gran.GRAN_M2: "2min",
    Gran.GRAN_M3: "3min",
    Gran.GRAN_M4: "4min",
    Gran.GRAN_M5: "5min",
    Gran.GRAN_M10: "10min",
    Gran.GRAN_M15: "15min",
    Gran.GRAN_M30: "30min",
    Gran.GRAN_H1: "1H",
    Gran.GRAN_H2: "2H",
    Gran.GRAN_H3: "3H",
    Gran.GRAN_H4: "4H",
    Gran.GRAN_H6: "6H",
    Gran.GRAN_H8: "8H",
    Gran.GRAN_H12: "12H",
    Gran.GRAN_D: "D"
}

WEEKDAY_ID_DICT = {
    0: "Mon",
    1: "Tue",
    2: "Wed",
    3: "Thu",
    4: "Fri",
    5: "Sat",
    6: "Sun"
}


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


class DateRangeManager():

    def __init__(self):
        self._date_list = []
        self._upper_pos = 0
        self._lower_pos = 0

    def init_date_list(self, date_list):

        self._date_list = sorted(date_list)

        if date_list:
            self._upper_pos = len(date_list) - 1
        else:
            self._upper_pos = 0

        self._lower_pos = 0

    @property
    def length(self):
        return len(self._date_list)

    @property
    def count(self):
        if self._date_list:
            count = self._upper_pos - self._lower_pos + 1
        else:
            count = 0

        return count

    @property
    def list_all(self):
        return self._date_list.copy()

    @property
    def list_within_range(self):
        return self._date_list[self._lower_pos:self._upper_pos + 1]

    @property
    def upper_pos(self):
        return self._upper_pos

    @property
    def lower_pos(self):
        return self._lower_pos

    @property
    def upper_date(self):
        return self._date_list[self._upper_pos]

    @property
    def lower_date(self):
        return self._date_list[self._lower_pos]

    @property
    def slidable_count(self):
        return self.length - self.count

    def slide(self, step):

        if self._date_list:
            if 0 < step:
                max_pos = len(self._date_list) - 1
                updated_pos = self._upper_pos + step
                if max_pos < updated_pos:
                    slide_cnt = max_pos - self._upper_pos
                else:
                    slide_cnt = step
            else:
                updated_pos = self._lower_pos + step
                if updated_pos < 0:
                    slide_cnt = 0 - self._lower_pos
                else:
                    slide_cnt = step

            self._upper_pos += slide_cnt
            self._lower_pos += slide_cnt

    def expand_range(self, step, priority_upper=True):

        if self._date_list:
            if 0 < step:
                max_pos = len(self._date_list) - 1
                if priority_upper:
                    updated_pos = self._upper_pos + step
                    if max_pos < updated_pos:
                        self._upper_pos = max_pos
                        slide_cnt = updated_pos - max_pos
                        lower_pos = self._lower_pos
                        lower_pos -= slide_cnt
                        if lower_pos < 0:
                            lower_pos = 0
                        self._lower_pos = lower_pos

                    else:
                        self._upper_pos += step
                else:
                    updated_pos = self._lower_pos - step
                    if updated_pos < 0:
                        self._lower_pos = 0
                        upper_pos = self._upper_pos
                        upper_pos -= updated_pos
                        if max_pos < upper_pos:
                            upper_pos = max_pos
                        self._upper_pos = upper_pos
                    else:
                        self._lower_pos -= step

    def shrink_range(self, step, priority_upper=True):

        if self._date_list:
            if 0 < step:
                if self.count <= step:
                    slide_cnt = self.count - 1
                else:
                    slide_cnt = step

                if priority_upper:
                    self._upper_pos -= slide_cnt
                else:
                    self._lower_pos += slide_cnt

    def set_upper(self, date):

        idx = self._date_list.index(date)
        if idx < self._lower_pos:
            idx = self._lower_pos

        self._upper_pos = idx

    def set_lower(self, date):

        idx = self._date_list.index(date)
        if self._upper_pos < idx:
            idx = self._upper_pos

        self._lower_pos = idx


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


_g_node: Node = None
_g_service_client_candle: Client = None


def set_node(node: Node) -> None:
    global _g_node
    _g_node = node


def get_node() -> Node:
    global _g_node
    return _g_node


def set_service_client_candle(client: Client) -> None:
    global _g_service_client_candle
    _g_service_client_candle = client


def get_logger():
    global _g_node
    return _g_node.get_logger()


def call_servive_sync(srv_cli: Client,
                      request: SrvTypeRequest,
                      executor: Executor=None,
                      timeout_sec: float=None
                      ) -> SrvTypeResponse:
    future = srv_cli.call_async(request)
    global _g_node
    rclpy.spin_until_future_complete(_g_node, future, executor, timeout_sec)

    if future.done() is False:
        raise Exception("future.done() is False")
    if future.result() is None:
        raise Exception("future.result() is None")

    return future.result()


def call_servive_sync_candle(request: SrvTypeRequest,
                             executor: Executor=None,
                             timeout_sec: float=None
                             ) -> SrvTypeResponse:
    global _g_service_client_candle
    result = call_servive_sync(_g_service_client_candle,
                               request,
                               executor,
                               timeout_sec)
    return result


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
