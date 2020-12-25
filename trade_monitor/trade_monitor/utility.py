from PySide2.QtCore import Qt, QRectF
from PySide2.QtGui import QImage, QPixmap, QBrush, QIcon, QPainter
from PySide2.QtGui import QLinearGradient, QColor


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
