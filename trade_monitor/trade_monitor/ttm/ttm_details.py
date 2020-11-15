import os
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QFile
from PySide2.QtUiTools import QUiLoader


class TtmDetails(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui = self._load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle("TTM Details")

        self._ui = ui

    def _load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "ttm_details.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def init_resize(self):
        pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
