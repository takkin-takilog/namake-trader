import sys
import os

from PySide2.QtWidgets import QApplication, QWidget
from PySide2.QtCore import Qt, QFile, QCoreApplication  # @UnresolvedImport
from PySide2.QtUiTools import QUiLoader  # @UnresolvedImport


class GuiMonitor(QWidget):
    def __init__(self):
        super(GuiMonitor, self).__init__()
        self.load_ui()

    def load_ui(self):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gui_monitor.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        loader.load(ui_file, self)
        ui_file.close()


def main():
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = GuiMonitor()
    widget.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
