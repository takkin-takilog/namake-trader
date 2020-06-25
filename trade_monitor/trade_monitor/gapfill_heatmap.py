import sys
import os


from PySide2.QtWidgets import QApplication, QWidget, QMainWindow, QSizePolicy
from PySide2.QtCore import Qt, QFile, QSizeF
from PySide2.QtUiTools import QUiLoader
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QVector3D, QGuiApplication


def dataToBarDataRow(data):
    return list(QtDataVisualization.QBarDataItem(d) for d in data)


def dataToBarDataArray(data):
    return list(dataToBarDataRow(row) for row in data)


class GapFillHeatMap(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        ui = self.__load_ui(parent)
        self.setCentralWidget(ui)
        self.resize(ui.frameSize())

        self.setWindowTitle('Qt DataVisualization 3D Bars')

        self.bars = QtDataVisualization.Q3DBars()

        self.columnAxis = QtDataVisualization.QCategory3DAxis()
        self.columnAxis.setTitle('Columns')
        self.columnAxis.setTitleVisible(True)
        self.columnAxis.setLabels(['Column1', 'Column2'])
        self.columnAxis.setLabelAutoRotation(30)

        self.rowAxis = QtDataVisualization.QCategory3DAxis()
        self.rowAxis.setTitle('Rows')
        self.rowAxis.setTitleVisible(True)
        self.rowAxis.setLabels(['Row1', 'Row2'])
        self.rowAxis.setLabelAutoRotation(30)

        self.valueAxis = QtDataVisualization.QValue3DAxis()
        self.valueAxis.setTitle('Values')
        self.valueAxis.setTitleVisible(True)
        self.valueAxis.setRange(0, 5)

        self.bars.setRowAxis(self.rowAxis)
        self.bars.setColumnAxis(self.columnAxis)
        self.bars.setValueAxis(self.valueAxis)

        self.series = QtDataVisualization.QBar3DSeries()
        self.arrayData = [[1, 2], [3, 4]]
        self.series.dataProxy().addRows(dataToBarDataArray(self.arrayData))

        self.bars.setPrimarySeries(self.series)

        self.container = QWidget.createWindowContainer(self.bars)

        if not self.bars.hasContext():
            print("Couldn't initialize the OpenGL context.")
            sys.exit(-1)

        camera = self.bars.scene().activeCamera()
        camera.setYRotation(22.5)

        geometry = QGuiApplication.primaryScreen().geometry()
        size = geometry.height() * 3 / 4
        self.container.setMinimumSize(size, size)

        #self.container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.container.setFocusPolicy(Qt.StrongFocus)
        self.container.setParent(ui.widget)


        self.__ui = ui

    def __load_ui(self, parent):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "gapfill_heatmap.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent)
        ui_file.close()

        return ui

    def resizeEvent(self, event):
        fs = self.__ui.widget.frameSize()
        self.container.resize(fs)


if __name__ == "__main__":
    from PySide2.QtCore import QCoreApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = GapFillHeatMap()
    widget.show()
    sys.exit(app.exec_())
