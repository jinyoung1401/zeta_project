import sys
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QAbstractTableModel, Qt

class MyTableModel(QAbstractTableModel):
    def __init__(self, data):
        super(MyTableModel, self).__init__()
        self._data = data

    def rowCount(self, index):
        return len(self._data)

    def columnCount(self, index):
        return len(self._data[0])

    def data(self, index, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            return self._data[index.row()][index.column()]

        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        headers = ["Column 1", "Column 2", "Column 3"]
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return headers[section]

        return None

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi('table_view.ui', self)

        data = [
            ["Row 1, Col 1", "Row 1, Col 2", "Row 1, Col 3"],
            ["Row 2, Col 1", "Row 2, Col 2", "Row 2, Col 3"],
            ["Row 3, Col 1", "Row 3, Col 2", "Row 3, Col 3"],
        ]

        self.model = MyTableModel(data)
        self.tableView.setModel(self.model)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
