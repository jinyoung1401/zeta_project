import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileSystemModel
from PyQt5.uic import loadUi

class ColumnViewExample(QMainWindow):
    def __init__(self):
        super(ColumnViewExample, self).__init__()
        loadUi('column_view.ui', self)

        self.model = QFileSystemModel()
        self.model.setRootPath('')

        self.columnView.setModel(self.model)
        self.columnView.setRootIndex(self.model.index(''))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ColumnViewExample()
    window.show()
    sys.exit(app.exec_())
