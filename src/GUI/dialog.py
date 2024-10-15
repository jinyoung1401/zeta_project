import sys
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.uic import loadUi

class MyDialog(QDialog):
    def __init__(self):
        super(MyDialog, self).__init__()
        loadUi('dialog.ui', self)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = MyDialog()
    if dialog.exec_() == QDialog.Accepted:
        print("OK clicked")
    else:
        print("Cancel clicked")

    sys.exit(app.exec_())
