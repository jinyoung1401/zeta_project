import sys
from PyQt5 import QtWidgets, uic

class MyApp(QtWidgets.QMainWindow):
	def __init__(self):
		super(MyApp, self).__init__()
		uic.loadUi('check_box.ui', self)

		self.checkBox.stateChanged.connect(self.update_label)

	def update_label(self):
		if self.checkBox.isChecked():
			self.label.setText("Checkbox is checked!")

		else:
			self.label.setText("Checkbox is unchecked!")

if __name__ == "__main__":

	app = QtWidgets.QApplication(sys.argv)
	window = MyApp()
	window.show()
	sys.exit(app.exec_())