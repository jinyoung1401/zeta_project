import sys
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QFileDialog

class MyApp(QtWidgets.QMainWindow):
	def __init__(self):
		super(MyApp, self).__init__()
		uic.loadUi('tool_button.ui', self)

		self.toolButton.clicked.connect(self.open_file)

	def open_file(self):
		options = QFileDialog.Options()
		file_name, _ = QFileDialog.getOpenFileName(self, "Open Text File", "", "Text Files(*.txt);;All Files (*)", options=options)
		if file_name:
			with open(file_name, 'r') as file:
				file_content = file.read()
				self.textEdit.setPlainText(file_content)


if __name__ == "__main__":

	app = QtWidgets.QApplication(sys.argv)
	window = MyApp()
	window.show()
	sys.exit(app.exec_())