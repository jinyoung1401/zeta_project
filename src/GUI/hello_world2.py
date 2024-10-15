import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic

class HelloWorld(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi('hello_world.ui', self)

if __name__ == "__main__":

	app = QApplication(sys.argv)
	window = QMainWindow()
	window.show()
	sys.exit(app.exec_())