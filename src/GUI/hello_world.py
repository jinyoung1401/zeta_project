import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow

class MainWindow(QMainWindow):
	def __init__(self):
		super().__init__()

		label = QLabel("HELLOW WORLD", self)

		label.setGeometry(50,50,200,50)
		self.setWindowTitle("Hello World App")
		self.setGeometry(100,100,300,200)
if __name__ == "__main__":

	app = QApplication(sys.argv)
	window = MainWindow()
	window.show()
	sys.exit(app.exec_())