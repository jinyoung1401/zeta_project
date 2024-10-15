import sys
from PyQt5 import QtWidgets, uic

class MyApp(QtWidgets.QMainWindow):
	def __init__(self):
		super(MyApp, self).__init__()
		uic.loadUi('command_link_button.ui', self)

		self.commandLinkButton = self.findChild(QtWidgets.QCommandLinkButton, 'commandLinkButton')
		self.commandLinkButton.clicked.connect(self.on_command_link_button_clicked)
		self.show()
	def on_command_link_button_clicked(self):
		QtWidgets.QMessageBox.information(self, "Info", "Command Link Button Clicked!")
		
if __name__ == "__main__":

	app = QtWidgets.QApplication(sys.argv)
	window = MyApp()
	window.show()
	sys.exit(app.exec_())