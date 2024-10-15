import sys
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.uic import loadUi
from PyQt5 import uic

class RadioButtonDemo(QDialog):
	def __init__(self):
		super(RadioButtonDemo, self).__init__()
		uic.loadUi('radio_button.ui', self)

		self.radioButton_1.toggled.connect(self.onRadioButtonToggled)
		self.radioButton_2.toggled.connect(self.onRadioButtonToggled)
		self.radioButton_3.toggled.connect(self.onRadioButtonToggled)
	
	def onRadioButtonToggled(self):
		radio_button = self.sender()
		if radio_button.isChecked():
			self.label.setText(f"Selected item: {radio_button.objectName()}")

if __name__ == "__main__":

	app = QApplication(sys.argv)
	window = RadioButtonDemo()
	window.show()
	sys.exit(app.exec_())