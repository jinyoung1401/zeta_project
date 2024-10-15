import sys
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QStringListModel
from PyQt5.QtWidgets import QMessageBox

class ListViewExample(QtWidgets.QMainWindow):
	def __init__(self):
		super(ListViewExample, self).__init__()
		uic.loadUi('list_view.ui', self)

		self.listView = self.findChild(QtWidgets.QListView, 'listView')

		self.model = QStringListModel()
		self.listView.setModel(self.model)

		self.add_data()

		self.listView.clicked.connect(self.on_item_clicked)

	def add_data(self):
		items = ['Item 1', 'Item 2', 'Item 3', 'Item 4', 'Item 5']
		self.model.setStringList(items)

	def on_item_clicked(self, index):
		item = self.model.data(index, 0)
		QMessageBox.information(self, 'Item Clicked', f'You clicked: {item}')

if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	window = ListViewExample()
	window.show()
	sys.exit(app.exec_())
