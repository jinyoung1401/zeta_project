import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTreeView, QFileSystemModel
from PyQt5 import uic

class TreeViewExample(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi('tree_view.ui', self)

		self.treeView=self.findChild(QTreeView, 'treeView')

		model = QFileSystemModel()
		model.setRootPath('')

		self.treeView.setModel(model)
		self.treeView.setRootIndex(model.index(''))

if __name__ == '__main__':
	app = QApplication(sys.argv)
	window = TreeViewExample()
	window.show()
	sys.exit(app.exec_())