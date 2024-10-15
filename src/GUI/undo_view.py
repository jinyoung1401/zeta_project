import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QUndoStack, QUndoView, QTextEdit, QVBoxLayout, QWidget
from PyQt5 import uic
from PyQt5.QtWidgets import QUndoCommand

class TextEditCommand(QUndoCommand):
	def __init__(self, text_edit, prev_text, new_text):
		super(TextEditCommand, self).__init__()
		self.text_edit = text_edit
		self.prev_text = prev_text
		self.new_text = new_text

	def undo(self):
		self.text_edit.blockSignals(True)
		self.text_edit.setPlainText(self.prev_text)
		self.text_edit.blockSignals(False)

	def redo(self):
		self.text_edit.blockSignals(True)
		self.text_edit.setPlainText(self.new_text)
		self.text_edit.blockSignals(False)

class MainWindow(QMainWindow):
	def __init__(self):
		super(MainWindow, self).__init__()
		uic.loadUi('undo_view.ui', self)

		self.text_edit = self.findChild(QTextEdit, 'textEdit')
		self.undo_view = self.findChild(QUndoView, 'undoView')

		self.undo_stack = QUndoStack(self)
		self.undo_view.setStack(self.undo_stack)

		self.text_edit.textChanged.connect(self.text_changed)
		self.prev_text = ""

	def text_changed(self):
		new_text = self.text_edit.toPlainText()
		command = TextEditCommand(self.text_edit, self.prev_text, new_text)
		self.undo_stack.push(command)
		self. prev_text = new_text

app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()