from PyQt5 import QtWidgets, uic, QtGui, QtCore
import sys

class Recipe:
    def __init__(self, name, ingredients, instructions, reviews=None):
        self.name = name
        self.ingredients = ingredients  # [(ingredient_name, amount), ...]
        self.instructions = instructions  # List of instructions
        self.reviews = reviews if reviews else []  # [(rating, comment), ...]

    def add_review(self, rating, comment):
        self.reviews.append((rating, comment))

    def average_rating(self):
        if not self.reviews:
            return 0
        total = sum(rating for rating, _ in self.reviews)
        return total / len(self.reviews)

class RecipeBookApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(RecipeBookApp, self).__init__()
        uic.loadUi('recipe.ui', self)  # UI 파일을 로드합니다.

        # 레시피 데이터 초기화
        self.recipes = []
        self.current_recipe = None

        self.back_button = self.findChild(QtWidgets.QPushButton, 'pushButton_3')
        self.back_button.clicked.connect(self.go_back_to_home)

        # 레시피 리스트 위젯 연결
        self.listWidget = self.findChild(QtWidgets.QListWidget, 'listWidget')
        self.listWidget.itemClicked.connect(self.display_recipe)

        # 스택드 위젯을 찾아서 연결
        self.stackedWidget = self.findChild(QtWidgets.QStackedWidget, 'stackedWidget')

        # 각 페이지의 위젯 연결
        self.ingredient_view = self.findChild(QtWidgets.QColumnView, 'columnView')
        self.instruction_view = self.findChild(QtWidgets.QListView, 'listView_2')
        self.review_spinbox = self.findChild(QtWidgets.QSpinBox, 'spinBox')
        self.review_text = self.findChild(QtWidgets.QTextEdit, 'textEdit_2')
        self.review_submit_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.review_submit_button.clicked.connect(self.submit_review)

        # 새 레시피 추가 버튼 연결
        self.new_button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.new_button.clicked.connect(self.add_new_recipe)

        # 리뷰 리스트 위젯 연결 (올바른 위젯 이름 사용)
        self.review_list_widget = self.findChild(QtWidgets.QListWidget, 'listWidget_2')

        self.load_sample_data()
        self.update_recipe_list()

    def go_back_to_home(self):
        # 스택 위젯의 첫 번째 페이지(홈 화면)로 이동
        self.stackedWidget.setCurrentIndex(0)

    def load_sample_data(self):
        # 샘플 레시피 데이터 추가
        self.recipes.append(Recipe("Spaghetti", [("Pasta", "200g"), ("Tomato Sauce", "150ml")],
                                   ["Boil water", "Cook pasta", "Add sauce"]))
        self.recipes.append(Recipe("Pancakes", [("Flour", "100g"), ("Eggs", "2"), ("Milk", "200ml")],
                                   ["Mix ingredients", "Pour batter into pan", "Cook on both sides"]))

    def update_recipe_list(self):
        # 레시피 리스트 업데이트
        self.listWidget.clear()
        for recipe in self.recipes:
            self.listWidget.addItem(f"{recipe.name} ({recipe.average_rating():.1f})")

    def display_recipe(self, item):
        # 선택한 레시피를 표시
        recipe_name = item.text().split(' ')[0]
        for recipe in self.recipes:
            if recipe.name == recipe_name:
                self.current_recipe = recipe
                break
        
        if self.current_recipe:
            # 재료를 표시
            model = QtGui.QStandardItemModel()  # QtGui를 사용
            for ingredient, amount in self.current_recipe.ingredients:
                item = QtGui.QStandardItem(f"{ingredient}: {amount}")
                model.appendRow(item)
            self.ingredient_view.setModel(model)

            # 조리법을 표시
            model = QtGui.QStandardItemModel()  # QtGui를 사용
            for step in self.current_recipe.instructions:
                item = QtGui.QStandardItem(step)
                model.appendRow(item)
            self.instruction_view.setModel(model)

            # 리뷰를 listWidget_2에 표시
            self.review_list_widget.clear()     # 기존 리뷰 목록 초기화
            for rating, comment in self.current_recipe.reviews:
                review_text = f"Rating: {rating}, Comment: {comment}"
                self.review_list_widget.addItem(review_text)

            # 페이지를 전환
            self.stackedWidget.setCurrentIndex(1)  # 페이지 2로 전환

    def submit_review(self):
        if self.current_recipe:
            rating = self.review_spinbox.value()
            comment = self.review_text.toPlainText()
            
            # 리뷰를 레시피에 추가
            self.current_recipe.add_review(rating, comment)
            
            # 리뷰 제출 후 초기화
            self.review_spinbox.setValue(0)
            self.review_text.clear()
            
            # 리뷰 리스트를 즉시 갱신
            self.update_review_list()

            # 레시피 리스트도 갱신 (평균 별점 갱신)
            self.update_recipe_list()

    def update_review_list(self):
        if self.current_recipe:
            self.review_list_widget.clear()
            for rating, comment in self.current_recipe.reviews:
                review_text = f"Rating: {rating}, Comment: {comment}"
                self.review_list_widget.addItem(review_text)

    def add_new_recipe(self):
        # 새 레시피 추가 로직 (간단하게 대화 상자나 새 창을 열 수 있음)
        pass

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = RecipeBookApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
