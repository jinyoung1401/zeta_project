from PyQt5 import QtWidgets, uic, QtGui
import sys
import json
import requests
from bs4 import BeautifulSoup

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

    def to_dict(self):
        return {
            'name': self.name,
            'ingredients': self.ingredients,
            'instructions': self.instructions,
            'reviews': self.reviews
        }

    @staticmethod
    def from_dict(data):
        return Recipe(data['name'], data['ingredients'], data['instructions'], data['reviews'])

class RecipeBookApp(QtWidgets.QMainWindow):
    def __init__(self):
        super(RecipeBookApp, self).__init__()
        uic.loadUi('recipe.ui', self)  # UI 파일을 로드합니다.

        self.recipes = []
        self.current_recipe = None

        # 버튼과 위젯 연결
        self.back_button = self.findChild(QtWidgets.QPushButton, 'pushButton_3')
        self.back_button.clicked.connect(self.go_back_to_home)

        self.listWidget = self.findChild(QtWidgets.QListWidget, 'listWidget')
        self.listWidget.itemClicked.connect(self.display_recipe)

        self.stackedWidget = self.findChild(QtWidgets.QStackedWidget, 'stackedWidget')

        self.ingredient_view = self.findChild(QtWidgets.QListView, 'listView')
        self.instruction_view = self.findChild(QtWidgets.QListView, 'listView_2')
        self.review_spinbox = self.findChild(QtWidgets.QSpinBox, 'spinBox')
        self.review_text = self.findChild(QtWidgets.QTextEdit, 'textEdit_2')
        self.review_submit_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.review_submit_button.clicked.connect(self.submit_review)

        self.new_button = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.new_button.clicked.connect(self.add_new_recipe_from_web)

        self.new_manual_button = self.findChild(QtWidgets.QPushButton, 'pushButton_4')
        self.new_manual_button.clicked.connect(self.add_new_recipe_manually)

        self.review_list_widget = self.findChild(QtWidgets.QListWidget, 'listWidget_2')

        self.delete_button = self.findChild(QtWidgets.QPushButton, 'pushButton_5')
        self.delete_button.clicked.connect(self.delete_recipe)

        self.load_recipes()
        self.update_recipe_list()

    def go_back_to_home(self):
        self.stackedWidget.setCurrentIndex(0)

    def save_recipes(self):
        with open('recipes.json', 'w') as f:
            json.dump([recipe.to_dict() for recipe in self.recipes], f)

    def load_recipes(self):
        try:
            with open('recipes.json', 'r') as f:
                recipes_data = json.load(f)
                self.recipes = [Recipe.from_dict(data) for data in recipes_data]
        except FileNotFoundError:
            self.load_sample_data()  # 파일이 없으면 샘플 데이터를 로드

    def update_recipe_list(self):
        self.listWidget.clear()
        for recipe in self.recipes:
            self.listWidget.addItem(f"{recipe.name} ({recipe.average_rating():.1f})")

    def load_sample_data(self):
        sample_recipes = [
            Recipe("Sample Recipe 1", [("Sugar", "100g"), ("Flour", "200g")], ["Mix ingredients", "Bake at 180C for 30 minutes"]),
            Recipe("Sample Recipe 2", [("Salt", "1 tsp"), ("Water", "500ml")], ["Boil water", "Add salt", "Stir until dissolved"])
        ]
        self.recipes = sample_recipes
        self.save_recipes()


    def load_recipes(self):
        try:
            with open('recipes.json', 'r') as f:
                recipes_data = json.load(f)
                self.recipes = [Recipe.from_dict(data) for data in recipes_data]
        except FileNotFoundError:
            self.load_sample_data()  # 파일이 없으면 샘플 데이터를 로드


    def display_recipe(self, item):
        recipe_name = item.text().split(' (')[0]  # Extract name from item text

        print(f"Searching for recipe: {recipe_name}")  # Debugging line

        for recipe in self.recipes:
            if recipe.name == recipe_name:
                print(f"Recipe found: {recipe_name}")  # Debugging line
                self.current_recipe = recipe
                break
        else:
            print("Recipe not found")
            return  # Exit if the recipe is not found

        # Update ingredients list view
        ingredient_model = QtGui.QStandardItemModel()
        for ingredient, amount in self.current_recipe.ingredients:
            item = QtGui.QStandardItem(f"{ingredient}: {amount}")
            ingredient_model.appendRow(item)
        self.ingredient_view.setModel(ingredient_model)

        # Update instructions list view
        instruction_model = QtGui.QStandardItemModel()
        for step in self.current_recipe.instructions:
            item = QtGui.QStandardItem(step)
            instruction_model.appendRow(item)
        self.instruction_view.setModel(instruction_model)

        # Update reviews list widget
        self.review_list_widget.clear()
        for rating, comment in self.current_recipe.reviews:
            review_text = f"Rating: {rating}, Comment: {comment}"
            self.review_list_widget.addItem(review_text)

        self.stackedWidget.setCurrentIndex(1)


    def submit_review(self):
        if self.current_recipe:
            rating = self.review_spinbox.value()
            comment = self.review_text.toPlainText()
            
            self.current_recipe.add_review(rating, comment)
            self.review_spinbox.setValue(0)
            self.review_text.clear()

            self.update_review_list()
            self.update_recipe_list()
            self.save_recipes()

    def update_recipe_list(self):
        self.listWidget.clear()
        for recipe in self.recipes:
            self.listWidget.addItem(f"{recipe.name} ({recipe.average_rating():.1f})")

    def add_new_recipe_from_web(self):
        url, ok_url = QtWidgets.QInputDialog.getText(self, 'New Recipe from Web', 'Enter recipe URL:')
        if ok_url and url:
            name, ingredients, instructions = self.scrape_recipe(url)
            if name and ingredients and instructions:
                print(f"Adding new recipe: {name}")  # Debugging line
                new_recipe = Recipe(name, ingredients, instructions)
                self.recipes.append(new_recipe)
                self.update_recipe_list()
                self.save_recipes()
                self.display_recipe(self.listWidget.item(self.listWidget.count() - 1))  # Display newly added recipe


    def add_new_recipe_manually(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("새로운 레시피 작성")

        layout = QtWidgets.QVBoxLayout()

        name_label = QtWidgets.QLabel("레시피 이름:")
        name_edit = QtWidgets.QLineEdit()
        layout.addWidget(name_label)
        layout.addWidget(name_edit)

        ingredient_label = QtWidgets.QLabel("재료 (재료: 양, ...):")
        ingredient_edit = QtWidgets.QLineEdit()
        layout.addWidget(ingredient_label)
        layout.addWidget(ingredient_edit)

        instruction_label = QtWidgets.QLabel("조리법 (step 1, step 2, ...):")
        instruction_edit = QtWidgets.QLineEdit()
        layout.addWidget(instruction_label)
        layout.addWidget(instruction_edit)

        button_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        button_box.accepted.connect(lambda: self.save_manual_recipe(name_edit.text(), ingredient_edit.text(), instruction_edit.text(), dialog))
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)

        dialog.setLayout(layout)
        dialog.exec_()

    def save_manual_recipe(self, name, ingredients_str, instructions_str, dialog):
        ingredients = [tuple(item.strip().split(':')) for item in ingredients_str.split(',') if ':' in item]
        instructions = [step.strip() for step in instructions_str.split(',') if step.strip()]

        if name and ingredients and instructions:
            new_recipe = Recipe(name, ingredients, instructions)
            self.recipes.append(new_recipe)
            self.update_recipe_list()  # 리스트를 업데이트합니다.
            self.display_recipe(self.listWidget.item(self.listWidget.count() - 1))  # 마지막 레시피를 디스플레이
            self.stackedWidget.setCurrentIndex(1)  # 레시피 상세 페이지로 이동
            self.save_recipes()
        dialog.accept()

    def scrape_recipe(self, url):
        try:
            response = requests.get(url)
            soup = BeautifulSoup(response.text, 'html.parser')

            # Define the list of kitchen tools to exclude
            tools_keywords = ["볼", "나이프", "뒤집개", "주방나이프", "냄비", "숟가락", "칼", "도마", "그릇", "젓가락", "조리용나이프", "소스볼", "프라이팬", "요리집게", "요리스푼", "접시"]

            # Extract name
            name = soup.title.get_text(strip=True)
            print(f"Extracted Name: {name}")

            # Extract ingredients
            ingredients = soup.select('.ready_ingre3 ul li')
            ingredient_list = [ingredient.get_text(strip=True).replace("구매", "").strip() for ingredient in ingredients]
            print(f"Extracted Ingredients: {ingredient_list}")

            # Filter out tools from ingredients
            filtered_ingredients = [item for item in ingredient_list if not any(tool in item for tool in tools_keywords)]

            # Extract instructions
            steps = soup.select('.view_step_cont')
            filtered_steps = []
            for step in steps:
                step_text = step.get_text(strip=True)
                for tool in tools_keywords:
                    step_text = step_text.replace(tool, "")
                step_text = step_text.replace(" ,", "").replace(", ,", "").replace(" , ", "").strip(", ")
                filtered_steps.append(step_text.strip())
            print(f"Extracted Instructions: {filtered_steps}")

            return name, filtered_ingredients, filtered_steps
        except Exception as e:
            print(f"Failed to scrape the recipe: {e}")
            return None, None, None



    def delete_recipe(self):
        if self.current_recipe:
            reply = QtWidgets.QMessageBox.question(self, 'Confirm Deletion',
                f"Are you sure you want to delete '{self.current_recipe.name}'?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)

            if reply == QtWidgets.QMessageBox.Yes:
                self.recipes.remove(self.current_recipe)
                self.current_recipe = None
                self.update_recipe_list()
                self.ingredient_view.setModel(QtGui.QStandardItemModel())
                self.instruction_view.setModel(QtGui.QStandardItemModel())
                self.review_list_widget.clear()
                self.save_recipes()
        else:
            print("No recipe selected for deletion")



def main():
    app = QtWidgets.QApplication(sys.argv)
    window = RecipeBookApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
