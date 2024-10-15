import sys
import json
import requests
from bs4 import BeautifulSoup
from PyQt5 import QtWidgets, QtGui, QtCore, uic

class RecipeScraper(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi('recipe.ui', self)  # .ui 파일 로드

        # 버튼 클릭 이벤트 연결
        self.pushButton.clicked.connect(self.get_url_from_user)
        self.pushButton_2.clicked.connect(self.go_back)
        self.pushButton_4.clicked.connect(self.add_new_recipe)
        self.pushButton_5.clicked.connect(self.delete_recipe)
        self.pushButton_3.clicked.connect(self.go_back)

        # 모델 설정
        self.model_ingredients = QtGui.QStandardItemModel()
        self.listView.setModel(self.model_ingredients)

        self.model_steps = QtGui.QStandardItemModel()
        self.listView_2.setModel(self.model_steps)

        self.load_recipes()  # 저장된 레시피를 로드하고 표시

    def get_url_from_user(self):
        # URL 입력 받기
        url, ok = QtWidgets.QInputDialog.getText(self, 'URL 입력', '레시피 URL을 입력하세요:')
        if ok and url:
            self.fetch_recipe(url)

    def fetch_recipe(self, url):
        try:
            response = requests.get(url)
            soup = BeautifulSoup(response.text, 'html.parser')

            # 재료와 조리법 크롤링
            ingredients = soup.select('.ready_ingre3 ul li')
            ingredient_list = [ingredient.get_text(strip=True).replace("구매", "").strip() for ingredient in ingredients]
            tools_keywords = ["볼", "나이프", "뒤집개", "주방나이프", "냄비", "숟가락", "칼", "도마", "그릇", "젓가락", "조리용나이프", "소스볼", "프라이팬", "요리집게", "요리스푼", "접시"]
            filtered_ingredients = [item for item in ingredient_list if not any(tool in item for tool in tools_keywords)]

            steps = soup.select('.view_step_cont')
            filtered_steps = []
            for step in steps:
                step_text = step.get_text(strip=True)
                for tool in tools_keywords:
                    step_text = step_text.replace(tool, "")
                step_text = step_text.replace(" ,", "").replace(", ,", "").replace(" , ", "").strip(", ")
                filtered_steps.append(step_text.strip())

            # 페이지 전환
            self.stackedWidget.setCurrentIndex(1)  # 페이지 2로 전환

            # UI 업데이트
            self.model_ingredients.clear()
            for item in filtered_ingredients:
                list_item = QtGui.QStandardItem(item)
                self.model_ingredients.appendRow(list_item)

            self.model_steps.clear()
            for idx, step in enumerate(filtered_steps, 1):
                list_item = QtGui.QStandardItem(f"{idx}. {step}")
                self.model_steps.appendRow(list_item)

            # 레시피를 저장
            recipe_name, ok = QtWidgets.QInputDialog.getText(self, '레시피 이름 입력', '레시피의 이름을 입력하세요:')
            if ok and recipe_name:
                self.save_recipe(recipe_name, filtered_ingredients, filtered_steps)

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, '오류', f'레시피를 가져오는 중 오류가 발생했습니다: {e}')

    def save_recipe(self, name, ingredients, steps):
        try:
            recipes = self.load_recipes_from_file()
            
            if not isinstance(recipes, dict):
                raise ValueError("레시피 데이터가 올바르지 않습니다.")
            
            if name in recipes:
                QtWidgets.QMessageBox.warning(self, '경고', '이미 같은 이름의 레시피가 저장되어 있습니다.')
                return
            
            recipes[name] = {'ingredients': ingredients, 'steps': steps}
            
            with open('recipes.json', 'w', encoding='utf-8') as f:
                json.dump(recipes, f, ensure_ascii=False, indent=4)
            
            self.update_recipe_list()
        
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, '오류', f'레시피를 저장하는 중 오류가 발생했습니다: {str(e)}')


    def load_recipes(self):
        try:
            recipes = self.load_recipes_from_file()
            self.update_recipe_list(recipes)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, '오류', f'저장된 레시피를 불러오는 중 오류가 발생했습니다: {e}')

    def load_recipes_from_file(self):
        try:
            with open('recipes.json', 'r', encoding='utf-8') as f:
                data = json.load(f)
                # 데이터가 딕셔너리인지 확인
                if isinstance(data, dict):
                    return data
                else:
                    # 예상과 다른 형식의 데이터가 들어오면 빈 딕셔너리 반환
                    return {}
        except FileNotFoundError:
            return {}  # 파일이 없는 경우 빈 딕셔너리 반환
        except json.JSONDecodeError:
            return {}  # JSON 디코딩 오류 발생 시 빈 딕셔너리 반환

    def load_recipe_details(self, recipe_name):
        try:
            recipes = self.load_recipes_from_file()
            if not isinstance(recipes, dict) or recipe_name not in recipes:
                QtWidgets.QMessageBox.warning(self, '경고', '선택한 레시피가 목록에 없습니다.')
                return

            recipe = recipes[recipe_name]
            ingredients = recipe.get('ingredients', [])
            steps = recipe.get('steps', [])

            # UI 업데이트
            self.model_ingredients.clear()
            for item in ingredients:
                list_item = QtGui.QStandardItem(item)
                self.model_ingredients.appendRow(list_item)

            self.model_steps.clear()
            for idx, step in enumerate(steps, 1):
                list_item = QtGui.QStandardItem(f"{idx}. {step}")
                self.model_steps.appendRow(list_item)

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, '오류', f'레시피 세부 정보를 불러오는 중 오류가 발생했습니다: {str(e)}')

    def update_recipe_list(self, recipes=None):
        if recipes is None:
            recipes = self.load_recipes_from_file()
        
        if not isinstance(recipes, dict):
            QtWidgets.QMessageBox.critical(self, '오류', '레시피 데이터가 올바르지 않습니다.')
            return
        
        self.listWidget.clear()
        for recipe_name in recipes.keys():
            self.listWidget.addItem(recipe_name)

    def go_back(self):
        self.stackedWidget.setCurrentIndex(0)  # 페이지 1로 전환
        self.update_recipe_list()  # 페이지 1에서 레시피 목록을 업데이트

    def on_recipe_selected(self, item):
        recipe_name = item.text()
        self.load_recipe_details(recipe_name)

    def add_new_recipe(self):
        # 여기에 새로운 레시피 등록 로직 구현
        pass

    def delete_recipe(self):
        selected_items = self.listWidget.selectedItems()
        if not selected_items:
            QtWidgets.QMessageBox.warning(self, '경고', '삭제할 레시피를 선택하세요.')
            return

        selected_item = selected_items[0]
        recipe_name = selected_item.text()
        recipes = self.load_recipes_from_file()
        if recipe_name in recipes:
            del recipes[recipe_name]
            with open('recipes.json', 'w', encoding='utf-8') as f:
                json.dump(recipes, f, ensure_ascii=False, indent=4)
            self.update_recipe_list()
        else:
            QtWidgets.QMessageBox.warning(self, '경고', '선택한 레시피가 저장된 레시피 목록에 없습니다.')

    def setup_ui(self):
        # 리스트 위젯의 항목 선택 시 이벤트 연결
        self.listWidget.itemClicked.connect(self.on_recipe_selected)
        
    def on_recipe_selected(self, item):
        recipe_name = item.text()
        self.load_recipe_details(recipe_name)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RecipeScraper()
    window.show()
    sys.exit(app.exec_())
