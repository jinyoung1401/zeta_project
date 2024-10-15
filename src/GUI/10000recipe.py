import requests
from bs4 import BeautifulSoup

# 크롤링할 레시피 URL
url = "https://www.10000recipe.com/recipe/6963110"

# 페이지 요청 및 HTML 파싱
response = requests.get(url)
soup = BeautifulSoup(response.text, 'html.parser')

# '구매' 키워드를 사용해 구매 버튼 제거
ingredients = soup.select('.ready_ingre3 ul li')
ingredient_list = [ingredient.get_text(strip=True).replace("구매", "").strip() for ingredient in ingredients]

# 조리도구로 간주할 키워드 리스트
tools_keywords = ["볼", "나이프", "뒤집개", "주방나이프", "냄비", "숟가락", "칼", "도마", "그릇", "젓가락", "조리용나이프", "소스볼", "프라이팬", "요리집게", "요리스푼", "접시"]

# 재료에서 조리도구 제외 필터링
filtered_ingredients = [item for item in ingredient_list if not any(tool in item for tool in tools_keywords)]

# 조리법 크롤링
steps = soup.select('.view_step_cont')

# 조리법에서 조리도구 관련 단어만 제거하고, 쉼표와 공백도 처리
filtered_steps = []
for step in steps:
    step_text = step.get_text(strip=True)
    for tool in tools_keywords:
        step_text = step_text.replace(tool, "")
    # 쉼표와 공백 제거
    step_text = step_text.replace(" ,", "").replace(", ,", "").replace(" , ", "").strip(", ")
    filtered_steps.append(step_text.strip())

# 결과 출력
print("재료:")
for item in filtered_ingredients:
    print(item)

print("\n조리법:")
for idx, step in enumerate(filtered_steps, 1):
    print(f"{idx}. {step}")
