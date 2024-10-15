import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5 import uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi('robot_gui.ui', self)  # UI 파일 로드

        # QGraphicsView 위젯을 가져옵니다.
        self.graphics_view = self.findChild(QGraphicsView, 'graphicsView')

        # QGraphicsScene을 생성하고 QGraphicsView에 설정합니다.
        self.scene = QGraphicsScene()
        self.graphics_view.setScene(self.scene)

        # 맵 이미지 파일 경로
        map_path = '/home/jinyoung/zeta_ws/src/zeta2_edu_autonomous/zeta2_navigation/maps/school/slam_toolbox_handong3.pgm'

        # QPixmap을 사용하여 맵 이미지를 로드합니다.
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"Failed to load map image from {map_path}")
        else:
            # QGraphicsPixmapItem을 생성하고 씬에 추가합니다.
            pixmap_item = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(pixmap_item)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
