from PyQt6.QtCore import Qt
from PyQt6.QtGui import QMovie, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Create a QMovie object and set the GIF file path
        movie = QMovie("/Users/jom/Downloads/ah-shit-here-we-go-again-ah-shit.gif")

        # Create a QLabel object and set the QMovie object as its movie
        label = QLabel()
        label.setMovie(movie)

        # Start the QMovie object
        movie.start()

        # Create a QVBoxLayout object and add the QLabel widget to it
        layout = QVBoxLayout()
        layout.addStretch()
        layout.addWidget(label)
        layout.addStretch()

        # Create a QWidget object and set the QVBoxLayout object as its layout
        widget = QWidget()
        widget.setLayout(layout)

        # Set the central widget of the main window to the QWidget object
        self.setCentralWidget(widget)

        # Center the window on the screen
        screen = QApplication.primaryScreen()
        screen_size = screen.size()
        window_size = self.size()
        x = (screen_size.width() - window_size.width()) / 2
        y = (screen_size.height() - window_size.height()) / 2
        self.move(x, y)

if __name__ == "__main__":
    app = QApplication([])
    screen_count = app.screens()
    if len(screen_count) > 1:
        second_screen = screen_count[1]
        geometry = second_screen.geometry()
        window = MainWindow()
        window.move(geometry.x(), geometry.y())
        window.setBaseSize(1600,2560)
        window.show()
        window.showFullScreen()
    app.exec()