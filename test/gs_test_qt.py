from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton, QLineEdit, QVBoxLayout, QWidget
import sys
from Pyro5.api import Proxy

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("Guidance access")

        self.text_field = QLineEdit(self)
        self.text_field.setText('TARGET "Tables/Table 1"')

        self.button = QPushButton("Set", self)
        self.button.clicked.connect(self.print_text)
        self.button2 = QPushButton("Abort", self)
        self.button2.clicked.connect(self.abort)
        self.button3 = QPushButton("Fail Ack", self)
        self.button3.clicked.connect(self.failAck)

        layout = QVBoxLayout()
        layout.addWidget(self.text_field)
        layout.addWidget(self.button)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)

        container = QWidget()
        container.setLayout(layout)

        self.gs = Proxy("PYRONAME:gnc.gs")

        self.setCentralWidget(container)

    def print_text(self):
        txt = self.text_field.text()
        print(txt)
        self.gs = Proxy("PYRONAME:gnc.gs")
        self.gs.setMission(txt)

    def abort(self):
        # self.text_field.setText('ABORT')
        self.gs = Proxy("PYRONAME:gnc.gs")
        self.gs.setMission('ABORT')

    def failAck(self):
        # self.text_field.setText('ABORT')
        gs = Proxy("PYRONAME:gnc.gs")
        gs.failAck()
        cs = Proxy("PYRONAME:gnc.cs")
        cs.failAck()
        ns = Proxy("PYRONAME:gnc.ns")
        ns.failAck()

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec_()
