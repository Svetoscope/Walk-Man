import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QComboBox, QListWidget, QHBoxLayout
from queue import Queue
from PyQt5.QtCore import QTimer
import threading
import time


class UserInterface(QWidget):
    def __init__(self, ui_queue, ui_start_flag, ui_update_flag):
        super().__init__()

        self.UI_queue = ui_queue
        self.UI_start_flag = ui_start_flag
        self.UI_update_flag = ui_update_flag

        # Create widgets
        self.label = QLabel('Selected Sequence: ')
        self.button_add = QPushButton('Add Sequence to Queue')
        self.label_status = QLabel('Status: STOP')
        self.button_start_stop = QPushButton('Start/Stop Queue')
        self.dropdown = QComboBox()
        self.dropdown.addItems(['Right Leg', 'Left Leg', 'Stand Up'])
        self.list_widget = QListWidget()

        # Connect button click events to methods
        self.button_add.clicked.connect(self.on_add_button_click)
        self.button_start_stop.clicked.connect(self.on_start_stop_button_click)

        # Set up layout
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_add)
        button_layout.addWidget(self.button_start_stop)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.label)
        main_layout.addWidget(self.dropdown)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.label_status)
        main_layout.addWidget(self.list_widget)

        # Set the layout for the main window
        self.setLayout(main_layout)

        # Start a timer to regularly check for updates in the UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)

    def select_sequence(self) -> str:
        selected_option = self.dropdown.currentText()
        if selected_option == 'Right Leg':
            return 'R'
        elif selected_option == 'Left Leg':
            return 'L'
        elif selected_option == 'Stand Up':
            return 'S'

    def on_add_button_click(self):
        sequence_to_add = self.select_sequence()
        self.UI_queue.put(sequence_to_add)
        self.list_widget.addItem(sequence_to_add)

    def on_start_stop_button_click(self):
        if not self.UI_start_flag.is_set():
            self.UI_start_flag.set()
            self.label_status.setText('Status: START')
        else:
            self.UI_start_flag.clear()
            self.label_status.setText('Status: STOP')

    def update_ui(self):
        if self.UI_update_flag.is_set():
            self.list_widget.takeItem(0)
            self.UI_update_flag.clear()

"""
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QComboBox
from queue import Queue

class UserInterface(QWidget):
    UI_queue = Queue()

    def __init__(self):
        super().__init__()

        # Create widgets
        self.label = QLabel('Selected Sequence : ')
        self.button = QPushButton('Add Sequence to Queue')
        self.dropdown = QComboBox()
        self.dropdown.addItems(['Right Leg', 'Left Leg', 'Stand Up'])

        # Connect button click event to a method
        self.button.clicked.connect(self.on_button_click)

        # Set up layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)
        layout.addWidget(self.dropdown)

        # Set the layout for the main window
        self.setLayout(layout)

    def select_sequence(self) -> str:
        selected_option = self.dropdown.currentText()
        if selected_option == 'Right Leg':
            return 'R'
        elif selected_option == 'Left Leg':
            return 'L'
        elif selected_option == 'Stand Up':
            return 'S'

    def on_button_click(self):
        sequence_to_add = self.select_sequence()
        self.UI_queue.put(sequence_to_add)
"""