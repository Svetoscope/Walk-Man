# PyQt5 widgets and layout elements
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QSlider, QSizePolicy, QSpacerItem
# PyQt5 core functionalities
from PyQt5.QtCore import QTimer, Qt

# Define the user interface class
class UserInterface(QWidget):
    # Initialize the user interface with required parameters
    def __init__(self, ui_queue, ui_start_flag, ui_update_flag):
        super().__init__()

        # Set the window title
        self.setWindowTitle('WalkMan')

        # Create a spacer for layout
        spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        # Store references to communication queue and flags
        self.UI_queue = ui_queue
        self.UI_start_flag = ui_start_flag
        self.UI_update_flag = ui_update_flag

        # Gear slider setup
        self.gear_selected = 0
        self.gear_label_0 = QLabel('1')
        self.gear_label_1 = QLabel('2')
        self.gear_label_2 = QLabel('3')
        self.gear_slider = QSlider(Qt.Horizontal)
        self.gear_slider.setMinimum(0)
        self.gear_slider.setMaximum(2)
        self.gear_slider.setTickInterval(1)
        self.gear_slider.setTickPosition(QSlider.TicksBelow)
        self.gear_slider.valueChanged.connect(self.gear_slider_changed)

        # Create layout for gear labels
        gear_lbl_layout = QHBoxLayout()
        gear_lbl_layout.addWidget(self.gear_label_0)
        gear_lbl_layout.addItem(spacer)
        gear_lbl_layout.addWidget(self.gear_label_1)
        gear_lbl_layout.addItem(spacer)
        gear_lbl_layout.addWidget(self.gear_label_2)
        gear_lbl_layout.setAlignment(Qt.AlignHCenter)

        # Create layout for gear controls
        gear_layout = QVBoxLayout()
        gear_layout.addLayout(gear_lbl_layout)
        gear_layout.addWidget(self.gear_slider)

        # State slider setup
        self.state_selected = 0
        self.state_label_0 = QLabel('Neutral')
        self.state_label_1 = QLabel('Park')
        self.state_label_2 = QLabel('Drive')
        self.state_slider = QSlider(Qt.Horizontal)
        self.state_slider.setMinimum(0)
        self.state_slider.setMaximum(2)
        self.state_slider.setTickInterval(1)
        self.state_slider.setTickPosition(QSlider.TicksBelow)
        self.state_slider.valueChanged.connect(self.state_slider_changed)

        # Create layout for state labels
        state_lbl_layout = QHBoxLayout()
        state_lbl_layout.addWidget(self.state_label_0)
        state_lbl_layout.addItem(spacer)
        state_lbl_layout.addWidget(self.state_label_1)
        state_lbl_layout.addItem(spacer)
        state_lbl_layout.addWidget(self.state_label_2)
        state_lbl_layout.setAlignment(Qt.AlignHCenter)

        # Create layout for state controls
        state_layout = QVBoxLayout()
        state_lbl_layout.setSpacing(0)
        state_layout.addLayout(state_lbl_layout)
        state_layout.addWidget(self.state_slider)

        # Center display
        self.button_start_stop = QPushButton('Start/Stop')
        self.button_start_stop.setStyleSheet("")
        self.button_start_stop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.button_start_stop.clicked.connect(self.on_start_stop_button_click)

        self.current_label = QLabel('Current command: ')
        self.next_label = QLabel('Next command: ')
        queue_layout = QVBoxLayout()
        queue_layout.addWidget(self.current_label)
        queue_layout.addWidget(self.next_label)

        self.current_label_val = QLabel('N')
        self.next_label_val = QLabel(' ')
        queue_layout_val = QVBoxLayout()
        queue_layout_val.addWidget(self.current_label_val)
        queue_layout_val.addWidget(self.next_label_val)

        center_layout = QHBoxLayout()
        center_layout.addItem(spacer)
        center_layout.addWidget(self.button_start_stop)
        center_layout.addItem(spacer)
        center_layout.addLayout(queue_layout)
        center_layout.addLayout(queue_layout_val)
        center_layout.addItem(spacer)
        
        # Set up main layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(gear_layout)
        main_layout.addItem(spacer)
        main_layout.addLayout(center_layout)
        main_layout.addItem(spacer)
        main_layout.addLayout(state_layout)

        # Set the layout for the main window
        self.setLayout(main_layout)

        # Connect window resize event to adjust font size
        self.resizeEvent = self.on_resize
        
        # Start a timer to regularly check for updates in the UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)

    # Method to handle window show event
    def showEvent(self, event):
        self.showMaximized()  # Maximize the window when it's shown

    # Method to adjust font size recursively for all child widgets
    def adjust_font_size_recursive(self, widget, font_size):
        font = widget.font()
        font.setPointSize(font_size)
        widget.setFont(font)

        # Recursively adjust font size for child widgets
        for child_widget in widget.findChildren(QWidget):
            self.adjust_font_size_recursive(child_widget, font_size)

    # Method to handle window resize event and adjust font size
    def on_resize(self, event):
        font_size = self.height() // 35  # Calculate font size based on window height
        for widget in self.findChildren(QWidget):
            self.adjust_font_size_recursive(widget, font_size)

    # Method to generate the message based on the gear box selection
    def gear_box(self) -> str:
        gear = self.gear_selected
        if gear == 0:
            return 'A'
        elif gear == 1:
            return 'B'
        elif gear == 2:
            return 'C'
        
    # Method to handle gear slider value change
    def gear_slider_changed(self, value):
        self.gear_selected = value
        if self.state_selected == 2:
            sequence_to_add = self.gear_box()
            if not self.UI_queue.empty():
                self.UI_queue.queue.clear()
            self.UI_queue.put(sequence_to_add)
            self.next_label_val.setText(sequence_to_add)

    # Method to handle state slider value change
    def state_slider_changed(self, value):
        self.state_selected = value
        if self.state_selected == 0:
            sequence_to_add = 'N'
        elif self.state_selected == 1:
            sequence_to_add = 'P'
        elif self.state_selected == 2:
            sequence_to_add = self.gear_box()
        if not self.UI_queue.empty():
            self.UI_queue.queue.clear()
        self.UI_queue.put(sequence_to_add)
        self.next_label_val.setText(sequence_to_add)

    # Method to handle start/stop button click
    def on_start_stop_button_click(self):
        if not self.UI_start_flag.is_set():
            self.UI_start_flag.set()
            self.button_start_stop.setStyleSheet("background-color: green") # Set background color to green
        else:
            self.UI_start_flag.clear()
            self.button_start_stop.setStyleSheet("") # Set background color to default

    # Method to update UI elements
    def update_ui(self):
        if self.UI_update_flag.is_set():
            if self.next_label_val.text() != ' ':
                self.current_label_val.setText(self.next_label_val.text())
            self.next_label_val.setText(' ')
            self.UI_update_flag.clear()
