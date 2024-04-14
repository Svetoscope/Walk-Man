import threading
import time
from queue import Queue
from ComArduino import ComArduino
from UserInterface import UserInterface
from PyQt5.QtWidgets import QApplication

# Define the Control class
class Control:
    def __init__(self):
        # Initialize ComArduino instance
        self.comArduino = ComArduino()
        # Start communication with Arduino or OpenRB
        self.comArduino.start()

    # Method to send a message to Arduino or OpenRB
    def send_message(self, message: str):
        """
        List of valid messages :
        <A> : Driving motors at speed 1
        <B> : Driving motors at speed 2
        <C> : Driving motors at speed 3
        <N> : Neutral state
        <P> : Parking robot
        <F> : Asking for feedback
        """
        self.comArduino.send_to_arduino(message)

    # Method to get the answer from Arduino or OpenRB
    def get_answer(self) -> str:
        """
        List of valid answers :
        <A> : Driving motors at speed 1
        <B> : Driving motors at speed 2
        <C> : Driving motors at speed 3
        <N> : Neutral state
        <P> : Parking robot
        <R> : Ready to start another sequence
        """
        return self.comArduino.msg_received
        

# Function to handle switch cases for Arduino or OpenRB answers
def msg_switch_case(arduino_answer) -> int:
    if arduino_answer == 'D1' or arduino_answer == 'D2' or arduino_answer == 'D3' or arduino_answer == 'N' or arduino_answer == 'P':
        # Wait for the sequence to be finished
        return 0
    elif arduino_answer == 'R':
        # Ready to start another sequence
        return 1
    else:
        # Error
        return -1

# Function to scan UI queue
def scan_ui_queue(self, my_queue):
    while True:
        if self.UI_queue.empty():
            break
        else:
            my_queue.put(self.UI_queue.get())

# Function to run the user interface
def run_user_interface(ui_closed_flag, ui_queue, ui_start_flag, ui_update_flag):
    # Initialization of the user interface
    app = QApplication([])
    window = UserInterface(ui_queue, ui_start_flag, ui_update_flag)
    window.show()
    app.exec_()
    ui_closed_flag.set()

# Main function
def main() -> None:
    # Create variables linked to the thread of the UI
    ui_queue = Queue()
    ui_update_flag = threading.Event()
    ui_start_flag = threading.Event()
    ui_closed_flag = threading.Event()
    ui_thread = threading.Thread(target=run_user_interface, args=(ui_closed_flag, ui_queue, ui_start_flag, ui_update_flag))
    ui_thread.start()

    # Initialize Control instance
    controller = Control()

    error_count = 0
    # INIT COM
    message = 'F'
    controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)

    message = 'F'
    controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)

    checked_answer_flag = False
    allow_new_command_flag = False
    last_walking_speed = ""
    keep_walking_flag = False

    while not ui_closed_flag.is_set():
        # print("Loop")
        if controller.comArduino.new_msg_flag:
            controller.comArduino.new_msg_flag = False
            answer = controller.get_answer()
            print("Arduino reply : ", answer)
            state = msg_switch_case(answer)
            checked_answer_flag = False

        if ui_start_flag.is_set() and not checked_answer_flag:
            checked_answer_flag = True
            allow_new_command_flag = True

            if message != 'F' and answer != message:  # Means last sequence asked didn't start
                error_count += 1
            else:
                # print('State : ', state)
                if state == 1:  # Ready to start another sequence
                    print('Ready to start another sequence')
                    error_count = 0
                    if ui_queue.empty() and keep_walking_flag:
                        print('Keep walking')
                        message = last_walking_speed
                        keep_walking_flag = False
                    else:
                        print('Wait new command')
                        message = ui_queue.get()
                    
                    ui_update_flag.set()
                    if message == 'A' or message == 'B' or message == 'C':
                        last_walking_speed = message
                        keep_walking_flag = True
                    else:
                        keep_walking_flag = False
                elif state == 0:  # Wait for the sequence to be finished
                    print('Wait for the sequence to be finished')
                    error_count = 0
                    message = 'F'
                else:
                    print('Invalid Arduino answer')
                    error_count += 1
                    message = 'F'

            if error_count > 3:
                break

        if ui_start_flag.is_set() and checked_answer_flag and allow_new_command_flag:
            controller.send_message(message)
            print("Message sent : ", message)
            allow_new_command_flag = False

        time.sleep(1)

    ui_thread.join()
    return


if __name__ == "__main__":
    main()
